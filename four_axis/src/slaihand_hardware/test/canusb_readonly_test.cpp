// Actual ROS plugin over a generated /dev/pts fake bus; never accepts a real
// port argument. All observed TX bytes are checked against the read whitelist.
#include "slaihand_hardware/canusb_readonly.hpp"
#include <hardware_interface/component_parser.hpp>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <pluginlib/class_loader.hpp>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <thread>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>
using namespace slaihand_transport;
void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
class FakeBus {
 public:
  int master = -1;
  std::string path;
  std::array<Mapping,4> mapping;
  std::atomic<bool> running{true}, respond{true}, illegal{false};
  std::atomic<int> tx_count{0};
  std::thread worker;
  explicit FakeBus(const std::array<Mapping,4>& m) : mapping(m) {
    int slave; char name[128];
    require(::openpty(&master, &slave, name, nullptr, nullptr) == 0, "Cannot create fake PTY");
    path = name; ::close(slave);
    require(path.rfind("/dev/pts/", 0) == 0, "Test must use a newly created PTY");
    ::fcntl(master, F_SETFL, O_NONBLOCK);
    worker = std::thread([this] {run();});
  }
  ~FakeBus() {running=false; if(worker.joinable()) worker.join(); if(master>=0) ::close(master);}
  void run() {
    std::vector<std::uint8_t> buffered;
    while (running) {
      std::uint8_t data[4096]; const auto n = ::read(master, data, sizeof(data));
      if (n > 0) buffered.insert(buffered.end(), data, data+n);
      while (buffered.size() >= 30) {
        std::array<std::uint8_t,30> request;
        std::copy_n(buffered.begin(), 30, request.begin()); buffered.erase(buffered.begin(), buffered.begin()+30);
        const int axis = request[21];
        const int reg = request[23] == 0xCC ? -1 : request[24];
        try {
          if (request != read_packet(axis, reg)) {illegal=true; continue;}
        } catch (...) {illegal=true; continue;}
        ++tx_count;
        if (!respond) continue;
        std::array<std::uint8_t,8> payload{};
        if (reg != -1) {
          payload[0]=axis; payload[2]=0x33; payload[3]=reg;
          double v = reg == 7 ? 0 : reg == 8 ? axis : reg == 9 ? 2000 : reg == 10 ? 1 :
                     reg == 21 ? 12.5 : reg == 22 ? (axis<3 ? 25. : 45.) : (axis<3 ? 200. : 54.);
          std::uint32_t bits;
          if (reg >= 21) {const float f=v; std::memcpy(&bits,&f,4);} else bits=v;
          for (int j=0;j<4;++j) payload[4+j]=(bits>>(8*j))&255;
        } else {
          const double degrees[]{15,25,-10,5};
          const auto& m=mapping[axis-1];
          const auto p=std::uint16_t(std::llround((m.zero+m.sign*degrees[axis-1]*std::acos(-1.)/180.+12.5)*65535./25.));
          payload={std::uint8_t(axis),std::uint8_t(p>>8),std::uint8_t(p&255),0x7F,0xF8,0x34,
            std::uint8_t(30+axis),std::uint8_t(25+axis)};
        }
        std::array<std::uint8_t,16> reply{0xAA,0x11,8,0,0,0,0};
        std::copy(payload.begin(),payload.end(),reply.begin()+7);reply[15]=0x55;
        if (::write(master,reply.data(),reply.size()) != 16) illegal=true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
};
int main(int argc, char** argv) {
  try {
    require(argc==2,"Pass the expanded read-only model, never a hardware port");
    std::ifstream input(argv[1]);require(input.good(),"Cannot read expanded model");
    std::string xml((std::istreambuf_iterator<char>(input)),{});
    auto infos=hardware_interface::parse_control_resources_from_urdf(xml);
    require(infos.size()==1,"Expected one owner");
    auto info=infos[0];
    std::array<Mapping,4> mapping;
    for (int i=0;i<4;++i) mapping[i]={i+1,std::stoi(info.joints.at(i).parameters.at("hardware_sign")),
      std::stod(info.joints.at(i).parameters.at("zero_offset_rad"))};
    FakeBus bus(mapping);
    info.hardware_parameters["port"]=bus.path; // The model's real port is NEVER opened.
    info.hardware_parameters["log_directory"]=(std::filesystem::path(argv[1]).parent_path()/"fake_audit").string();
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader("hardware_interface", "hardware_interface::SystemInterface");
    auto loaded=loader.createSharedInstance("slaihand_hardware/CanUsbReadOnly");
    auto& hardware=*loaded;
    using Callback=hardware_interface::CallbackReturn;
    require(hardware.on_init(info)==Callback::SUCCESS,"Read-only init failed");
    require(bus.tx_count==0,"on_init sent data");
    require(hardware.export_command_interfaces().empty(),"Movement interface exported");
    auto values=hardware.export_state_interfaces();
    require(values.size()==44,"Missing four-axis diagnostics");
    require(hardware.on_configure(rclcpp_lifecycle::State{})==Callback::SUCCESS,"Fake configuration failed");
    // A second owner must fail without clearing the existing owner's exclusion.
    ReadOnlyPort contender;
    bool refused=false;try{contender.open(bus.path);}catch(...){refused=true;}
    require(refused,"Second owner acquired the same port");
    ReadOnlyPort third;refused=false;try{third.open(bus.path);}catch(...){refused=true;}
    require(refused,"Failed contender cleared exclusivity");
    require(hardware.on_activate(rclcpp_lifecycle::State{})==Callback::SUCCESS,"Feedback activation failed");
    const double expected[]{15,25,-10,5};
    for(int i=0;i<4;++i){
      require(std::abs(values[i*11].get_optional().value()*180./std::acos(-1.)-expected[i])<.012,"Model coordinate mismatch");
      require(values[i*11+7].get_optional().value()==1.,"Fresh feedback marked invalid");
      require(values[i*11+8].get_optional().value()==1.,"Mode not exposed");
      require(values[i*11+9].get_optional().value()==2000.,"TIMEOUT not exposed");
    }
    // A simulation of missing feedback must become invalid, never frozen q.
    bus.respond=false;
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    hardware.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01));
    std::this_thread::sleep_for(std::chrono::milliseconds(280));
    require(hardware.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01))==hardware_interface::return_type::OK,"Read failed");
    for(int i=0;i<4;++i){
      require(values[i*11+7].get_optional().value()==0.,"Missing feedback still valid");
      require(std::isnan(values[i*11].get_optional().value()),"Stale position published as current");
    }
    hardware.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01)); // no TX path exists
    require(hardware.on_cleanup(rclcpp_lifecycle::State{})==Callback::SUCCESS,"Cleanup failed");
    contender.open(bus.path);contender.close();
    require(!bus.illegal,"Enable/write/motion/unknown TX observed");
    auto bad=info;hardware_interface::InterfaceInfo command;command.name="position";
    bad.joints[0].command_interfaces.push_back(command);
    slaihand_hardware::CanUsbReadOnly wrong;
    require(wrong.on_init(bad)==Callback::ERROR,"Read-only hardware accepted a motion interface");
    std::cout<<"PASS ROS plugin + fake PTY: 4-axis feedback, canonical calibration, fresh/stale flags, exclusive owner, zero motion/parameter TX\n";
    return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
