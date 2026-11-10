// Offline only: exercise Jazzy's URDF parser and the actual fake hardware plugin.
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <hardware_interface/component_parser.hpp>
#include "slaihand_hardware/openarm_joint_fake.hpp"

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Pass the canonical or expanded URDF path");
    std::ifstream input(argv[1]);
    require(input.good(), "Cannot read URDF");
    std::string xml((std::istreambuf_iterator<char>(input)), {});
    if (xml.find("<ros2_control") == std::string::npos) {
      std::string control = "<ros2_control name='OfflineLimitTest' type='system'>"
        "<hardware><plugin>slaihand_hardware/OpenArmJointFake</plugin>"
        "<param name='transport'>fake</param></hardware>";
      for (int i = 1; i <= 4; ++i) {
        control += "<joint name='slaihand_joint" + std::to_string(i) + "'>"
          "<param name='codec_profile'>" + (i <= 2 ? std::string("DM10010L") : std::string("DM8009")) +
          "</param><param name='sim_max_velocity'>0.5</param>"
          "<command_interface name='position'/><state_interface name='position'/>"
          "<state_interface name='velocity'/></joint>";
      }
      control += "</ros2_control>";
      xml.insert(xml.rfind("</robot>"), control);
    }
    const auto infos = hardware_interface::parse_control_resources_from_urdf(xml);
    require(infos.size() == 1, "Expected one hardware owner");
    const auto& info = infos.front();
    const double radians = std::acos(-1.0) / 180.0;
    const auto& j1 = info.limits.at("slaihand_joint1");
    require(std::abs(j1.min_position / radians + 80.0) < 1e-9, "J1 lower must be -80deg");
    require(std::abs(j1.max_position / radians - 200.0) < 1e-9, "J1 upper must be 200deg");
    const auto& j4 = info.limits.at("slaihand_joint4");
    require(std::abs(j4.min_position / radians + 50.0) < 1e-9, "J4 lower must be -50deg");
    require(std::abs(j4.max_position / radians - 90.0) < 1e-9, "J4 upper must be +90deg");
    for (const auto& joint : info.joints) {
      const auto& params = joint.command_interfaces.at(0).parameters;
      require(!params.count("min") && !params.count("max"), "Duplicate command-interface limit table");
    }
    slaihand_hardware::OpenArmJointFake hardware;
    using Callback = hardware_interface::CallbackReturn;
    require(hardware.on_init(info) == Callback::SUCCESS, "Fake initialization failed");
    auto commands = hardware.export_command_interfaces();
    auto states = hardware.export_state_interfaces();
    // Only in-memory fake feedback: +170 is allowed, the old +80 bound is gone.
    require(commands.at(0).set_value(170.0 * radians), "Cannot set fake command");
    require(hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0)) ==
            hardware_interface::return_type::OK, "Fake write failed");
    require(std::abs(states.at(0).get_optional().value() / radians - 170.0) < 1e-9, "Old +80 limit still active");
    require(commands.at(0).set_value(-150.0 * radians), "Cannot set fake command");
    require(hardware.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(10.0)) ==
            hardware_interface::return_type::OK, "Fake write failed");
    require(std::abs(states.at(0).get_optional().value() / radians + 80.0) < 1e-9, "Canonical -80 clamp missing");
    std::cout << "URDF limits -> Jazzy parser -> fake hardware verified; no CAN transport\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
