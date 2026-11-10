// Dependency-free protocol regressions; never opens a physical device.
#include <slaihand_transport/canusb_readonly.hpp>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace slaihand_transport;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
Frame reg(int axis, int rid, double value) {
  Frame f{0, {std::uint8_t(axis), 0, 0x33, std::uint8_t(rid), 0,0,0,0}};
  std::uint32_t bits;
  if (rid >= 21) { const float v = value; std::memcpy(&bits, &v, sizeof(bits)); }
  else bits = value;
  for (int j = 0; j < 4; ++j) f.data[4+j] = (bits >> (8*j)) & 255;
  return f;
}
void setup(ReadOnlyState& state, int axis) {
  const double vmax = axis < 3 ? 25. : axis == 5 ? 10. : 45.;
  const double tmax = axis < 3 ? 200. : axis == 5 ? 28. : 54.;
  for (const auto& pair : {std::pair{7,0.}, {8,double(axis)}, {9,2000.}, {10,1.},
       {21,12.5}, {22,vmax}, {23,tmax}}) {
    state.feed(reg(axis, pair.first, pair.second), 1.);
  }
}
std::array<std::uint8_t, 16> rx(const Frame& f) {
  std::array<std::uint8_t, 16> result{0xAA,0x11,8};
  for (int j = 0; j < 4; ++j) result[3+j] = (f.can_id >> (8*j)) & 255;
  std::copy(f.data.begin(), f.data.end(), result.begin()+7); result[15] = 0x55;
  return result;
}
int main(int argc, char** argv) {
  try {
    if (argc == 3 && std::string(argv[1]) == "--replay") {
      std::ifstream input(argv[2]); require(input.good(), "Cannot read offline fixture");
      std::array<Mapping,4> mapping;
      for(auto& m:mapping) require(bool(input>>m.can_id>>m.sign>>m.zero),"Missing fixture mapping");
      ReadOnlyState state(mapping);
      double at;std::uint32_t cid;std::string hex;
      while(input>>at>>cid>>hex){
        require(hex.size()==16,"Malformed fixture payload");
        Frame f{cid,{}};
        for(int j=0;j<8;++j) f.data[j]=std::stoul(hex.substr(j*2,2),nullptr,16);
        state.feed(f,at);
      }
      require(!state.conflicted(),"Real fixture has conflicting registers");
      for(int i=1;i<=4;++i){
        require(state.axis(i).feedback.has_value(),"Missing real fixture feedback");
        const auto& f=*state.axis(i).feedback;
        std::cout<<std::setprecision(17)<<i<<' '<<f.position<<' '<<f.velocity<<' '<<f.effort_estimate<<' '
          <<f.driver_temperature<<' '<<f.motor_temperature<<' '<<f.motor_state<<' '<<f.received_at<<'\n';
      }
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--packets") {
      for (int i = 1; i <= 5; ++i) {
        for (const int r : {-1,7,8,9,10,21,22,23}) {
          std::cout << std::dec << i << ' ' << r << ' ';
          for (auto byte : read_packet(i, r)) std::cout << std::hex << std::setfill('0') << std::setw(2) << int(byte);
          std::cout << '\n';
        }
      }
      return 0;
    }
    for (const auto& request : {std::pair{0,-1}, {6,-1}, {1,1}, {4,24}, {1,0xFC}, {1,-2}}) {
      bool refused = false;
      try { (void)read_packet(request.first, request.second); } catch (const std::invalid_argument&) {refused=true;}
      require(refused, "Non-whitelisted TX was accepted");
    }
    // J5 is a legitimate read axis: CAN ID5 using the same 30-byte TX layout.
    for (const int reg : {-1,7,8,9,10,21,22,23}) {
      const auto packet = read_packet(5, reg);
      require(packet[21] == 5, "J5 read request lost the axis byte");
      require(packet[23] == (reg == -1 ? 0xCC : 0x33), "J5 read request corrupted its register selector");
    }
    MotionPort controlled;
    for(const auto& bad: {Frame{6,{}},Frame{0x101,{}},Frame{0x7FF,{1,0,0x55,10,1,0,0,0}},
                         Frame{1,{255,255,255,255,255,255,255,0xFE}},Frame{4,{255,255,255,255,255,255,255,0xFB}}}){
      bool rejected=false;try{controlled.send_motor(bad);}catch(const std::invalid_argument&){rejected=true;}
      require(rejected,"Controlled transport accepted invalid axis/PV/write/zero/clear command");
    }
    { // J5 special frames pass the transport's native-command gate; the port is
      // closed, so the expected failure is the runtime port check, not validation.
      bool validation_rejected=false,closed_port=false;
      try{controlled.send_motor(Frame{5,{255,255,255,255,255,255,255,0xFC}});}
      catch(const std::invalid_argument&){validation_rejected=true;}
      catch(const std::runtime_error&){closed_port=true;}
      require(!validation_rejected&&closed_port,"J5 native enable was rejected by transport validation");
    }
    Stream stream;
    const Frame j1{0, {1,0x8F,0x99,0x7F,0xF7,0xFF,0x1E,0x1A}};
    const Frame j4{0, {4,0x7A,0xAA,0x7F,0xF7,0xFF,0x1C,0x1B}};
    const auto raw1 = rx(j1), raw4 = rx(j4);
    const std::uint8_t noise[]{1,2,0xAA,3,4};
    require(stream.feed(noise, sizeof(noise)).empty(), "Noise became a frame");
    require(stream.feed(raw1.data(), 7).empty(), "Incomplete frame emitted");
    const auto frames = stream.feed(raw1.data()+7, 9);
    require(frames.size() == 1 && frames[0].data == j1.data, "Split frame lost");
    auto bad = raw1; bad[15] = 0;
    stream.feed(bad.data(), bad.size());
    auto parsed = stream.feed(raw4.data(), raw4.size());
    require(parsed.size() == 1 && parsed[0].data == j4.data, "Recovery after bad footer failed");
    auto ext = raw1; ext[5] = 0xFF;
    require(stream.feed(ext.data(), ext.size()).empty(), "Extended CAN ID accepted");

    ReadOnlyState state({Mapping{1,1,1.}, {2,1,0.}, {3,1,0.}, {4,-1,-0.4}});
    state.feed(j1, 1.); require(!state.axis(1).feedback, "Decoded before measured ranges");
    for (int i = 1; i <= 4; ++i) setup(state, i);
    state.feed(j1, 2.); state.feed(j4, 2.01);
    require(state.axis(1).feedback && state.axis(4).feedback, "Shared CAN ID0 overwrote another axis");
    const auto& f = *state.axis(4).feedback;
    const double motor_position = 0x7AAA * 25. / 65535 - 12.5;
    require(std::abs(f.position - (-1.*(motor_position+0.4))) < 1e-12, "Position sign/zero mismatch");
    require(std::abs(f.effort_estimate + (2047.*108./4095.-54.)) < 1e-12, "Torque estimate mapping mismatch");
    require(f.motor_state == 0 && f.driver_temperature == 28 && f.motor_temperature == 27, "Diagnostic fields mismatch");
    auto wrong = j1; wrong.can_id = 1; state.feed(wrong, 4.);
    require(state.axis(1).feedback->received_at == 2., "Wrong reply CAN ID updated state");
    wrong = j4; wrong.data[0] = 5; state.feed(wrong, 4.);
    require(state.axis(4).feedback->received_at == 2.01, "J5 updated a four-axis state");
    for (int i = 2; i <= 3; ++i) { auto f2 = j1; f2.data[0] = i; state.feed(f2, 2.); }
    require(state.complete(2.1, .25) && !state.complete(3., .25), "Freshness is not per-axis");
    state.feed(reg(1, 21, 25.), 4.);
    require(state.conflicted() && !state.axis(1).feedback && !state.ready(1), "Changing PMAX did not invalidate state");
    ReadOnlyState invalid({Mapping{1,1,0.}, {2,1,0.}, {3,1,0.}, {4,1,0.}});
    invalid.feed(reg(2, 23, std::numeric_limits<double>::quiet_NaN()), 1.);
    require(invalid.conflicted(), "NaN register accepted");
    { // Five-axis state: J5 uses its own measured DM4340 PMAX/VMAX/TMAX and the
      // same canonical sign/zero transform. J5 feedback must never land in J4.
      const std::array<Mapping,5> m5{Mapping{1,1,1.}, {2,1,0.}, {3,1,0.}, {4,-1,-0.4},
                                     Mapping{5,1,2.5389105058}};
      ReadOnlyState five(m5);
      require(five.axis_count()==5, "Five-axis mapping lost");
      for (int i = 1; i <= 5; ++i) setup(five, i);
      const double pmax = 12.5, vmax = 10., tmax = 28., q_motor = .25;
      const auto p = std::uint16_t(std::llround((q_motor + pmax) * 65535. / (2 * pmax)));
      const auto v = std::uint16_t(std::llround((.2 + vmax) * 4095. / (2 * vmax)));
      const auto t = std::uint16_t(std::llround((3. + tmax) * 4095. / (2 * tmax)));
      const Frame j5{0, {std::uint8_t((2 << 4) | 5), std::uint8_t(p >> 8), std::uint8_t(p & 255), std::uint8_t(v >> 4),
        std::uint8_t(((v & 15) << 4) | (t >> 8)), std::uint8_t(t & 255), 35, 31}};
      for (int i = 1; i <= 4; ++i) { auto frame = j1; frame.data[0] = std::uint8_t(i); five.feed(frame, 3.); }
      five.feed(j5, 3.);
      require(five.axis(5).feedback.has_value(), "J5 feedback was not decoded");
      const auto& f5 = *five.axis(5).feedback;
      require(std::abs(f5.position - (q_motor - 2.5389105058)) < 2 * pmax / 65535. + 1e-12, "J5 position sign/zero mismatch");
      require(std::abs(f5.velocity - .2) < 2 * vmax / 4095. + 1e-12, "J5 velocity did not use the DM4340 VMAX");
      require(std::abs(f5.effort_estimate - 3.) < 2 * tmax / 4095. + 1e-12, "J5 torque did not use the DM4340 TMAX");
      require(f5.motor_state == 2 && f5.driver_temperature == 35 && f5.motor_temperature == 31,
              "J5 diagnostics mismatch");
      require(five.complete(3.1, .25), "Five-axis state is not complete");
      state.feed(j5, 3.);
      require(state.axis(4).feedback->received_at == 2.01, "Four-axis state consumed a J5 frame");
      bool too_many = false;
      try { ReadOnlyState invalid(std::vector<Mapping>{Mapping{1,1,0}, {2,1,0}, {3,1,0}, {4,1,0}, {5,1,0}, {6,1,0}}); }
      catch (const std::invalid_argument&) { too_many = true; }
      require(too_many, "Six-axis read-only state was accepted");
    }
    std::cout << "PASS readonly whitelist, split/noisy framing, real payloads, ID0 demux, runtime ranges, sign/zero, stale/conflict rejection, five-axis J5 read/state\n";
    return 0;
  } catch (const std::exception& e) {std::cerr << e.what() << '\n'; return 1;}
}
