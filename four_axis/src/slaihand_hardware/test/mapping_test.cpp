// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/mapping.hpp"
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <iostream>
#include <limits>

int main() {
  namespace dm = openarm::damiao_motor;
  for (const double sign : {-1.0, 1.0}) {
    const slaihand_hardware::JointMapping mapping{sign, 0.43};
    mapping.validate();
    for (const double q : {-1.0, 0.0, 0.52}) {
      if (std::abs(mapping.to_joint(mapping.to_motor(q)) - q) > 1e-12) return 1;
      for (const auto profile : {dm::MotorType::DM10010L, dm::MotorType::DM8009, dm::MotorType::DM4340}) {
        const dm::Motor motor(profile, 0, 0);
        const auto packet = dm::CanPacketEncoder::create_mit_control_command(
          motor, {0.0, 0.0, mapping.to_motor(q), 0.0, 0.0});
        if (packet.data.size() != 8 || packet.send_can_id != 0) return 2;
        const auto code = (static_cast<unsigned>(packet.data[0]) << 8) | packet.data[1];
        const double pmax = dm::Motor::get_limit_param(profile).pMax;
        const double decoded = code * (2.0*pmax)/65535.0 - pmax;
        if (std::abs(mapping.to_joint(decoded) - q) > 2.0*pmax/65535.0 + 1e-12) return 3;
      }
    }
  }
  for (const auto invalid : {slaihand_hardware::JointMapping{0,0},
         slaihand_hardware::JointMapping{1,std::numeric_limits<double>::quiet_NaN()}}) {
    try { invalid.validate(); return 4; } catch (const std::invalid_argument&) {}
  }
  std::cout << "PASS signed zero-offset round trips, invalid mappings, and three upstream MIT codecs\n";
  return 0;
}
