// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include <slaihand_transport/canusb_readonly.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace slaihand_hardware {
namespace dm=openarm::damiao_motor;
inline dm::Motor native_motor(int axis) {
  if(axis<1||axis>4) throw std::invalid_argument("Native codec only supports J1-J4");
  return dm::Motor(axis<=2?dm::MotorType::DM10010L:dm::MotorType::DM8009,axis,0);
}
inline void check_native_profile(int axis,const slaihand_transport::AxisState& state) {
  const auto p=dm::Motor::get_limit_param(native_motor(axis).get_motor_type());
  const double expected[]{p.pMax,p.vMax,p.tMax};
  for(int j=0;j<3;++j) if(!state.registers[j+4]||std::abs(*state.registers[j+4]-expected[j])>1e-6) {
    throw std::invalid_argument("Measured PMAX/VMAX/TMAX do not match the official codec profile");
  }
}
inline slaihand_transport::Frame native_special(int axis,bool enable) {
  const auto motor=native_motor(axis);
  const auto packet=enable?dm::CanPacketEncoder::create_enable_command(motor):dm::CanPacketEncoder::create_disable_command(motor);
  slaihand_transport::Frame out{packet.send_can_id,{}};std::copy_n(packet.data.begin(),8,out.data.begin());return out;
}
struct NativeDecoded {double q,dq,kp,kd,tau;};
inline NativeDecoded decode_native(int axis,const std::array<std::uint8_t,8>& b) {
  const auto p=dm::Motor::get_limit_param(native_motor(axis).get_motor_type());
  return {((b[0]<<8)|b[1])*2*p.pMax/65535.-p.pMax,
    ((b[2]<<4)|(b[3]>>4))*2*p.vMax/4095.-p.vMax,
    (((b[3]&15)<<8)|b[4])*500./4095.,((b[5]<<4)|(b[6]>>4))*5./4095.,
    (((b[6]&15)<<8)|b[7])*2*p.tMax/4095.-p.tMax};
}
inline slaihand_transport::Frame native_command(int axis,const slaihand_transport::Mapping& map,
                                               double q,double kp,double kd,double tau=0.) {
  const auto motor=native_motor(axis);const auto p=dm::Motor::get_limit_param(motor.get_motor_type());
  const double raw=map.zero+map.sign*q;
  if(!std::isfinite(raw)||std::abs(raw)>p.pMax||!std::isfinite(kp)||kp<0||kp>500||!std::isfinite(kd)||kd<0||kd>5||
     !std::isfinite(tau)||std::abs(tau)>p.tMax) {
    throw std::invalid_argument("Invalid native MIT field; no silent clipping");
  }
  // Exactly the official OpenArm encoder. No host PD or gravity estimate; tau is
  // an explicit host feedforward supplied in the motor frame.
  const auto packet=dm::CanPacketEncoder::create_mit_control_command(motor,{kp,kd,raw,0.,tau});
  slaihand_transport::Frame out{packet.send_can_id,{}};std::copy_n(packet.data.begin(),8,out.data.begin());return out;
}
}  // namespace slaihand_hardware
