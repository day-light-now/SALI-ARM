// Deterministic J5 DM4340 codec/profile tests. No serial port, CAN socket,
// motor, or real calibration is opened or fabricated here.
#include "slaihand_hardware/native_codec.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace slaihand_hardware;
using namespace slaihand_transport;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
AxisState profile(double pmax,double vmax,double tmax){
  AxisState state;
  state.registers[4]=pmax;state.registers[5]=vmax;state.registers[6]=tmax;
  return state;
}
int main(){try{
  const auto motor=native_motor(5);
  require(motor.get_motor_type()==dm::MotorType::DM4340,"J5 is not assigned the DM4340 codec");
  require(motor.get_send_can_id()==5,"J5 native send CAN id is not 5");
  const auto limit=dm::Motor::get_limit_param(dm::MotorType::DM4340);
  require(limit.pMax==12.5&&limit.vMax==10&&limit.tMax==28,"DM4340 profile constants changed");
  for(int axis:{0,6,99}){
    bool refused=false;try{(void)native_motor(axis);}catch(const std::invalid_argument&){refused=true;}
    require(refused,"Native codec accepted an axis outside J1-J5");
  }
  // Measured J5 registers must match DM4340, not J1/J2 (25/200) or J3/J4 (45/54).
  check_native_profile(5,profile(12.5,10,28));
  for(const auto& wrong:{profile(12.5,25,200),profile(12.5,45,54),profile(12.5,10,54),
                         profile(12.5,45,28),profile(12.4,10,28)}){
    bool refused=false;try{check_native_profile(5,wrong);}catch(const std::invalid_argument&){refused=true;}
    require(refused,"J5 profile accepted non-DM4340 PMAX/VMAX/TMAX");
  }
  {AxisState missing=profile(12.5,10,28);missing.registers[6].reset();
   bool refused=false;try{check_native_profile(5,missing);}catch(const std::invalid_argument&){refused=true;}
   require(refused,"J5 profile accepted a missing measured TMAX");}
  bool refused=false;try{check_native_profile(4,profile(12.5,10,28));}catch(const std::invalid_argument&){refused=true;}
  require(refused,"J4 accepted the J5 DM4340 profile");

  const auto enable=native_special(5,true),disable=native_special(5,false);
  require(enable.can_id==5&&disable.can_id==5,"J5 special frame CAN id is not 5");
  require(enable.data[7]==0xFC&&disable.data[7]==0xFD,"J5 special frame command byte is wrong");
  for(int j=0;j<7;++j)require(enable.data[j]==0xFF&&disable.data[j]==0xFF,"J5 special frame padding changed");

  // J5 MIT commands use DM4340 scaling and the measured mapping. This test's
  // zero is an explicit input, not a claim about the physical J5 calibration.
  const Mapping map{5,1,2.5389105058};
  for(double q:{-0.5,0.,0.5}){
    const auto frame=native_command(5,map,q,160,5);
    require(frame.can_id==5,"J5 MIT frame CAN id is not 5");
    const auto decoded=decode_native(5,frame.data);
    require(std::abs(decoded.q-(map.zero+q))<=2*12.5/65535.+1e-12,"J5 MIT position round trip failed");
    require(std::abs(decoded.kp-160)<.2&&std::abs(decoded.kd-5)<.001,"J5 MIT gains round trip failed");
  }
  {const Mapping negative{5,-1,0.};
   const auto decoded=decode_native(5,native_command(5,negative,.3,0,0).data);
   require(std::abs(negative.sign*(decoded.q-negative.zero)-.3)<=2*12.5/65535.+1e-12,
           "J5 negative sign mapping failed");}
  for(const Mapping invalid:{Mapping{5,0,0},Mapping{5,1,std::numeric_limits<double>::quiet_NaN()}}){
    bool invalid_refused=false;try{(void)native_command(5,invalid,0,0,0);}catch(const std::invalid_argument&){invalid_refused=true;}
    require(invalid_refused,"J5 command accepted an unspecified sign/zero calibration");
  }
  for(const double tau:{28.001,-28.001}){
    bool tau_refused=false;try{(void)native_command(5,map,0,0,0,tau);}catch(const std::invalid_argument&){tau_refused=true;}
    require(tau_refused,"J5 command accepted torque outside the DM4340 TMAX");
  }
  bool range_refused=false;try{(void)native_command(5,map,13.,0,0);}catch(const std::invalid_argument&){range_refused=true;}
  require(range_refused,"J5 command accepted a position outside DM4340 PMAX");

  const auto packet=read_packet(5,-1);
  require(packet[21]==5&&packet[23]==0xCC,"J5 read packet axis/refresh selector is wrong");
  std::cout<<"PASS J5 DM4340 profile validation, CAN5 special/MIT frames, sign/zero requirement, read request and no J3/J4 throttles\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
