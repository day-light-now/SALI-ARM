// No ROS nodes, serial descriptors or hardware. Deterministic supervisor tests.
#include "slaihand_hardware/motion_supervisor.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>
using namespace slaihand_hardware;
using namespace slaihand_transport;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
Frame reg(int axis,int rid,double value){Frame f{0,{std::uint8_t(axis),0,0x33,std::uint8_t(rid),0,0,0,0}};
  std::uint32_t bits;if(rid>=21){float v=value;std::memcpy(&bits,&v,4);}else bits=value;
  for(int j=0;j<4;++j)f.data[4+j]=(bits>>(8*j))&255;return f;}
struct Fixture {
  static std::array<Mapping,4> maps(){return {Mapping{1,1,0},{2,1,0},{3,1,0},{4,-1,0}};}
  static std::array<PositionBounds,4> bounds(){const auto r=std::acos(-1.)/180.;return {PositionBounds{-80*r,200*r},{-10*r,190*r},{-90*r,90*r},{-50*r,90*r}};}
  ReadOnlyState state{maps()};MotionSupervisor core{MotionPolicy{},bounds()};
  double at=10;std::array<double,4> q{.3,.4,.2,.1};std::array<double,4> dq{};
  std::array<bool,4> enabled{};
  int drop=0;std::array<int,4> enable_count{};
  Fixture(){for(int i=1;i<=4;++i)for(const auto& pair:{std::pair{7,0.},{8,double(i)},{9,2000.},{10,1.},{21,12.5},{22,i<3?25.:45.},{23,i<3?200.:54.}})state.feed(reg(i,pair.first,pair.second),at);fresh();}
  void feedback(int i,int code,double stamp){
    const auto m=maps()[i-1];const auto p=std::uint16_t(std::llround((m.sign*q[i-1]+12.5)*65535./25.));
    const double vmax=i<=2?25.:45.;
    const auto velocity=std::uint16_t(std::llround((m.sign*dq[i-1]+vmax)*4095./(2*vmax)));
    Frame f{0,{std::uint8_t((code<<4)|i),std::uint8_t(p>>8),std::uint8_t(p&255),
      std::uint8_t(velocity>>4),std::uint8_t(((velocity&15)<<4)|7),0xFF,30,25}};state.feed(f,stamp);
  }
  void fresh(){for(int i=1;i<=4;++i)if(i!=drop)feedback(i,enabled[i-1]?1:0,at);}
  Batch tick(double dt=.01,bool owned=false,std::vector<double> reference={}){
    at+=dt;fresh();const auto batch=core.tick(at,state,reference,owned);
    for(std::size_t j=0;j<batch.count;++j){const auto& a=batch.actions[j];core.submitted(a,at);
      if(a.role==Role::ENABLE){enabled[a.frame.can_id-1]=true;++enable_count[a.frame.can_id-1];}
      if(a.role==Role::DISABLE)enabled[a.frame.can_id-1]=false;
      if(a.role==Role::MIT&&enabled[a.frame.can_id-1])q[a.frame.can_id-1]=maps()[a.frame.can_id-1].sign*decode_native(a.frame.can_id,a.frame.data).q;
      core.written(a,at+(j+1)*.000001);
    }return batch;
  }
  void hold(){require(core.arm(at),"Arm request rejected");for(int n=0;n<220;++n)tick();
    require(core.stage()==MotionStage::HOLD&&core.hold_ready(),"Did not reach stable native hold");}
};
// Deterministic five-axis fixture. J5 uses the official DM4340 profile limits
// (PMAX 12.5, VMAX 10, TMAX 28) because those are the measured/declared values;
// the zero and sign are explicit test inputs, not a claim of real calibration.
struct FiveFixture {
  static std::array<Mapping,5> maps(){return {Mapping{1,1,0},{2,1,0},{3,1,0},{4,-1,0},{5,1,0}};}
  static std::vector<PositionBounds> bounds(){const auto r=std::acos(-1.)/180.;
    return {{-80*r,200*r},{-10*r,190*r},{-90*r,90*r},{-50*r,90*r},{-60*r,60*r}};}
  static MotionPolicy default_policy(){MotionPolicy p;p.guard.assign(5,0.);return p;}
  MotionPolicy policy;
  ReadOnlyState state{maps()};
  MotionSupervisor core;
  double at=10;
  std::array<double,5> q{.3,.4,.2,.1,.5},dq{};
  std::array<bool,5> enabled{};
  std::array<int,5> enable_count{};
  explicit FiveFixture(MotionPolicy p=default_policy(),bool j5_timeout_zero=false,bool j5_wrong_profile=false)
      :policy(std::move(p)),core(policy,bounds()){
    for(int i=1;i<=5;++i){
      const double vmax=i<=2?25.:i<=4?45.:10.;
      const double tmax=i<=2?200.:i<=4?54.:28.;
      for(const auto& pair:{std::pair{7,0.},{8,double(i)},{9,double(i==5&&j5_timeout_zero?0:2000)},{10,1.},
           {21,12.5},{22,vmax},{23,i==5&&j5_wrong_profile?54.:tmax}})state.feed(reg(i,pair.first,pair.second),at);
    }
    fresh();
  }
  void feedback(int i,int code,double stamp){
    const auto m=maps()[i-1];const double pmax=12.5;
    const auto p=std::uint16_t(std::llround((m.sign*q[i-1]+m.zero+pmax)*65535./(2*pmax)));
    const double vmax=i<=2?25.:i<=4?45.:10.;
    const auto velocity=std::uint16_t(std::llround((m.sign*dq[i-1]+vmax)*4095./(2*vmax)));
    Frame f{0,{std::uint8_t((code<<4)|i),std::uint8_t(p>>8),std::uint8_t(p&255),
      std::uint8_t(velocity>>4),std::uint8_t(((velocity&15)<<4)|7),0xFF,30,25}};state.feed(f,stamp);
  }
  void fresh(){for(int i=1;i<=5;++i)feedback(i,enabled[i-1]?1:0,at);}
  // Explicit five-axis arm/stagger/ramp/hold helper. It enables all five axes;
  // this is not a J5-only or per-axis-enable mode.
  void hold(){require(core.arm(at),"Five-axis arm request rejected");for(int n=0;n<260;++n)tick();
    require(core.stage()==MotionStage::HOLD&&core.hold_ready(),"Did not reach stable five-axis hold");}
  Batch tick(double dt=.01,bool owned=false,std::vector<double> reference={}){
    at+=dt;fresh();const auto batch=core.tick(at,state,reference,owned);
    for(std::size_t j=0;j<batch.count;++j){const auto& a=batch.actions[j];core.submitted(a,at);
      if(a.role==Role::ENABLE){enabled[a.frame.can_id-1]=true;++enable_count[a.frame.can_id-1];}
      if(a.role==Role::DISABLE)enabled[a.frame.can_id-1]=false;
      if(a.role==Role::MIT&&enabled[a.frame.can_id-1]){
        const auto m=maps()[a.frame.can_id-1];
        q[a.frame.can_id-1]=m.sign*(decode_native(a.frame.can_id,a.frame.data).q-m.zero);}
      core.written(a,at+(j+1)*.000001);
    }return batch;
  }
};
std::shared_ptr<const GravityFfModel> test_ff_model(){
  return std::make_shared<const GravityFfModel>(GravityFfModel::parse(
    "G:0:0:-9.8|L:link1:0.1:0.5:0:0"
    "|J:slaihand_joint1:base:link1:0:0:0:0:0:0:0:1:0:0:-4:4:r"
    "|J:slaihand_joint2:link1:link2:0:0:0:0:0:0:0:0:1:0:-4:4:r"
    "|J:slaihand_joint3:link2:link3:0:0:0:0:0:0:0:0:1:0:-4:4:r"
    "|J:slaihand_joint4:link3:link4:0:0:0:0:0:0:0:0:1:0:-4:4:r"));
}
void all_stop(const Batch& b){int count=0;for(std::size_t j=0;j<b.count;++j){require(b.actions[j].role!=Role::MIT&&b.actions[j].role!=Role::ENABLE,"Motion emitted after global fault");if(b.actions[j].role==Role::DISABLE)++count;}require(count==4,"Fault did not disable all axes");}
int main(){try{
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   require(!f.core.arm(f.at,true,std::numeric_limits<double>::quiet_NaN(),5.),"NaN float Kd accepted");
   require(!f.core.arm(f.at,true,5.1,5.),"Out-of-codec float Kd accepted");
   require(!f.core.arm(f.at,true,2.,0.),"Zero float duration accepted");
   require(!f.core.arm(f.at,true,2.,121.),"Over-budget float duration accepted");
   require(f.core.arm(f.at,true,2.,20.),"Configured float rejected");
   require(!f.core.arm(f.at,true,0.,1.),"Active float settings changed");
   require(f.core.float_kd()==2.&&f.core.float_seconds()==20.,"Float settings not frozen");
   for(int n=0;n<1300;++n)f.tick();
   require(f.core.stage()==MotionStage::HOLD,"Configured duration still stopped at old 12 second cap");
   const auto commands=f.tick();
   for(std::size_t n=0;n<commands.count;++n)if(commands.actions[n].role==Role::MIT){
     const auto decoded=decode_native(commands.actions[n].frame.can_id,commands.actions[n].frame.data);
     require(decoded.kp==0.&&std::abs(decoded.kd-2.)<.002,"Configured Kd not encoded");}
   for(int n=0;n<1500&&f.core.stage()==MotionStage::HOLD;++n)f.tick();
   require(f.core.stage()==MotionStage::STOPPING,"Configured float watchdog failed");
   require(std::strcmp(f.core.reason(),"GRAVITY_FLOAT_SESSION_COMPLETE")==0,"Wrong configured timeout reason");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   require(f.core.arm(f.at,true,2.,5.),"Float request rejected");
   for(int n=0;n<410;++n)f.tick();
   // Operator is actively dragging at 0.2 rad/s; readiness must come from the
   // completed compensation ramp, not from a stationary pose.
   f.dq[0]=.2;
   for(int n=0;n<100;++n)f.tick();
   require(f.core.stage()==MotionStage::HOLD&&f.core.hold_ready(),
       "Float waits for a stationary pose while the operator is dragging");}
  {Fixture f;require(f.core.arm(f.at),"Position hold request rejected");
   for(int n=0;n<100;++n)f.tick();
   f.dq[0]=.2;
   for(int n=0;n<220;++n)f.tick();
   require(!f.core.hold_ready(),"Position-mode stationary readiness gate was lost");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   require(f.core.arm(f.at,true,2.,5.),"Float request rejected");
   for(int n=0;n<410;++n)f.tick();
   f.dq[0]=1.2;
   for(int n=0;n<20;++n)f.tick();
   require(f.core.stage()==MotionStage::HOLD,
       "Hand drag at 1.2 rad/s was stopped by the position-mode feedback speed gate");}
  {Fixture f;f.hold();f.dq[0]=1.2;all_stop(f.tick());
   require(std::strcmp(f.core.reason(),"FEEDBACK_SPEED")==0,
       "Position-mode feedback speed gate was lost");}
  {Fixture f;require(!f.core.arm(f.at,true),"Float accepted without configured model and permission");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   require(f.core.arm(f.at,true),"Explicit configured float rejected");
   for(int n=0;n<410;++n)f.tick();
   require(f.core.hold_ready()&&f.core.gravity_float(),"Float ramp did not become ready");
   require(f.core.gravity_alpha()==1.&&f.core.gravity_ramp()==3.,"Wrong float compensation settings");
   f.q[0]+=.15;const auto b=f.tick();
   require(f.core.stage()==MotionStage::HOLD,"Dragging beyond position-following gate stopped float");
   for(std::size_t n=0;n<b.count;++n)if(b.actions[n].role==Role::MIT){
     const auto d=decode_native(b.actions[n].frame.can_id,b.actions[n].frame.data);
     require(d.kp==0.&&d.kd>4.99,"Float retained position stiffness or lost damping");}
   all_stop(f.tick(.01,true,f.core.target()));
   require(std::strcmp(f.core.reason(),"GRAVITY_FLOAT_REJECTS_TRAJECTORY_OWNER")==0,"Float accepted trajectory owner");
   for(int n=0;n<8;++n)f.tick();
   require(f.core.disabled_observed()&&!f.core.gravity_active(),"Float stop did not clear output");
   require(f.core.reset(f.at,f.state),"Float disabled reset failed");
   require(f.core.arm(f.at),"Normal arm after float rejected");
   require(!f.core.gravity_float(),"Normal arm inherited zero-stiffness mode");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());f.core.arm(f.at,true);
   for(int n=0;n<1300&&f.core.stage()!=MotionStage::DISABLED;++n)f.tick();
   require(f.core.disabled_observed(),"Float watchdog did not end session");
   require(std::strcmp(f.core.reason(),"GRAVITY_FLOAT_SESSION_COMPLETE")==0,"Float watchdog reason lost");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());f.core.arm(f.at,true);
   for(int n=0;n<410;++n)f.tick();
   // Hand-drag mode must not disable the motors just because the operator briefly
   // crosses the URDF soft limit; position mode keeps that termination below.
   f.q[1]=Fixture::bounds()[1].lower-.02;const auto batch=f.tick();
   require(f.core.stage()==MotionStage::HOLD,"Float stopped when dragging past the URDF limit");
   for(std::size_t n=0;n<batch.count;++n)
     require(batch.actions[n].role!=Role::DISABLE,"Float disabled axes after crossing the URDF limit");}
  {Fixture f;f.hold();f.q[1]=Fixture::bounds()[1].lower-.02;all_stop(f.tick());
   require(std::strcmp(f.core.reason(),"ACTUAL_OUTSIDE_OPERATING_LIMITS")==0,
       "Position mode lost the actual-position URDF limit termination");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());f.core.arm(f.at,true);
   for(int n=0;n<410;++n)f.tick();
   // test_ff_model links use +/-4 rad model limits; cross them while floating.
   f.q[0]=4.2;const auto batch=f.tick();
   require(f.core.stage()==MotionStage::HOLD,
       "Free drag stopped when the gravity model input crossed its URDF limit");
   for(std::size_t n=0;n<batch.count;++n)
     require(batch.actions[n].role!=Role::DISABLE,
         "Free drag disabled axes when the gravity model input crossed its URDF limit");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   f.core.arm(f.at,true,2.,20.);
   for(int n=0;n<410;++n)f.tick();
   require(f.core.hold_ready()&&f.core.gravity_float(),"Lock test float not ready");
   const auto command=f.core.command_id();
   require(!f.core.request_gravity_mode(true,command+1,f.at),"Wrong command_id lock accepted");
   const double remaining_before=f.core.remaining_s(f.at);
   // A fast drag must first decelerate under the existing float damping.
   f.dq[0]=.5;
   require(f.core.request_gravity_mode(true,command,f.at),"Fast lock request rejected");
   f.tick();
   require(f.core.gravity_lock_state()==GravityLockState::LOCKING&&f.core.gravity_float(),
       "Fast drag was latched in one cycle instead of locking/decelerating");
   require(f.core.gravity_alpha()==1.&&f.core.gravity_ramp()==3.,
       "Lock transition changed gravity feedforward");
   f.dq.fill(0.);f.tick();
   require(f.core.gravity_lock_state()==GravityLockState::LOCKED&&f.core.gravity_hold(),
       "Fresh stationary feedback did not latch gravity hold");
   const auto hold_target=f.core.hold_target();
   require(hold_target[0]==f.q[0]&&hold_target[3]==f.q[3],"Lock target is not the same-cycle feedback");
   require(f.core.remaining_s(f.at)<remaining_before,"Lock reset or extended the session timer");
   require(!f.core.request_gravity_mode(true,command,f.at),"Duplicate lock accepted");
   // Kp must ramp up; the first locked packet cannot jump to the full gain.
   auto first=f.tick();
   double first_kp=-1;
   for(std::size_t n=0;n<first.count;++n)if(first.actions[n].role==Role::MIT)
     first_kp=decode_native(first.actions[n].frame.can_id,first.actions[n].frame.data).kp;
   require(first_kp>=0&&first_kp<p.kp,"Locked packet jumped to full Kp in one cycle");
   for(int n=0;n<150;++n)f.tick();
   require(f.core.hold_gain_scale()>=.999,"Hold Kp ramp did not complete");
   f.q[0]+=.01;
   const auto target_before=f.core.target();
   auto locked=f.tick();
   require(f.core.target()==target_before,"External motion rewrote the locked target");
   require(std::abs(f.core.hold_error()[0])>.009,"Held target error not exposed");
   for(std::size_t n=0;n<locked.count;++n)if(locked.actions[n].role==Role::MIT){
     const auto d=decode_native(locked.actions[n].frame.can_id,locked.actions[n].frame.data);
     require(d.kp>150.&&d.kd>4.9,"Validated position gain was not applied after the ramp");}
   // Release ramps stiffness down while keeping gravity feedforward, then returns to float.
   Fixture r;r.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   r.core.arm(r.at,true,2.,20.);for(int n=0;n<410;++n)r.tick();
   const auto release_command=r.core.command_id();
   require(r.core.request_gravity_mode(true,release_command,r.at),"Release fixture lock rejected");
   r.tick();require(r.core.gravity_lock_state()==GravityLockState::LOCKED,"Release fixture did not lock");
   for(int n=0;n<150;++n)r.tick();
   require(r.core.request_gravity_mode(false,release_command,r.at),"Release request rejected");
   r.tick();require(r.core.gravity_lock_state()==GravityLockState::RELEASING,
       "Release did not enter a ramped state");
   for(int n=0;n<150;++n)r.tick();
   require(r.core.gravity_float()&&!r.core.gravity_hold()&&r.core.gravity_alpha()==1.,
       "Release did not restore float with gravity support");
   auto post=r.tick();for(std::size_t n=0;n<post.count;++n)if(post.actions[n].role==Role::MIT){
     const auto d=decode_native(post.actions[n].frame.can_id,post.actions[n].frame.data);
     require(d.kp==0.&&d.kd<=5.,"Released float retained position stiffness");}
   require(!r.core.request_gravity_mode(false,release_command,r.at),"Duplicate release accepted");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   f.core.arm(f.at,true,2.,20.);for(int n=0;n<410;++n)f.tick();
   const auto command=f.core.command_id();
   require(f.core.request_gravity_mode(true,command,f.at),"Early-release lock rejected");
   f.tick();require(f.core.gravity_lock_state()==GravityLockState::LOCKED,"Early-release fixture not locked");
   for(int n=0;n<10;++n)f.tick();
   const double gain_before=f.core.hold_gain_scale();
   require(gain_before>0.&&gain_before<.2,"Early release did not test a partial gain");
   require(f.core.request_gravity_mode(false,command,f.at),"Early release request rejected");
   auto first=f.tick();
   require(f.core.gravity_lock_state()==GravityLockState::RELEASING,"Early release did not enter RELEASING");
   double kp_after=-1;
   for(std::size_t n=0;n<first.count;++n)if(first.actions[n].role==Role::MIT)
     kp_after=decode_native(first.actions[n].frame.can_id,first.actions[n].frame.data).kp;
   require(kp_after>=0&&kp_after<=gain_before*p.kp+1.,
       "Early release jumped back to full Kp instead of starting at the actual gain");
   require(f.core.hold_gain_scale()<=gain_before,"Early release gain increased");}
  {Fixture f;MotionPolicy p;p.gravity_float_enabled=true;
   f.core=MotionSupervisor(p,Fixture::bounds(),test_ff_model());
   f.core.arm(f.at,true,2.,20.);for(int n=0;n<410;++n)f.tick();
   f.q[0]=Fixture::bounds()[0].lower-.01;f.fresh();
   require(f.core.request_gravity_mode(true,f.core.command_id(),f.at),"Limit lock request not accepted");
   f.tick();
   require(f.core.gravity_float()&&f.core.gravity_lock_state()==GravityLockState::NONE,
       "Lock outside the URDF limit hardened position mode");
   require(std::strstr(f.core.lock_reason(),"LIMIT")!=nullptr,"Limit lock rejection reason lost");}
  {MotionPolicy p;p.runtime=180;p.validate();p.runtime=180.001;
   bool rejected=false;try{p.validate();}catch(const std::invalid_argument&){rejected=true;}
   require(rejected,"Session beyond 180 seconds accepted");}
  {Fixture f;f.hold();for(int n=0;n<5000;++n)f.tick();
   require(f.core.stage()==MotionStage::HOLD,"Old 45-second session cap remains");
   for(int n=0;n<14000&&f.core.stage()==MotionStage::HOLD;++n)f.tick();
   require(f.core.stage()==MotionStage::STOPPING&&std::strcmp(f.core.reason(),"FINITE_SESSION_COMPLETE")==0,
           "180-second finite session did not stop");}
  {MotionPolicy p;p.reference_speed=1.;p.validate();p.reference_speed=1.000001;
   bool rejected=false;try{p.validate();}catch(const std::invalid_argument&){rejected=true;}
   require(rejected,"Reference speed beyond authorised 1 rad/s accepted");}
  {Fixture f;f.hold();auto ref=f.core.target();
   for(int n=0;n<100;++n){ref[0]+=.0099;f.tick(.01,true,ref);}
   require(f.core.stage()==MotionStage::TRACK,"Authorised 0.99rad/s reference rejected");
   ref[0]+=.0101;all_stop(f.tick(.01,true,ref));
   require(std::strcmp(f.core.reason(),"REFERENCE_SPEED")==0,"1.01rad/s did not trigger reference speed guard");}
  {Fixture f;for(int n=0;n<10;++n)require(f.tick().count==0,"Implicit motion before explicit arm");}
  {Fixture f;f.hold();for(int i=0;i<4;++i)require(f.enable_count[i]==1,"Axis enabled repeatedly");
   const auto anchor=f.core.anchor();f.tick(.01,false,{0,0,0,0});require(f.core.target()==anchor,"Unclaimed zero commands caused homing");
   auto ref=anchor;for(int n=0;n<600;++n){ref[0]+=.0001;f.tick(.01,true,ref);}
   require(f.core.stage()==MotionStage::TRACK&&f.core.target()[0]-anchor[0]>.05,"Invented +/-2deg window still active");
   ref[0]+=.1;all_stop(f.tick(.01,true,ref));for(int n=0;n<5;++n)f.tick();
   require(f.core.disabled_observed(),"Fresh disabled observations not confirmed");require(f.core.reset(f.at,f.state),"Manual reset failed after disabled observations");}
  {Fixture f;f.q[0]=170*std::acos(-1.)/180.;f.q[1]=-9*std::acos(-1.)/180.;f.fresh();f.hold();
   require(f.core.anchor()[0]>2.9&&f.core.anchor()[1]<-.15,"Full URDF limits were narrowed or inverted");}
  for(int axis=0;axis<4;++axis)for(bool upper:{false,true}){
    Fixture f;const auto limits=Fixture::bounds();const double boundary=upper?limits[axis].upper:limits[axis].lower;
    f.q[axis]=boundary+(upper?-1:1)*3*25./65535.;f.fresh();f.hold();auto ref=f.core.target();
    for(int n=0;n<20;++n){ref[axis]=upper?std::min(boundary,ref[axis]+.0001):std::max(boundary,ref[axis]-.0001);
      auto batch=f.tick(.01,true,ref);require(f.core.stage()==MotionStage::TRACK,"Boundary target rejected inside full URDF sector");
      for(std::size_t j=0;j<batch.count;++j)if(batch.actions[j].role==Role::MIT){
        const int i=batch.actions[j].frame.can_id-1;const auto d=decode_native(i+1,batch.actions[j].frame.data);
        const double q=Fixture::maps()[i].sign*d.q;
        require(q>=limits[i].lower-1e-12&&q<=limits[i].upper+1e-12,"Quantized native target crossed a URDF endpoint");
      }
    }
    require(std::abs(f.q[axis]-boundary)<=25./65535.+1e-10,"Boundary representable range was narrowed by more than one code");
  }
  {Fixture f;f.q[0]=-150*std::acos(-1.)/180.;f.fresh();f.core.arm(f.at);f.tick();require(f.core.stage()==MotionStage::DISABLED&&!f.core.attempted(),"Invalid J1 sector was armed");}
  {Fixture f;f.state.feed(reg(1,9,0),f.at);f.core.arm(f.at);for(int n=0;n<320;++n)f.tick();require(!f.core.attempted(),"Invalid TIMEOUT reached enable");}
  {Fixture f;f.hold();f.at+=.001;f.feedback(2,8,f.at);all_stop(f.core.tick(f.at,f.state,f.core.target(),false));}
  {Fixture f;f.hold();all_stop(f.tick(.08));}
  {Fixture f;f.hold();auto ref=f.core.target();ref[3]=std::numeric_limits<double>::quiet_NaN();all_stop(f.tick(.01,true,ref));}
  {Fixture f;f.hold();f.drop=2;for(int n=0;n<10&&f.core.stage()!=MotionStage::STOPPING;++n)f.tick();
   require(f.core.stage()==MotionStage::STOPPING,"Stale axis did not globally stop");for(int n=0;n<40;++n)f.tick();
   require(f.core.stage()==MotionStage::UNKNOWN&&!f.core.disabled_observed(),"Missing stop feedback falsely confirmed disable");
   require(!f.core.arm(f.at),"UNKNOWN re-armed");require(f.tick().count==0,"UNKNOWN sent active queries or motion");}
  {Fixture f;f.hold();f.core.stop("TEST_STOP",f.at,true);auto b=f.core.tick(f.at,f.state,f.core.target(),false);
   for(std::size_t j=0;j<b.count;++j)f.core.written(b.actions[j],f.at+.001);
   for(int i=1;i<=4;++i)f.feedback(i,0,f.at-.01);
   f.core.tick(f.at+.02,f.state,f.core.target(),false);require(!f.core.disabled_observed(),"Cached state0 accepted as fresh stop confirmation");}
  {Fixture f;f.hold();f.enabled[0]=false;all_stop(f.tick());}
  {Fixture f;f.hold();f.state.feed(reg(3,23,55),f.at);all_stop(f.tick());}
  {Fixture f;f.core.stop("OPERATOR",f.at,true);all_stop(f.core.tick(f.at,f.state,{},false));}
  {MotionPolicy bad;bad.guard[0]=-1;bool refused=false;try{bad.validate();}catch(...){refused=true;}require(refused,"Negative torque guard accepted");}
  // User disabled all host torque thresholds. Native request > old 8/10/2/2
  // and feedback > those values must not abort. Other guards stay exercised above.
  for(int axis=0;axis<4;++axis){
    Fixture f;f.hold();const auto held=f.q;auto ref=f.core.target();
    for(int n=0;n<17;++n){ref[axis]+=.004;f.q=held;f.tick(.01,true,ref);}
    require(f.core.stage()==MotionStage::TRACK,"Removed request torque threshold still stops motion");
    require(std::abs(f.core.request_estimate()[axis])>10.,"Regression did not exceed the old request threshold");
    f.q=held;f.at+=.001;f.fresh();
    const int i=axis+1;const auto mapping=Fixture::maps()[axis];
    const auto p=std::uint16_t(std::llround((mapping.sign*f.q[axis]+12.5)*65535./25.));
    const double tmax=i<=2?200.:54.;const auto t=std::uint16_t(std::llround((12.+tmax)*4095./(2*tmax)));
    Frame frame{0,{std::uint8_t(0x10|i),std::uint8_t(p>>8),std::uint8_t(p&255),
      0x7F,std::uint8_t(0xF0|(t>>8)),std::uint8_t(t&255),30,25}};
    f.state.feed(frame,f.at);f.core.tick(f.at,f.state,ref,true);
    require(f.core.stage()==MotionStage::TRACK,"Removed feedback torque threshold still stops motion");
  }
  { // feedforward: independent ramp, boundary preservation, write receipts
    auto model=test_ff_model();
    double fast=0,slow=0;
    for(int index=0;index<2;++index){Fixture f;MotionPolicy p;p.gravity_alpha=.5;p.gravity_ramp=index==0?1.:10.;
      f.core=MotionSupervisor(p,Fixture::bounds(),model);f.hold();
      (index==0?fast:slow)=f.core.ff_applied()[0];}
    require(fast<-.2,"1s feedforward ramp produced no written feedforward");
    require(std::abs(slow)<std::abs(fast)*.6,"10s feedforward ramp was not independent from the 1s gain ramp");
    int crossings=0;
    for(int axis=0;axis<4;++axis)for(bool upper:{false,true}){
      Fixture f;MotionPolicy p;p.gravity_alpha=.5;
      const auto limits=Fixture::bounds();const double boundary=upper?limits[axis].upper:limits[axis].lower;
      f.q[axis]=boundary+(upper?-1:1)*3*25./65535.;f.fresh();
      f.core=MotionSupervisor(p,limits,model);f.hold();auto ref=f.core.target();
      for(int n=0;n<20;++n){ref[axis]=upper?std::min(boundary,ref[axis]+.0001):std::max(boundary,ref[axis]-.0001);
        const auto batch=f.tick(.01,true,ref);
        for(std::size_t j=0;j<batch.count;++j)if(batch.actions[j].role==Role::MIT){
          const int i=batch.actions[j].frame.can_id-1;
          const double q=Fixture::maps()[i].sign*decode_native(i+1,batch.actions[j].frame.data).q;
          if(q<limits[i].lower-1e-12||q>limits[i].upper+1e-12)++crossings;}}
    }
    require(crossings==0,"non-zero feedforward re-encode lost the boundary correction");
    {Fixture f;MotionPolicy p;p.gravity_alpha=.5;p.gravity_ramp=0.;
     f.core=MotionSupervisor(p,Fixture::bounds(),model);f.hold();
     require(f.core.gravity_active(),"feedforward inactive while holding");
     require(std::abs(f.core.ff_applied()[0])>0.1,"no written feedforward while holding");
     const auto before=f.core.ff_applied();
     f.at+=.01;f.fresh();const auto batch=f.core.tick(f.at,f.state,f.core.target(),false);
     for(std::size_t j=0;j<batch.count;++j){const auto& a=batch.actions[j];
       if(a.role==Role::MIT&&a.frame.can_id==1)continue;f.core.written(a,f.at+.001);}
     require(f.core.ff_applied()[0]==before[0],"axis reported a write that did not happen");
     f.core.stop("TEST_STOP",f.at,true);for(int n=0;n<8;++n)f.tick();
     require(!f.core.gravity_active(),"feedforward still active after stop");
     require(f.core.stage()==MotionStage::DISABLED,"test stop did not reach disabled");
     for(double v:f.core.ff_applied())require(v==0.,"stale applied feedforward after stop");
     for(double v:f.core.ff_candidate())require(v==0.,"stale candidate feedforward after stop");}
  }
  { // Five-axis default is deliberately J5-calibration-pending: whole-arm motion
    // must be refused before any FC/MIT frame, while five-axis feedback is present.
    FiveFixture f;
    require(f.core.axis_count()==5&&f.state.axis_count()==5,"Five-axis active set lost");
    require(f.state.complete(f.at,.25),"J5 feedback was not included in the five-axis state");
    require(!f.core.arm(f.at),"Five-axis arm accepted with J5 calibration pending");
    require(std::strcmp(f.core.reason(),"AXIS_CALIBRATION_PENDING")==0,"Missing J5 calibration reason lost");
    for(int n=0;n<20;++n)require(f.tick().count==0,"Motion frame emitted while J5 calibration was pending");
    require(!f.core.attempted(),"J5 calibration-pending preflight enabled an axis");}
  { // Explicit J5 qualification is still not enough when the drive watchdog is 0.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    FiveFixture f(p,true,false);
    require(f.core.arm(f.at),"Explicit five-axis preflight did not start");
    bool enabled=false;
    for(int n=0;n<20;++n){const auto b=f.tick();for(std::size_t j=0;j<b.count;++j)enabled|=b.actions[j].role==Role::ENABLE;}
    require(!enabled,"J5 TIMEOUT0 reached an enable frame");
    require(!f.core.attempted(),"J5 TIMEOUT0 preflight attempted enable");
    require(f.core.stage()==MotionStage::DISABLED,"J5 TIMEOUT0 did not end in DISABLED");
    require(std::strstr(f.core.reason(),"TIMEOUT")!=nullptr,"J5 TIMEOUT0 rejection reason lost");}
  { // J5 must match the DM4340 PMAX/VMAX/TMAX profile, not J3/J4 values.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    FiveFixture f(p,false,true);
    require(f.core.arm(f.at),"Explicit five-axis preflight did not start");
    bool enabled=false;
    for(int n=0;n<20;++n){const auto b=f.tick();for(std::size_t j=0;j<b.count;++j)enabled|=b.actions[j].role==Role::ENABLE;}
    require(!enabled,"J5 with J3/J4 profile reached enable");
    require(!f.core.attempted()&&f.core.stage()==MotionStage::DISABLED,"J5 wrong profile did not refuse motion");
    require(std::strstr(f.core.reason(),"PMAX/VMAX/TMAX")!=nullptr,"J5 profile rejection reason lost");}
  { // A five-axis stop must cover J5 even though J5 cannot be armed by default.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    FiveFixture f(p);
    f.core.stop("TEST_FIVE_AXIS_STOP",f.at,true);
    const auto batch=f.tick();
    std::set<int> disabled;for(std::size_t j=0;j<batch.count;++j)if(batch.actions[j].role==Role::DISABLE)disabled.insert(int(batch.actions[j].frame.can_id));
    require(disabled.size()==5&&disabled.count(5)==1,"Five-axis stop did not disable J1-J5");
    for(int n=0;n<8&&!f.core.disabled_observed();++n)f.tick();
    require(f.core.disabled_observed(),"Five-axis stop did not observe all five axes disabled");
    require(f.core.reset(f.at,f.state),"Five-axis disabled reset did not cover J5");}
  { // A) per-axis configuration validation and legacy scalar compatibility.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    p.kp_axis={160.,150.,140.,130.,120.};p.kd_axis={5.,4.,3.,2.,1.};
    p.validate(5);
    const auto bracketed=parse_gain_axis_parameter("[160, 150.5, 140, 130, 120]","kp_axis");
    require(bracketed.size()==5&&std::abs(bracketed[1]-150.5)<1e-12,"Bracketed axis list was not parsed");
    require(parse_gain_axis_parameter("  ","kp_axis").empty(),"Blank axis list did not mean not supplied");
    require(parse_gain_axis_parameter("160 150 140 130 120","kp_axis").size()==5,"Space-separated axis list was not parsed");
    for(const char* bad:{"160,150,140,130,120,","160,,140,130,120",
                         "160,150,140,130,nan","160,150,140,130,inf","160;150;140;130;120",
                         "[160,150,140,130,120"}){
      bool refused=false;try{parse_gain_axis_parameter(bad,"kp_axis");}catch(const std::invalid_argument&){refused=true;}
      require(refused,"Malformed axis list was accepted");}
    const auto rejects=[](const MotionPolicy& candidate,std::size_t axes){
      try{candidate.validate(axes);}catch(const std::invalid_argument&){return true;}return false;};
    MotionPolicy short_list;short_list.guard.assign(5,0.);short_list.kp_axis={160,150};
    require(rejects(short_list,5),"Two-entry Kp axis list accepted");
    MotionPolicy wrong_kp;wrong_kp.guard.assign(5,0.);wrong_kp.kp_axis={160,150,140,130};
    require(rejects(wrong_kp,5),"Wrong Kp axis list length accepted");
    MotionPolicy wrong_kd;wrong_kd.guard.assign(5,0.);wrong_kd.kd_axis={5,4,3,2};
    require(rejects(wrong_kd,5),"Wrong Kd axis list length accepted");
    MotionPolicy nan_kp;nan_kp.guard.assign(5,0.);
    nan_kp.kp_axis={160,150,140,130,std::numeric_limits<double>::quiet_NaN()};
    require(rejects(nan_kp,5),"NaN per-axis Kp accepted");
    MotionPolicy inf_kd;inf_kd.guard.assign(5,0.);
    inf_kd.kd_axis={5,4,3,2,std::numeric_limits<double>::infinity()};
    require(rejects(inf_kd,5),"Infinite per-axis Kd accepted");
    MotionPolicy high_kp;high_kp.guard.assign(5,0.);high_kp.kp_axis={160,150,140,130,280.0001};
    require(rejects(high_kp,5),"Per-axis Kp above the experiment 280 cap accepted");
    MotionPolicy high_kd;high_kd.guard.assign(5,0.);high_kd.kd_axis={5,4,3,2,5.0001};
    require(rejects(high_kd,5),"Per-axis Kd above the adopted 5 cap accepted");
    MotionPolicy zero_kp;zero_kp.guard.assign(5,0.);zero_kp.kp_axis={160,150,140,130,0};
    require(rejects(zero_kp,5),"Zero per-axis Kp accepted");
    MotionPolicy six;six.guard.assign(5,0.);six.kp_axis={1,2,3,4,5,6};
    require(rejects(six,5),"Six-entry axis list accepted");
    MotionPolicy four_only;four_only.guard.assign(4,0.);four_only.kp_axis={160,150,140,130};
    require(rejects(four_only,4),"Four-axis per-axis array accepted");
    MotionPolicy arrays_four_axis;arrays_four_axis.kp_axis={160,150,140,130,120};
    bool four_refused=false;
    try{MotionSupervisor core(arrays_four_axis,Fixture::bounds());}catch(const std::invalid_argument&){four_refused=true;}
    require(four_refused,"Four-axis supervisor accepted a per-axis gain list");
    // Legacy scalar policy keeps the adopted 160/5 on every axis and in the packet.
    Fixture legacy;legacy.hold();
    require(legacy.core.kp_axis().size()==4&&legacy.core.kp_axis()[3]==160.&&legacy.core.kd_axis()[3]==5.,
            "Legacy scalar policy did not fill every axis");
    require(legacy.core.gain_axis_json().find("\"kp_axis_source\":\"scalar_legacy\"")!=std::string::npos,
            "Legacy gain source was not reported as scalar_legacy");
    const auto legacy_batch=legacy.core.tick(legacy.at,legacy.state,legacy.core.target(),false);
    for(std::size_t j=0;j<legacy_batch.count;++j)if(legacy_batch.actions[j].role==Role::MIT){
      const auto d=decode_native(legacy_batch.actions[j].frame.can_id,legacy_batch.actions[j].frame.data);
      require(std::abs(d.kp-160.)<.13&&std::abs(d.kd-5.)<.002,"Legacy scalar policy packet changed");}
  }
  { // A) five-axis independent Kp/Kd reach ramp, HOLD and MIT encoding.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    p.kp_axis={150.,140.,130.,120.,110.};p.kd_axis={4.,3.5,3.,2.5,2.};
    FiveFixture f(p);
    require(f.core.kp_axis().size()==5&&f.core.kd_axis().size()==5,"Per-axis gain accessors lost the active set");
    for(std::size_t i=0;i<5;++i){
      require(std::abs(f.core.kp_axis()[i]-p.kp_axis[i])<1e-12,"Effective Kp differs from the accepted list");
      require(std::abs(f.core.kd_axis()[i]-p.kd_axis[i])<1e-12,"Effective Kd differs from the accepted list");}
    const auto gain_json=f.core.gain_axis_json();
    require(gain_json.find("\"kp_axis\":[150")!=std::string::npos,"Identity gain JSON lost per-axis Kp");
    require(gain_json.find("\"kp_axis_source\":\"per_axis\"")!=std::string::npos,"Identity Kp source is not per_axis");
    require(gain_json.find("\"kd_axis_source\":\"per_axis\"")!=std::string::npos,"Identity Kd source is not per_axis");
    require(f.core.arm(f.at),"Per-axis gain arm request rejected");
    bool ramp_checked=false;
    for(int n=0;n<200&&!ramp_checked;++n){
      const auto batch=f.tick();
      if(f.core.stage()!=MotionStage::RAMP)continue;
      double min_ratio=1e9,max_ratio=-1e9;
      for(std::size_t j=0;j<batch.count;++j)if(batch.actions[j].role==Role::MIT){
        const int axis=batch.actions[j].frame.can_id;
        const auto d=decode_native(axis,batch.actions[j].frame.data);
        require(d.kp<=p.kp_axis[axis-1]+.13,"Ramp packet exceeded the configured per-axis Kp");
        const double ratio=d.kp/p.kp_axis[axis-1];
        min_ratio=std::min(min_ratio,ratio);max_ratio=std::max(max_ratio,ratio);}
      require(max_ratio-min_ratio<.25,"Ramp gain was not one time scale of the per-axis gains");
      ramp_checked=true;}
    require(ramp_checked,"Never observed a five-axis RAMP packet");
    for(int n=0;n<240&&f.core.stage()!=MotionStage::HOLD;++n)f.tick();
    require(f.core.stage()==MotionStage::HOLD,"Per-axis ramp did not reach HOLD");
    const auto batch=f.tick();
    std::array<double,5> encoded_kp{};
    for(std::size_t j=0;j<batch.count;++j)if(batch.actions[j].role==Role::MIT){
      const int axis=batch.actions[j].frame.can_id;
      const auto d=decode_native(axis,batch.actions[j].frame.data);
      encoded_kp[axis-1]=d.kp;
      require(std::abs(d.kp-p.kp_axis[axis-1])<.13,
              (std::string("Final MIT Kp is not the per-axis value for J")+std::to_string(axis)).c_str());
      require(std::abs(d.kd-p.kd_axis[axis-1])<.002,
              (std::string("Final MIT Kd is not the per-axis value for J")+std::to_string(axis)).c_str());}
    require(encoded_kp[0]>encoded_kp[4]+1.,"Per-axis Kp list collapsed to one global value");
    // The request estimate must be derived from the same encoded per-axis packet.
    f.at+=.01;f.fresh();
    const auto estimate_batch=f.core.tick(f.at,f.state,f.core.target(),false);
    const auto estimates=f.core.request_estimate();
    for(std::size_t j=0;j<estimate_batch.count;++j)if(estimate_batch.actions[j].role==Role::MIT){
      const int axis=estimate_batch.actions[j].frame.can_id;
      const auto& fb=*f.state.axis(axis).feedback;
      const auto mapping=FiveFixture::maps()[axis-1];
      const auto d=decode_native(axis,estimate_batch.actions[j].frame.data);
      const double expected=d.kp*(d.q-(mapping.zero+mapping.sign*fb.position))+
                            d.kd*(d.dq-mapping.sign*fb.velocity)+d.tau;
      require(std::abs(estimates[axis-1]-expected)<1e-9,
              "Request estimate is inconsistent with the per-axis encoded packet");}
  }
  { // B) frozen first cause: J2 following error survives later stops/JTC errors.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    p.kp_axis={150.,140.,130.,120.,110.};p.kd_axis={4.,3.5,3.,2.5,2.};
    FiveFixture f(p);f.hold();
    const auto held=f.core.target();
    const auto command=f.core.command_id();
    const double degrees=std::acos(-1.)/180.;
    // J2 moves 6 deg away from its commanded target; every other axis stays
    // inside the unchanged 4 deg driver gate.
    f.q[1]=held[1]+6.*degrees;
    f.tick();
    require(f.core.stage()==MotionStage::STOPPING,"J2 following error did not enter STOPPING");
    require(f.core.has_first_fault(),"Following error did not freeze a first fault");
    const auto& snap=f.core.first_fault();
    require(snap.axis==2&&snap.joint=="slaihand_joint2","First fault axis is not J2");
    require(snap.code=="FOLLOWING_ERROR","First fault code is not FOLLOWING_ERROR");
    require(std::abs(snap.limit_rad-p.error)<1e-12,"Snapshot lost the 4 degree gate");
    require(std::abs(snap.error_rad-6.*degrees)<2e-3,"Snapshot error does not match the observed gap");
    require(std::abs(snap.target_rad-held[1])<1e-12,"Snapshot target is not the active target");
    require(std::abs(snap.actual_rad-(held[1]+6.*degrees))<2e-3,"Snapshot actual does not match feedback");
    require(snap.feedback_valid&&snap.feedback_age>=0.&&snap.feedback_age<=p.age+1e-9,
            "Snapshot feedback age/validity is wrong");
    require(snap.command_id==command,"Snapshot lost command_id");
    require(snap.encoded_kp.size()==5&&snap.encoded_kd.size()==5&&snap.encoded_tau_ff.size()==5,
            "Snapshot encoded gains are missing");
    for(std::size_t i=0;i<5;++i){
      require(std::abs(snap.encoded_kp[i]-p.kp_axis[i])<.13,"Snapshot encoded Kp is not the applied per-axis gain");
      require(std::abs(snap.encoded_kd[i]-p.kd_axis[i])<.002,"Snapshot encoded Kd is not the applied per-axis gain");}
    require(snap.last_complete_write_at>=0.&&snap.last_complete_write_at<=f.at,
            "Snapshot last complete write time is invalid");
    require(snap.request_estimate_at>=0.&&snap.request_estimate_at<snap.detected_at,
            "Snapshot request estimate is not the last completed active cycle");
    require(std::string(f.core.request_estimate_state())=="LAST_ACTIVE_STALE",
            "Post-stop request estimate was not marked stale");
    const double frozen_at=snap.detected_at;
    // A later OPERATOR_GLOBAL_STOP and a later JTC trajectory error must still
    // stop with priority but must not rewrite the frozen first cause.
    f.core.stop("OPERATOR_GLOBAL_STOP",f.at,true);
    f.core.stop("JTC_TRAJECTORY_ERROR",f.at);
    require(f.core.first_fault().detected_at==frozen_at&&f.core.first_fault().axis==2,
            "Later stop replaced the first cause");
    for(int n=0;n<40&&!f.core.disabled_observed();++n)f.tick();
    require(f.core.disabled_observed(),"Five-axis fault stop did not confirm all axes disabled");
    require(f.core.has_first_fault()&&f.core.first_fault().axis==2,"First cause lost after DISABLED");
    require(std::string(f.core.request_estimate_state())=="LAST_ACTIVE_STALE",
            "Stale request estimate flag lost after DISABLED");
    const auto json=f.core.first_fault_json("boot-fault-test",42);
    for(const char* needle:{"\"schema\":\"slaihand_first_fault/1\"","\"layer\":\"DRIVER\"","\"axis\":2",
        "\"joint\":\"slaihand_joint2\"","\"code\":\"FOLLOWING_ERROR\"","\"boot_id\":\"boot-fault-test\"",
        "\"cycle_id\":42","\"frozen\":true","\"request_estimate_state\":\"LAST_ACTIVE\"",
        "\"error_deg\":","\"limit_deg\":4"})
      require(json.find(needle)!=std::string::npos,(std::string("First-fault JSON missing ")+needle).c_str());
  }
  { // B) an operator stop is not fabricated into a first fault.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    FiveFixture f(p);f.hold();
    f.core.stop("OPERATOR_GLOBAL_STOP",f.at,true);
    require(!f.core.has_first_fault(),"Operator stop invented a driver first fault");
    f.tick();
    require(!f.core.has_first_fault(),"Global stop created a first fault after STOPPING");
    require(std::string(f.core.request_estimate_state())=="LAST_ACTIVE_STALE",
            "Global stop did not mark request estimates stale");
  }
  { // Explicit Kp ceiling: default stays 160; experiment opts into 280.
    MotionPolicy p;p.guard.assign(5,0.);p.kp_axis={280.,240.,160.,200.,160.};
    bool rejected=false;
    try{p.validate(5);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"Kp above the default 160 ceiling was accepted");
    p.kp_max=280.;p.validate(5);
    MotionPolicy too_high;p.guard.assign(5,0.);p.kp_max=501.;rejected=false;
    try{p.validate(5);}catch(const std::invalid_argument&){rejected=true;}
    require(rejected,"kp_max above the protocol ceiling was accepted");
  }
  { // Angle-scheduled Kp: parse, bounds, interpolation, rate limit, audit JSON.
    const auto schedule=parse_kp_schedule_parameter(
        "0:160,20:220,40:280;0:160,20:240,40:280;0:160,20:180,40:200;"
        "0:160,20:200,40:220;0:160,20:180,40:200","kp_schedule");
    require(schedule.size()==5,"Kp schedule parser lost an axis");
    require(std::abs(interpolate_kp_schedule(schedule[0],-30.)-160.)<1e-12,
            "Kp schedule negative clamp failed");
    require(std::abs(interpolate_kp_schedule(schedule[0],10.)-190.)<1e-12,
            "Kp schedule interpolation failed");
    require(std::abs(interpolate_kp_schedule(schedule[0],90.)-280.)<1e-12,
            "Kp schedule positive clamp failed");
    require(std::abs(slew_kp_gain(160.,280.,.01,80.)-160.8)<1e-12,
            "Kp schedule rate limiter failed");
    auto rejects_schedule=[&](const std::string& text){
      try{parse_kp_schedule_parameter(text,"kp_schedule");return false;}
      catch(const std::invalid_argument&){return true;}};
    require(rejects_schedule("20:160,10:140;0:160"),"Kp schedule accepted decreasing angles");
    require(rejects_schedule("0:160,0:140;0:160"),"Kp schedule accepted duplicate angles");
    require(rejects_schedule("0:nan,20:180;0:160"),"Kp schedule accepted non-finite value");
    require(rejects_schedule("0:160,;0:160"),"Kp schedule accepted empty point");

    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    p.kp_max=280.;p.kp_schedule=schedule;p.kp_schedule_slew=80.;
    p.validate(5);
    const std::array<PositionBounds,5> bounds={PositionBounds{-80*std::acos(-1.)/180.,200*std::acos(-1.)/180.},
      {-10*std::acos(-1.)/180.,190*std::acos(-1.)/180.},{-90*std::acos(-1.)/180.,90*std::acos(-1.)/180.},
      {-50*std::acos(-1.)/180.,90*std::acos(-1.)/180.},{-90*std::acos(-1.)/180.,90*std::acos(-1.)/180.}};
    MotionSupervisor s(p,bounds);
    require(s.kp_schedule_active(),"Kp schedule did not activate");
    require(s.kp_effective_axis().size()==5&&s.kp_schedule_target_axis().size()==5,
            "Kp schedule effective state missing");
    const auto json=s.gain_axis_json();
    for(const char* needle:{"\"kp_schedule_source\":\"angle_schedule\"",
        "\"kp_schedule_slew_kp_per_s\":80","\"kp_max\":280","\"kp_schedule\":"})
      require(json.find(needle)!=std::string::npos,
              (std::string("Kp schedule identity JSON missing ")+needle).c_str());
    require(json.find("\"kp_effective\"")==std::string::npos,
            "Static identity leaked dynamic effective Kp");
  }
  { // A second arm must reinitialize the schedule at the new pose.
    MotionPolicy p;p.guard.assign(5,0.);p.motion_qualified.assign(5,true);
    p.kp_max=280.;
    p.kp_schedule=parse_kp_schedule_parameter(
        "0:160,20:220,40:280;0:160,20:240,40:280;0:160;0:160,20:200,40:220;0:160",
        "kp_schedule");
    p.kp_schedule_slew=80.;
    FiveFixture f(p);f.hold();
    f.core.stop("TEST_STOP",f.at,true);
    for(int n=0;n<80&&!f.core.disabled_observed();++n)f.tick();
    require(f.core.disabled_observed(),"Schedule lifecycle stop did not reach DISABLED");
    f.q[0]=40.*std::acos(-1.)/180.;f.fresh();
    require(f.core.reset(f.at,f.state),"Schedule lifecycle reset rejected");
    require(f.core.arm(f.at),"Schedule lifecycle arm rejected");
    for(int n=0;n<80&&f.core.kp_schedule_target_axis()[0]<279.;++n)f.tick();
    require(f.core.kp_schedule_active(),"Schedule lifecycle did not activate");
    require(f.core.kp_effective_axis().size()==5&&
            std::abs(f.core.kp_effective_axis()[0]-280.)<1e-6,
            "Second arm inherited the old scheduled Kp instead of using the new 40 deg pose");
    require(f.core.kp_schedule_target_axis()[0]>=280.-1e-9,
            "Second arm schedule target is not the 40 deg endpoint");
  }
  std::cout<<"PASS no auto-enable/home, official native codec, full URDF sectors, stagger/ramp/hold, standard reference guard, global faults, stale/cached/UNKNOWN stop cases, feedforward ramp/boundary/write state, five-axis J5 calibration/TIMEOUT/profile gates, five-axis stop/reset coverage, per-axis Kp/Kd validation/ramp/hold/encoding/identity contract and frozen J2-first-fault evidence with post-stop stale request estimates\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
