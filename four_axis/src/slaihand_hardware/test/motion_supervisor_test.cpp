// No ROS nodes, serial descriptors or hardware. Deterministic supervisor tests.
#include "slaihand_hardware/motion_supervisor.hpp"
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
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
  Batch tick(double dt=.01,bool owned=false,std::array<double,4> reference={}){
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
  std::cout<<"PASS no auto-enable/home, official native codec, full URDF sectors, stagger/ramp/hold, standard reference guard, global faults, stale/cached/UNKNOWN stop cases, feedforward ramp/boundary/write state\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
