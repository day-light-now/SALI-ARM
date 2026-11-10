// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/motion_supervisor.hpp"
#include <algorithm>
#include <limits>
#include <cstring>
#include <vector>

namespace slaihand_hardware {
void MotionPolicy::validate()const{
  const double values[]{kp,kd,reference_speed,feedback_speed,error,temperature,ramp,stagger,age,gap,runtime};
  for(double v:values)if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid finite motion policy");
  if(kp>160||kd>5||
     reference_speed>1||feedback_speed>1||error>4*std::acos(-1.)/180.+1e-12||temperature>50||
     ramp<1||stagger<.03||age>.06||gap>.05||runtime>180)throw std::invalid_argument("Motion policy exceeds adopted limits");
  if(!std::isfinite(gravity_alpha)||gravity_alpha<0||gravity_alpha>1||
     !std::isfinite(gravity_ramp)||gravity_ramp<0)
    throw std::invalid_argument("Invalid gravity feedforward policy");
  for(double value:guard)if(!std::isfinite(value)||value<0)
    throw std::invalid_argument("Invalid host torque guard; zero disables it");
}
MotionSupervisor::MotionSupervisor(MotionPolicy policy,std::array<PositionBounds,4> bounds,
                                   std::shared_ptr<const GravityFfModel> gravity_model)
    :p_(policy),bounds_(bounds),gravity_model_(std::move(gravity_model)){
  p_.validate();
  for(auto& b:bounds_){
    if(!std::isfinite(b.lower)||!std::isfinite(b.upper)||b.lower>=b.upper)throw std::invalid_argument("Invalid URDF limits");
  }
  enable_at_.fill(-1);confirmed_at_.fill(-1);fd_at_.fill(-1);stop_seen_.fill(-1);
}
bool MotionSupervisor::arm(double at,bool gravity_float,double float_kd,double float_seconds){
  if(stage_!=MotionStage::DISARMED||!std::isfinite(at))return false;
  if(gravity_float&&(!p_.gravity_float_enabled||!gravity_model_||gravity_model_->empty()))return false;
  if(gravity_float&&(!std::isfinite(float_kd)||float_kd<0||float_kd>5||
                    !std::isfinite(float_seconds)||float_seconds<1||float_seconds>120))return false;
  floating_=gravity_float;
  float_kd_=gravity_float?float_kd:5.;float_seconds_=gravity_float?float_seconds:5.;
  ++command_id_;started_=last_tick_=at;stage_=MotionStage::PREFLIGHT;reason_="ARM_ACCEPTED_NOT_ENABLED";
  hold_ready_=false;stop_confirmed_=false;attempted_=false;next_enable_=0;preload_mask_=0;stable_at_=-1;
  enable_at_.fill(-1);confirmed_at_.fill(-1);fd_at_.fill(-1);stop_seen_.fill(-1);stop_samples_.fill(0);
  clear_feedforward(true);return true;
}
bool MotionSupervisor::reset(double at,const slaihand_transport::ReadOnlyState& s){
  if(stage_!=MotionStage::DISABLED||!s.complete(at,p_.age))return false;
  for(int i=1;i<=4;++i)if(s.axis(i).feedback->motor_state!=0)return false;
  stage_=MotionStage::DISARMED;reason_="MANUAL_RESET_DISABLED_ONLY";attempted_=false;
  clear_feedforward(true);return true;
}
void MotionSupervisor::stop(const char* why,double at,bool explicit_stop){
  if(stage_==MotionStage::STOPPING||stage_==MotionStage::UNKNOWN)return;
  reason_=why;hold_ready_=false;stable_at_=-1;stop_confirmed_=false;
  clear_feedforward(true);
  if(!attempted_&&!explicit_stop){stage_=MotionStage::DISABLED;return;}
  stage_=MotionStage::STOPPING;stop_at_=at;fd_at_.fill(-1);stop_seen_.fill(-1);stop_samples_.fill(0);
}
void MotionSupervisor::check_observations(double at,const slaihand_transport::ReadOnlyState& s,bool initial){
  if(s.conflicted()||!s.complete(at,p_.age))throw std::runtime_error("FEEDBACK_STALE_OR_CONFLICTED");
  for(int i=1;i<=4;++i){
    const auto& a=s.axis(i);const auto& f=*a.feedback;
    check_native_profile(i,a);
    if(*a.registers[3]!=1||*a.registers[2]!=2000||*a.registers[0]!=0||*a.registers[1]!=i)
      throw std::runtime_error("REQUIRE_MIT_TIMEOUT2000_IDS");
    if(f.driver_temperature>p_.temperature||f.motor_temperature>p_.temperature)throw std::runtime_error("TEMPERATURE");
    // Position mode keeps the 1 rad/s feedback speed gate and terminates when the
    // actual position leaves the operating limits. Hand dragging is a deliberate
    // operator motion: while floating, crossing the URDF soft limit must not throw
    // and disable the arm in the operator's hands. The initial arming preflight
    // still requires all axes inside the limits. Float keeps the adopted session
    // timeout, temperature, feedback age, cycle gap, MIT protocol and global-stop
    // checks; only the actual-position-limit termination is skipped after arming.
    if(!floating_&&std::abs(f.velocity)>p_.feedback_speed)throw std::runtime_error("FEEDBACK_SPEED");
    if((!floating_||initial)&&(f.position<bounds_[i-1].lower||f.position>bounds_[i-1].upper))
      throw std::runtime_error("ACTUAL_OUTSIDE_OPERATING_LIMITS");
    if(initial){
      if(f.motor_state!=0||std::abs(f.velocity)>.05)throw std::runtime_error("REQUIRE_DISABLED_STATIONARY");
    }else{
      if(f.motor_state!=0&&f.motor_state!=1)throw std::runtime_error("MOTOR_FAULT");
      if(confirmed_at_[i-1]>=0&&f.motor_state!=1)throw std::runtime_error("UNEXPECTED_AXIS_DISABLED");
      if(!floating_&&std::abs(target_[i-1]-f.position)>p_.error)throw std::runtime_error("FOLLOWING_ERROR");
      if(p_.guard[i-1]>0&&std::abs(f.effort_estimate)>p_.guard[i-1])throw std::runtime_error("FEEDBACK_ESTIMATE_GUARD_NOT_HARD_CLAMP");
    }
  }
}
void MotionSupervisor::clear_feedforward(bool keep_history){
  ff_requested_.fill(0.);ff_candidate_.fill(0.);ff_encoded_.fill(0.);ff_applied_.fill(0.);
  ff_active_=false;
  if(!keep_history){ff_last_written_.fill(0.);ff_written_at_.fill(-1.);}
}
Batch MotionSupervisor::controls(double at,const slaihand_transport::ReadOnlyState& s,int enable_axis){
  std::array<slaihand_transport::Frame,4> frames;
  // Gravity feedforward is computed once per cycle from the measured model angles.
  // ReadOnlyState already maps feedback into the URDF model frame
  // (q_model = sign * (raw - zero)), so no extra mapping is applied here.
  std::vector<double> gravity;
  if(gravity_model_&&gravity_alpha()>0){
    std::array<double,4> q_model{};
    for(int i=1;i<=4;++i){const auto& a=s.axis(i);const auto& f=*a.feedback;
      // Hand-drag mode may cross the URDF soft limit (the actual-position stop is
      // disabled while floating). The gravity model still only accepts angles inside
      // its URDF limits, so clamp the input at the boundary and hold the boundary
      // compensation torque instead of throwing and disabling the arm in the hand.
      q_model[i-1]=floating_?std::clamp(f.position,bounds_[i-1].lower,bounds_[i-1].upper)
                            :f.position;}
    gravity=gravity_model_->torque(q_model);
  }
  // Validate all axes before even a single FC or MIT command is emitted.
  for(int i=1;i<=4;++i){
    const auto& a=s.axis(i);const auto& f=*a.feedback;
    const double gain_scale=confirmed_at_[i-1]<0?0:std::clamp((at-confirmed_at_[i-1])/p_.ramp,0.,1.);
    // Independent feedforward ramp; zero duration means immediate once the axis
    // is confirmed enabled, the gain ramp is not reused for it.
    const double ff_scale=confirmed_at_[i-1]<0?0:
        (gravity_ramp()<=0?1.:std::clamp((at-confirmed_at_[i-1])/gravity_ramp(),0.,1.));
    const double kp=floating_?0.:gain_scale*p_.kp;
    const double kd=gain_scale*(floating_?float_kd_:p_.kd);
    double command_target=target_[i-1];
    frames[i-1]=native_command(i,a.mapping,command_target,kp,kd);
    // Represent a boundary target inside the exact URDF sector. This is only
    // encoder quantization (<= one position code), not an invented angle margin.
    // The corrected model target must survive the later feedforward re-encode.
    auto encoded=decode_native(i,frames[i-1].data);
    const double encoded_model=a.mapping.sign*(encoded.q-a.mapping.zero);
    if(encoded_model<bounds_[i-1].lower||encoded_model>bounds_[i-1].upper){
      const auto profile=dm::Motor::get_limit_param(native_motor(i).get_motor_type());
      const double boundary=encoded_model<bounds_[i-1].lower?bounds_[i-1].lower:bounds_[i-1].upper;
      const bool raw_increase=(encoded_model<bounds_[i-1].lower)==(a.mapping.sign==1);
      const double code=(a.mapping.zero+a.mapping.sign*boundary+profile.pMax)*65535./(2*profile.pMax);
      const double representable=raw_increase?std::ceil(code):std::floor(code);
      const double motor_input=(representable+.25)*2*profile.pMax/65535.-profile.pMax;
      command_target=a.mapping.sign*(motor_input-a.mapping.zero);
      frames[i-1]=native_command(i,a.mapping,command_target,kp,kd);
      encoded=decode_native(i,frames[i-1].data);
      const double checked=a.mapping.sign*(encoded.q-a.mapping.zero);
      if(checked<bounds_[i-1].lower-1e-12||checked>bounds_[i-1].upper+1e-12)throw std::runtime_error("NO_REPRESENTABLE_URDF_TARGET");
    }
    const auto base=decode_native(i,frames[i-1].data);
    const double motor_q=a.mapping.zero+a.mapping.sign*f.position,motor_v=a.mapping.sign*f.velocity;
    ff_requested_[i-1]=0.;ff_candidate_[i-1]=0.;ff_encoded_[i-1]=0.;
    if(!gravity.empty()){
      // tau_protocol = sign * tau_model (virtual-work consistency, A = diag(sign)).
      const double requested=a.mapping.sign*(ff_scale*gravity_alpha()*gravity[i-1]);
      const double pd=base.kp*(base.q-motor_q)+base.kd*(base.dq-motor_v);
      const double headroom=p_.guard[i-1]>0?std::max(0.,p_.guard[i-1]-std::abs(pd)):
          dm::Motor::get_limit_param(native_motor(i).get_motor_type()).tMax;
      const double candidate=std::clamp(requested,-headroom,headroom);
      ff_requested_[i-1]=requested;ff_candidate_[i-1]=candidate;
      if(candidate!=0.){
        // The official OpenArm encoder truncates towards -tMax. Pre-bias by half a
        // code so the transmitted field is the nearest representable torque; the
        // applied value below is still decoded from the final packet.
        const auto profile=dm::Motor::get_limit_param(native_motor(i).get_motor_type());
        const double half_step=profile.tMax/4095.;
        const double encoded_input=std::clamp(candidate+half_step,-profile.tMax,profile.tMax);
        frames[i-1]=native_command(i,a.mapping,command_target,kp,kd,encoded_input);
      }
    }
    const auto d=decode_native(i,frames[i-1].data);
    // Final packet must still represent a position inside the exact URDF sector.
    const double final_model=a.mapping.sign*(d.q-a.mapping.zero);
    if(final_model<bounds_[i-1].lower-1e-12||final_model>bounds_[i-1].upper+1e-12)
      throw std::runtime_error("NO_REPRESENTABLE_URDF_TARGET");
    ff_encoded_[i-1]=d.tau;
    const double estimate=d.kp*(d.q-motor_q)+d.kd*(d.dq-motor_v)+d.tau;
    request_estimate_[i-1]=estimate;
    if(p_.guard[i-1]>0&&std::abs(estimate)>p_.guard[i-1])throw std::runtime_error("OBSERVED_STATE_REQUEST_GUARD_NOT_HARD_CLAMP");
  }
  Batch batch;if(enable_axis)batch.add(native_special(enable_axis,true),Role::ENABLE);
  for(int i=0;i<4;++i)batch.add(frames[i],Role::MIT);
  return batch;
}
Batch MotionSupervisor::stopping(double at,const slaihand_transport::ReadOnlyState& s){
  bool all=true;
  for(int i=1;i<=4;++i){
    const auto& a=s.axis(i);
    if(fd_at_[i-1]>=0&&a.feedback&&a.feedback->received_at>fd_at_[i-1]&&
       a.feedback->received_at>stop_seen_[i-1]&&at-a.feedback->received_at<=p_.age){
      stop_seen_[i-1]=a.feedback->received_at;
      if(a.feedback->motor_state==0)++stop_samples_[i-1];else stop_samples_[i-1]=0;
    }
    all&=stop_samples_[i-1]>=2&&a.feedback&&a.feedback->motor_state==0&&at-a.feedback->received_at<=p_.age;
  }
  if(all){stage_=MotionStage::DISABLED;stop_confirmed_=true;return {};}
  if(at-stop_at_>=.3){stage_=MotionStage::UNKNOWN;return {};}
  Batch b;for(int i=1;i<=4;++i)b.add(native_special(i,false),Role::DISABLE);
  for(int i=1;i<=4;++i){auto p=slaihand_transport::read_packet(i);slaihand_transport::Frame f{0x7FF,{}};
    std::copy_n(p.begin()+21,8,f.data.begin());b.add(f,Role::READ);}return b;
}
void MotionSupervisor::submitted(const Action& a,double){if(a.role==Role::ENABLE)attempted_=true;}
void MotionSupervisor::written(const Action& a,double at){
  if(a.role==Role::ENABLE){enable_at_[a.frame.can_id-1]=at;++next_enable_;}
  if(a.role==Role::DISABLE)fd_at_[a.frame.can_id-1]=at;
  if(a.role==Role::MIT){
    const int axis=a.frame.can_id-1;
    if(axis>=0&&axis<4){
      // Only a frame that reached the serial write path counts as applied. This is
      // still a host write receipt, not a drive torque confirmation.
      const auto decoded=decode_native(axis+1,a.frame.data);
      ff_applied_[axis]=decoded.tau;
      ff_last_written_[axis]=decoded.tau;
      ff_written_at_[axis]=at;
      ff_active_=gravity_model_&&gravity_alpha()>0&&
          (stage_==MotionStage::PRELOAD||stage_==MotionStage::ENABLING||
           stage_==MotionStage::RAMP||stage_==MotionStage::HOLD||stage_==MotionStage::TRACK);
    }
    if(stage_==MotionStage::PRELOAD){
      preload_mask_|=1u<<(a.frame.can_id-1);
      if(preload_mask_==15){preload_at_=at;stage_=MotionStage::ENABLING;}
    }
  }
}
Batch MotionSupervisor::tick(double at,slaihand_transport::ReadOnlyState& s,const std::array<double,4>& ref,bool owned){
  if(!std::isfinite(at)){stop("NONFINITE_CLOCK",last_tick_);return stage_==MotionStage::STOPPING?stopping(last_tick_,s):Batch{};}
  if(stage_==MotionStage::STOPPING)return stopping(at,s);
  if(stage_==MotionStage::UNKNOWN||stage_==MotionStage::DISARMED||stage_==MotionStage::DISABLED)return {};
  try{
    const double dt=at-last_tick_;
    if(at<last_tick_||(attempted_&&dt>p_.gap))throw std::runtime_error("CONTROL_CYCLE_GAP");
    last_tick_=at;
    if(at-started_>p_.runtime)throw std::runtime_error("FINITE_SESSION_COMPLETE");
    // A lost client cannot leave zero-stiffness output running indefinitely.
    if(floating_&&at-started_>float_seconds_+7.)throw std::runtime_error("GRAVITY_FLOAT_SESSION_COMPLETE");
    if(floating_&&owned)throw std::runtime_error("GRAVITY_FLOAT_REJECTS_TRAJECTORY_OWNER");
    if(stage_==MotionStage::PREFLIGHT){
      if(at-started_>3)throw std::runtime_error("PREFLIGHT_TIMEOUT");
      if(!s.complete(at,p_.age))return {};
      check_observations(at,s,true);
      for(int i=0;i<4;++i)anchor_[i]=target_[i]=s.axis(i+1).feedback->position;
      stage_=MotionStage::PRELOAD;return controls(at,s);
    }
    check_observations(at,s,false);
    if(floating_){
      for(int i=0;i<4;++i)target_[i]=s.axis(i+1).feedback->position;
    }
    if(stage_==MotionStage::PRELOAD)return controls(at,s);
    for(int i=0;i<4;++i)if(enable_at_[i]>=0&&confirmed_at_[i]<0){
      const auto& f=*s.axis(i+1).feedback;
      if(f.motor_state==1&&f.received_at>enable_at_[i])confirmed_at_[i]=at;
      else if(at-enable_at_[i]>.08)throw std::runtime_error("ENABLE_STATE_NOT_OBSERVED");
    }
    if(stage_==MotionStage::ENABLING){
      const bool previous=next_enable_==0||(confirmed_at_[next_enable_-1]>=0&&at-enable_at_[next_enable_-1]>=p_.stagger);
      if(next_enable_<4&&previous)return controls(at,s,next_enable_+1);
      if(next_enable_==4&&confirmed_at_[3]>=0)stage_=MotionStage::RAMP;
    }
    if(stage_==MotionStage::RAMP&&at-confirmed_at_[3]>=p_.ramp){stage_=MotionStage::HOLD;reason_="RAMP_COMPLETE_NOT_POSITION_ACCURACY_ACCEPTANCE";}
    if(stage_==MotionStage::HOLD||stage_==MotionStage::TRACK){
      bool stationary=true;for(int i=1;i<=4;++i)stationary&=std::abs(s.axis(i).feedback->velocity)<=.05;
      if(stationary){if(stable_at_<0)stable_at_=at;}else stable_at_=-1;
      // Hand dragging must not wait for a stationary pose: the operator is
      // deliberately moving the arm. Float readiness is the completed feedforward
      // ramp; position mode keeps the stationary+gain-ramp hold criterion.
      hold_ready_=floating_?at-confirmed_at_[3]>=gravity_ramp()
                            :stable_at_>=0&&at-stable_at_>=.5;
      if(owned){
        for(int i=0;i<4;++i){
          if(!std::isfinite(ref[i])||ref[i]<bounds_[i].lower||ref[i]>bounds_[i].upper)
            throw std::runtime_error("TARGET_OUTSIDE_URDF_LIMITS");
          if(dt<=0||std::abs(ref[i]-target_[i])>p_.reference_speed*dt+1e-7)throw std::runtime_error("REFERENCE_SPEED");
        }
        target_=ref;stage_=MotionStage::TRACK;
      }else stage_=MotionStage::HOLD;
    }
    return controls(at,s);
  }catch(const std::exception& e){
    // Exception strings above are fixed literals; retain a stable bounded reason.
    std::strncpy(reason_buffer_.data(),e.what(),reason_buffer_.size()-1);reason_buffer_.back()=0;
    stop(reason_buffer_.data(),at);return stage_==MotionStage::STOPPING?stopping(at,s):Batch{};
  }
}
}  // namespace slaihand_hardware
