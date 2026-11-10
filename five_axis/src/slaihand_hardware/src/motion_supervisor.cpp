// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/motion_supervisor.hpp"
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <limits>
#include <cstring>
#include <locale>
#include <sstream>
#include <vector>

namespace slaihand_hardware {
namespace {
std::string json_number(double value){
  if(!std::isfinite(value))return "null";
  std::ostringstream text;text.imbue(std::locale::classic());text<<std::setprecision(17)<<value;
  return text.str();
}
void append_json_array(std::ostringstream& out,const std::vector<double>& values){
  out<<'[';
  for(std::size_t i=0;i<values.size();++i){if(i)out<<',';out<<json_number(values[i]);}
  out<<']';
}
std::string trim_copy(const std::string& text){
  const auto not_space=[](unsigned char c){return !std::isspace(c);};
  auto begin=std::find_if(text.begin(),text.end(),not_space);
  auto end=std::find_if(text.rbegin(),text.rend(),not_space).base();
  if(begin>=end)return std::string();
  return std::string(begin,end);
}
}  // namespace
std::vector<double> parse_gain_axis_parameter(const std::string& text,const char* name){
  const std::string key=name&&*name?name:"gain_axis";
  std::string list=trim_copy(text);
  std::vector<double> values;
  if(list.empty())return values;
  if(list.front()=='['||list.back()==']'){
    if(list.size()<2||list.front()!='['||list.back()!=']')
      throw std::invalid_argument("Unbalanced "+key+" axis list");
    list=trim_copy(list.substr(1,list.size()-2));
    if(list.empty())return values;
  }
  // Refuse empty elements ("160,,150", "1 2 3 4 5,"): a malformed list must not
  // silently shrink into a shorter list that could later look length-valid.
  {
    std::size_t begin=0;
    for(;;){
      const auto comma=list.find(',',begin);
      const std::string element=trim_copy(list.substr(begin,comma==std::string::npos
          ?std::string::npos:comma-begin));
      if(element.empty())throw std::invalid_argument("Empty element in "+key+" axis list");
      if(comma==std::string::npos)break;
      begin=comma+1;
    }
  }
  std::string separated=list;
  std::replace(separated.begin(),separated.end(),',',' ');
  std::istringstream stream(separated);stream.imbue(std::locale::classic());
  std::string token;
  while(stream>>token){
    std::size_t used=0;double value=0.;
    try{value=std::stod(token,&used);}
    catch(const std::exception&){throw std::invalid_argument("Non-numeric "+key+" axis gain");}
    if(used!=token.size()||!std::isfinite(value))
      throw std::invalid_argument("Non-finite "+key+" axis gain");
    values.push_back(value);
  }
  return values;
}
std::vector<std::vector<KpSchedulePoint>> parse_kp_schedule_parameter(
    const std::string& text,const char* name){
  const std::string key=name&&*name?name:"kp_schedule";
  const std::string list=trim_copy(text);
  std::vector<std::vector<KpSchedulePoint>> result;
  if(list.empty())return result;
  std::size_t axis_begin=0;
  while(axis_begin<=list.size()){
    const auto axis_end=list.find(';',axis_begin);
    const std::string axis_text=trim_copy(list.substr(
        axis_begin,axis_end==std::string::npos?std::string::npos:axis_end-axis_begin));
    if(axis_text.empty())throw std::invalid_argument("Empty axis entry in "+key);
    std::vector<KpSchedulePoint> points;
    std::size_t begin=0;
    for(;;){
      const auto comma=axis_text.find(',',begin);
      const std::string element=trim_copy(axis_text.substr(
          begin,comma==std::string::npos?std::string::npos:comma-begin));
      if(element.empty())throw std::invalid_argument("Empty point in "+key);
      const auto colon=element.find(':');
      if(colon==std::string::npos||element.find(':',colon+1)!=std::string::npos)
        throw std::invalid_argument("Schedule point must be angle:kp in "+key);
      const std::string angle_text=trim_copy(element.substr(0,colon));
      const std::string kp_text=trim_copy(element.substr(colon+1));
      if(angle_text.empty()||kp_text.empty())throw std::invalid_argument("Empty angle or Kp in "+key);
      const auto parse_number=[&](const std::string& token)->double{
        std::size_t used=0;double value=0.;
        try{value=std::stod(token,&used);}
        catch(const std::exception&){throw std::invalid_argument("Non-numeric schedule value in "+key);}
        if(used!=token.size()||!std::isfinite(value))
          throw std::invalid_argument("Non-finite schedule value in "+key);
        return value;
      };
      const double angle=parse_number(angle_text),kp=parse_number(kp_text);
      if(!points.empty()&&angle<=points.back().angle_deg)
        throw std::invalid_argument("Schedule angles must be strictly increasing in "+key);
      points.push_back({angle,kp});
      if(comma==std::string::npos)break;
      begin=comma+1;
    }
    if(points.empty())throw std::invalid_argument("Empty schedule in "+key);
    result.push_back(std::move(points));
    if(axis_end==std::string::npos)break;
    axis_begin=axis_end+1;
  }
  return result;
}
double interpolate_kp_schedule(const std::vector<KpSchedulePoint>& points,
                               double angle_deg){
  if(points.empty()||!std::isfinite(angle_deg))
    throw std::invalid_argument("Kp schedule requires finite angle and at least one point");
  if(angle_deg<=points.front().angle_deg)return points.front().kp;
  if(angle_deg>=points.back().angle_deg)return points.back().kp;
  for(std::size_t i=1;i<points.size();++i){
    if(angle_deg<=points[i].angle_deg){
      const auto& a=points[i-1];const auto& b=points[i];
      const double span=b.angle_deg-a.angle_deg;
      if(!(span>0))throw std::invalid_argument("Kp schedule angles must increase");
      const double ratio=(angle_deg-a.angle_deg)/span;
      return a.kp+(b.kp-a.kp)*ratio;
    }
  }
  return points.back().kp;
}
double slew_kp_gain(double current,double target,double dt,double max_rate){
  if(!std::isfinite(current)||!std::isfinite(target)||!std::isfinite(dt)||
     !std::isfinite(max_rate)||dt<=0||max_rate<=0)
    throw std::invalid_argument("Invalid Kp slew input");
  const double step=max_rate*dt;
  if(target>current+step)return current+step;
  if(target<current-step)return current-step;
  return target;
}
void MotionPolicy::validate(std::size_t axis_count)const{
  const double values[]{kp,kd,reference_speed,feedback_speed,error,temperature,ramp,stagger,age,gap,runtime};
  for(double v:values)if(!std::isfinite(v)||v<=0)throw std::invalid_argument("Invalid finite motion policy");
  if(!std::isfinite(kp_max)||kp_max<=0||kp_max>500)
    throw std::invalid_argument("kp_max must be finite in (0,500]");
  if(kp>kp_max||kd>5||
     reference_speed>1||feedback_speed>1||error>4*std::acos(-1.)/180.+1e-12||temperature>50||
     ramp<1||stagger<.03||age>.06||gap>.05||runtime>180)throw std::invalid_argument("Motion policy exceeds adopted limits");
  if(!std::isfinite(gravity_alpha)||gravity_alpha<0||gravity_alpha>1||
     !std::isfinite(gravity_ramp)||gravity_ramp<0)
    throw std::invalid_argument("Invalid gravity feedforward policy");
  for(double value:guard)if(!std::isfinite(value)||value<0)
    throw std::invalid_argument("Invalid host torque guard; zero disables it");
  const std::size_t count=axis_count?axis_count:guard.size();
  if(count<4||count>5||guard.size()!=count)
    throw std::invalid_argument("Motion guard count must equal the four or five active axes");
  // Per-axis Kp/Kd are an additive five-axis feature. The scalar caps above are
  // unchanged; each list entry must satisfy the same adopted range, and a
  // supplied list must cover the active axes exactly (four axes keep the legacy
  // scalar policy).
  // Per-axis gains share the explicit policy ceiling. The protocol encoding
  // limit remains wider than kp_max and is not a mechanical capability claim.
  for(double value:kp_axis)if(!std::isfinite(value)||value<=0||value>kp_max)
    throw std::invalid_argument("Per-axis Kp must be finite in (0,kp_max]");
  for(double value:kd_axis)if(!std::isfinite(value)||value<=0||value>5)
    throw std::invalid_argument("Per-axis Kd must be finite in (0,5]");
  if(!kp_axis.empty()||!kd_axis.empty()){
    if(count!=5)throw std::invalid_argument("Per-axis Kp/Kd are supported only for the five active axes");
    if(!kp_axis.empty()&&kp_axis.size()!=count)
      throw std::invalid_argument("Kp axis list length must equal the five active axes");
    if(!kd_axis.empty()&&kd_axis.size()!=count)
      throw std::invalid_argument("Kd axis list length must equal the five active axes");
  }
  if(!motion_qualified.empty()&&motion_qualified.size()!=count)
    throw std::invalid_argument("Motion qualification count must equal the active axes");
  if(!kp_schedule.empty()){
    if(kp_schedule.size()!=count)
      throw std::invalid_argument("Kp schedule count must equal the active axes");
    if(!std::isfinite(kp_schedule_slew)||kp_schedule_slew<=0||kp_schedule_slew>5000)
      throw std::invalid_argument("Kp schedule slew must be finite in (0,5000]");
    for(const auto& points:kp_schedule){
      if(points.empty()||points.size()>32)
        throw std::invalid_argument("Kp schedule needs 1..32 points per axis");
      for(std::size_t i=0;i<points.size();++i){
        const auto& point=points[i];
        if(!std::isfinite(point.angle_deg)||point.angle_deg<-720||point.angle_deg>720)
          throw std::invalid_argument("Kp schedule angle must be finite in [-720,720]");
        if(!std::isfinite(point.kp)||point.kp<=0||point.kp>kp_max)
          throw std::invalid_argument("Kp schedule gain must be finite in (0,kp_max]");
        if(i&&point.angle_deg<=points[i-1].angle_deg)
          throw std::invalid_argument("Kp schedule angles must be strictly increasing");
      }
    }
  }
}
MotionSupervisor::MotionSupervisor(MotionPolicy policy,const std::vector<PositionBounds>& bounds,
                                   std::shared_ptr<const GravityFfModel> gravity_model)
    :p_(policy),bounds_(bounds),gravity_model_(std::move(gravity_model)){
  if(bounds_.size()<4||bounds_.size()>5)
    throw std::invalid_argument("Motion supervision supports four or five active axes");
  p_.validate(bounds_.size());
  // Normalize the additive per-axis gain lists once. An absent list keeps the
  // scalar legacy value for every axis; identity/audit read these effective
  // vectors, never the nominal policy scalar as if it were the applied value.
  const std::size_t active=bounds_.size();
  kp_axis_per_axis_=!p_.kp_axis.empty();
  kd_axis_per_axis_=!p_.kd_axis.empty();
  kp_axis_.assign(active,p_.kp);kd_axis_.assign(active,p_.kd);
  if(kp_axis_per_axis_)kp_axis_=p_.kp_axis;
  if(kd_axis_per_axis_)kd_axis_=p_.kd_axis;
  kp_schedule_active_=!p_.kp_schedule.empty();
  kp_effective_=kp_axis_;kp_schedule_target_=kp_axis_;
  kp_schedule_segment_.assign(active,0);
  if(kp_schedule_active_){
    for(std::size_t i=0;i<active;++i){
      // Safe pre-first-cycle value: the first schedule point, not zero.
      kp_effective_[i]=p_.kp_schedule[i].front().kp;
      kp_schedule_target_[i]=kp_effective_[i];
    }
  }
  // Empty qualification keeps the adopted four-axis behaviour. Extending that
  // default to five axes is deliberately refused for J5: its sign, zero and
  // limits are not calibrated, so the preflight rejects the whole arm instead
  // of pretending the fifth axis is ready.
  if(p_.motion_qualified.empty()){
    p_.motion_qualified.assign(bounds_.size(),true);
    if(bounds_.size()==5)p_.motion_qualified[4]=false;
  }
  for(std::size_t i=0;i<bounds_.size();++i){
    const auto& b=bounds_[i];
    const bool defined=std::isfinite(b.lower)||std::isfinite(b.upper);
    if(defined&&(!std::isfinite(b.lower)||!std::isfinite(b.upper)||b.lower>=b.upper))
      throw std::invalid_argument("Invalid URDF limits");
    // An unqualified axis is monitored for feedback and covered by the global
    // stop only. It has no motion sector yet, so undefined bounds are honest.
    if(!defined&&p_.motion_qualified[i])
      throw std::invalid_argument("Motion-qualified axis requires finite URDF limits");
  }
  // The existing gravity model is explicitly four-axis. A five-axis arm cannot
  // inherit it or a four-joint angle vector.
  if(gravity_model_&&!gravity_model_->empty()&&bounds_.size()!=4)
    throw std::invalid_argument("Gravity feedforward model is four-axis only");
  const std::size_t n=bounds_.size();
  ff_requested_.assign(n,0.);ff_candidate_.assign(n,0.);ff_encoded_.assign(n,0.);ff_applied_.assign(n,0.);
  ff_last_written_.assign(n,0.);ff_written_at_.assign(n,-1.);
  hold_target_.assign(n,0.);hold_error_.assign(n,0.);request_estimate_.assign(n,0.);
  anchor_.assign(n,0.);target_.assign(n,0.);
  enable_at_.assign(n,-1.);confirmed_at_.assign(n,-1.);fd_at_.assign(n,-1.);stop_seen_.assign(n,-1.);
  stop_samples_.assign(n,0);
  last_mit_kp_.assign(n,0.);last_mit_kd_.assign(n,0.);last_mit_tau_.assign(n,0.);last_mit_write_at_.assign(n,-1.);
  last_scaled_kp_.assign(n,0.);last_scaled_at_=-1;
  request_estimate_at_=-1;
  first_fault_.encoded_kp.assign(n,0.);first_fault_.encoded_kd.assign(n,0.);
  first_fault_.encoded_tau_ff.assign(n,0.);first_fault_.request_estimate.assign(n,0.);
  std::strncpy(lock_reason_.data(),"NONE",lock_reason_.size()-1);lock_code_=0;
}
bool MotionSupervisor::request_gravity_mode(bool hold,std::uint64_t command_id,double at){
  if(stage_!=MotionStage::HOLD||command_id!=command_id_)return false;
  if(hold){
    if(!floating_||gravity_hold_||lock_pending_||lock_state_!=GravityLockState::NONE||!hold_ready_)return false;
    lock_command_id_=command_id;lock_pending_=true;release_pending_=false;
    lock_state_=GravityLockState::LOCKING;
    std::strncpy(lock_reason_.data(),"LOCK_REQUESTED",lock_reason_.size()-1);lock_code_=1;
    return true;
  }
  if(!gravity_hold_||release_pending_)return false;
  release_pending_=true;release_started_=at;
  std::strncpy(lock_reason_.data(),"RELEASE_REQUESTED",lock_reason_.size()-1);lock_code_=7;
  return true;
}
bool MotionSupervisor::arm(double at,bool gravity_float,double float_kd,double float_seconds){
  if(stage_!=MotionStage::DISARMED||!std::isfinite(at))return false;
  // Whole-arm preflight: one uncalibrated controlled axis refuses every axis.
  // J5 in particular must not be enabled on a best-effort mapping.
  if(std::any_of(p_.motion_qualified.begin(),p_.motion_qualified.end(),[](bool qualified){return !qualified;})){
    reason_="AXIS_CALIBRATION_PENDING";hold_ready_=false;attempted_=false;return false;
  }
  if(gravity_float&&(!p_.gravity_float_enabled||!gravity_model_||gravity_model_->empty()))return false;
  if(gravity_float&&(!std::isfinite(float_kd)||float_kd<0||float_kd>5||
                    !std::isfinite(float_seconds)||float_seconds<1||float_seconds>120))return false;
  floating_=gravity_float;
  gravity_hold_=false;lock_state_=GravityLockState::NONE;lock_pending_=release_pending_=false;
  hold_gain_scale_=0;release_gain_start_=0;
  std::fill(hold_target_.begin(),hold_target_.end(),0.);std::fill(hold_error_.begin(),hold_error_.end(),0.);
  std::strncpy(lock_reason_.data(),"NONE",lock_reason_.size()-1);lock_code_=0;
  float_kd_=gravity_float?float_kd:5.;float_seconds_=gravity_float?float_seconds:5.;
  ++command_id_;started_=last_tick_=at;stage_=MotionStage::PREFLIGHT;reason_="ARM_ACCEPTED_NOT_ENABLED";
  hold_ready_=false;stop_confirmed_=false;attempted_=false;next_enable_=0;preload_mask_=0;stable_at_=-1;
  std::fill(enable_at_.begin(),enable_at_.end(),-1);std::fill(confirmed_at_.begin(),confirmed_at_.end(),-1);
  std::fill(fd_at_.begin(),fd_at_.end(),-1);std::fill(stop_seen_.begin(),stop_seen_.end(),-1);
  std::fill(stop_samples_.begin(),stop_samples_.end(),0u);
  clear_first_fault();
  reset_kp_schedule_state();
  clear_feedforward(true);return true;
}
bool MotionSupervisor::reset(double at,const slaihand_transport::ReadOnlyState& s){
  if(stage_!=MotionStage::DISABLED||!s.complete(at,p_.age))return false;
  if(s.axis_count()!=bounds_.size())return false;
  for(std::size_t i=1;i<=bounds_.size();++i)if(s.axis(int(i)).feedback->motor_state!=0)return false;
  stage_=MotionStage::DISARMED;reason_="MANUAL_RESET_DISABLED_ONLY";attempted_=false;
  floating_=gravity_hold_=false;lock_state_=GravityLockState::NONE;lock_pending_=release_pending_=false;
  lock_code_=0;clear_first_fault();reset_kp_schedule_state();clear_feedforward(true);return true;
}
void MotionSupervisor::stop(const char* why,double at,bool explicit_stop){
  if(stage_==MotionStage::STOPPING||stage_==MotionStage::UNKNOWN)return;
  reason_=why;hold_ready_=false;stable_at_=-1;stop_confirmed_=false;
  floating_=gravity_hold_=false;lock_state_=GravityLockState::NONE;lock_pending_=release_pending_=false;
  lock_code_=0;clear_feedforward(true);
  if(!attempted_&&!explicit_stop){stage_=MotionStage::DISABLED;return;}
  stage_=MotionStage::STOPPING;stop_at_=at;
  std::fill(fd_at_.begin(),fd_at_.end(),-1);std::fill(stop_seen_.begin(),stop_seen_.end(),-1);
  std::fill(stop_samples_.begin(),stop_samples_.end(),0u);
}
void MotionSupervisor::check_observations(double at,const slaihand_transport::ReadOnlyState& s,bool initial){
  const double unavailable=std::numeric_limits<double>::quiet_NaN();
  if(s.axis_count()!=bounds_.size()){
    capture_fault(at,s,0,"ACTIVE_AXIS_COUNT_MISMATCH",unavailable,unavailable);
    throw std::runtime_error("ACTIVE_AXIS_COUNT_MISMATCH");
  }
  if(s.conflicted()||!s.complete(at,p_.age)){
    capture_fault(at,s,0,"FEEDBACK_STALE_OR_CONFLICTED",unavailable,unavailable);
    throw std::runtime_error("FEEDBACK_STALE_OR_CONFLICTED");
  }
  for(std::size_t i=1;i<=bounds_.size();++i){
    if(!p_.motion_qualified[i-1]){
      capture_fault(at,s,int(i),"AXIS_CALIBRATION_PENDING",unavailable,unavailable);
      throw std::runtime_error("AXIS_CALIBRATION_PENDING");
    }
    const auto& a=s.axis(int(i));const auto& f=*a.feedback;
    try{check_native_profile(int(i),a);}
    catch(const std::exception&){
      capture_fault(at,s,int(i),"NATIVE_PROFILE_MISMATCH",unavailable,unavailable);
      throw;
    }
    if(*a.registers[3]!=1||*a.registers[2]!=2000||*a.registers[0]!=0||*a.registers[1]!=int(i)){
      capture_fault(at,s,int(i),"REQUIRE_MIT_TIMEOUT2000_IDS",unavailable,unavailable);
      throw std::runtime_error("REQUIRE_MIT_TIMEOUT2000_IDS");
    }
    if(f.driver_temperature>p_.temperature||f.motor_temperature>p_.temperature){
      capture_fault(at,s,int(i),"TEMPERATURE",unavailable,p_.temperature);
      throw std::runtime_error("TEMPERATURE");
    }
    // Position mode keeps the 1 rad/s feedback speed gate and terminates when the
    // actual position leaves the operating limits. Hand dragging is a deliberate
    // operator motion: while floating, crossing the URDF soft limit must not throw
    // and disable the arm in the operator's hands. The initial arming preflight
    // still requires all axes inside the limits. Float keeps the adopted session
    // timeout, temperature, feedback age, cycle gap, MIT protocol and global-stop
    // checks; only the actual-position-limit termination is skipped after arming.
    if(!floating_&&std::abs(f.velocity)>p_.feedback_speed){
      capture_fault(at,s,int(i),"FEEDBACK_SPEED",std::abs(f.velocity),p_.feedback_speed);
      throw std::runtime_error("FEEDBACK_SPEED");
    }
    if((!floating_||initial)&&(f.position<bounds_[i-1].lower||f.position>bounds_[i-1].upper)){
      capture_fault(at,s,int(i),"ACTUAL_OUTSIDE_OPERATING_LIMITS",f.position,unavailable);
      throw std::runtime_error("ACTUAL_OUTSIDE_OPERATING_LIMITS");
    }
    if(initial){
      if(f.motor_state!=0||std::abs(f.velocity)>.05){
        capture_fault(at,s,int(i),"REQUIRE_DISABLED_STATIONARY",unavailable,unavailable);
        throw std::runtime_error("REQUIRE_DISABLED_STATIONARY");
      }
    }else{
      if(f.motor_state!=0&&f.motor_state!=1){
        capture_fault(at,s,int(i),"MOTOR_FAULT",unavailable,unavailable);
        throw std::runtime_error("MOTOR_FAULT");
      }
      if(confirmed_at_[i-1]>=0&&f.motor_state!=1){
        capture_fault(at,s,int(i),"UNEXPECTED_AXIS_DISABLED",unavailable,unavailable);
        throw std::runtime_error("UNEXPECTED_AXIS_DISABLED");
      }
      // Capture the exact gate values before stop("...") can overwrite the stage.
      // This is the first-cause evidence the operator report asks for.
      if(!floating_&&std::abs(target_[i-1]-f.position)>p_.error){
        capture_fault(at,s,int(i),"FOLLOWING_ERROR",std::abs(target_[i-1]-f.position),p_.error);
        throw std::runtime_error("FOLLOWING_ERROR");
      }
      if(p_.guard[i-1]>0&&std::abs(f.effort_estimate)>p_.guard[i-1]){
        capture_fault(at,s,int(i),"FEEDBACK_ESTIMATE_GUARD_NOT_HARD_CLAMP",
                      std::abs(f.effort_estimate),p_.guard[i-1]);
        throw std::runtime_error("FEEDBACK_ESTIMATE_GUARD_NOT_HARD_CLAMP");
      }
    }
  }
}
void MotionSupervisor::capture_fault(double at,const slaihand_transport::ReadOnlyState& state,
                                     int axis,const char* code,double error_rad,double limit_rad){
  if(first_fault_.valid)return;  // first unsafe observation wins for the session
  const double unavailable=std::numeric_limits<double>::quiet_NaN();
  FaultSnapshot snapshot;
  snapshot.valid=true;
  snapshot.axis=axis;
  snapshot.code=code?code:"UNKNOWN_FAULT";
  snapshot.detected_at=at;
  snapshot.error_rad=error_rad;
  snapshot.limit_rad=limit_rad;
  snapshot.target_rad=unavailable;snapshot.actual_rad=unavailable;snapshot.feedback_age=unavailable;
  snapshot.feedback_valid=false;
  snapshot.request_estimate_at=request_estimate_at_;
  snapshot.encoded_kp=last_mit_kp_;snapshot.encoded_kd=last_mit_kd_;
  snapshot.encoded_tau_ff=last_mit_tau_;snapshot.request_estimate=request_estimate_;
  snapshot.command_id=command_id_;
  // A complete write batch means every active axis had at least one MIT write;
  // the earliest of those times is the point where the last batch was complete.
  bool complete=true;double last=-1;
  for(double written:last_mit_write_at_){
    if(written<0){complete=false;break;}
    last=last<0?written:std::min(last,written);
  }
  snapshot.last_complete_write_at=complete?last:-1;
  if(axis>=1&&axis<=int(bounds_.size())){
    snapshot.joint="slaihand_joint"+std::to_string(axis);
    const bool output_stage=stage_==MotionStage::PRELOAD||stage_==MotionStage::ENABLING||
                            stage_==MotionStage::RAMP||stage_==MotionStage::HOLD||
                            stage_==MotionStage::TRACK;
    if(output_stage)snapshot.target_rad=target_[std::size_t(axis-1)];
    if(axis<=int(state.axis_count())){
      const auto& observed=state.axis(axis);
      if(observed.feedback){
        const auto& feedback=*observed.feedback;
        snapshot.actual_rad=feedback.position;
        snapshot.feedback_age=at-feedback.received_at;
        snapshot.feedback_valid=state.ready(axis)&&snapshot.feedback_age>=0&&snapshot.feedback_age<=p_.age;
      }
    }
  }
  first_fault_=std::move(snapshot);
}
void MotionSupervisor::clear_first_fault(){
  // Session evidence reset: the frozen first cause, the decoded applied-write
  // history it refers to and the request-estimate age all belong to one session.
  // A new arm/reset must not let a previous session's packet be reported as the
  // last applied write of the new one.
  FaultSnapshot cleared;
  cleared.encoded_kp.assign(bounds_.size(),0.);
  cleared.encoded_kd.assign(bounds_.size(),0.);
  cleared.encoded_tau_ff.assign(bounds_.size(),0.);
  cleared.request_estimate.assign(bounds_.size(),0.);
  first_fault_=std::move(cleared);
  std::fill(last_mit_kp_.begin(),last_mit_kp_.end(),0.);
  std::fill(last_mit_kd_.begin(),last_mit_kd_.end(),0.);
  std::fill(last_mit_tau_.begin(),last_mit_tau_.end(),0.);
  std::fill(last_mit_write_at_.begin(),last_mit_write_at_.end(),-1.);
  std::fill(request_estimate_.begin(),request_estimate_.end(),0.);
  request_estimate_at_=-1;
}
std::string MotionSupervisor::gain_axis_json()const{
  std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
  out<<",\"kp_axis\":";
  append_json_array(out,kp_axis_);
  out<<",\"kd_axis\":";
  append_json_array(out,kd_axis_);
  out<<",\"kp_axis_source\":\""<<(kp_axis_per_axis_?"per_axis":"scalar_legacy")<<"\""
     <<",\"kd_axis_source\":\""<<(kd_axis_per_axis_?"per_axis":"scalar_legacy")<<"\"";
  out<<",\"kp_max\":"<<json_number(p_.kp_max);
  out<<",\"kp_schedule_source\":\""<<(kp_schedule_active_?"angle_schedule":"none")<<"\"";
  out<<",\"kp_schedule_slew_kp_per_s\":"<<json_number(p_.kp_schedule_slew);
  if(kp_schedule_active_){
    out<<",\"kp_schedule\":[";
    for(std::size_t i=0;i<p_.kp_schedule.size();++i){
      if(i)out<<',';
      out<<'[';
      for(std::size_t j=0;j<p_.kp_schedule[i].size();++j){
        if(j)out<<',';
        out<<'['<<json_number(p_.kp_schedule[i][j].angle_deg)<<','
          <<json_number(p_.kp_schedule[i][j].kp)<<']';
      }
      out<<']';
    }
    out<<']';
  }
  return out.str();
}
const char* MotionSupervisor::request_estimate_state()const{
  if(request_estimate_at_<0)return "UNAVAILABLE";
  switch(stage_){
    case MotionStage::PRELOAD:
    case MotionStage::ENABLING:
    case MotionStage::RAMP:
    case MotionStage::HOLD:
    case MotionStage::TRACK:return "CURRENT";
    default:return "LAST_ACTIVE_STALE";
  }
}
std::string MotionSupervisor::first_fault_json(const std::string& boot_id,std::uint64_t cycle_id)const{
  if(!first_fault_.valid)return "null";
  const FaultSnapshot& f=first_fault_;
  const double degrees=180./std::acos(-1.);
  std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17);
  out<<"{\"schema\":\"slaihand_first_fault/1\",\"layer\":\"DRIVER\""
     <<",\"axis\":"<<f.axis
     <<",\"joint\":"<<(f.joint.empty()?"null":"\""+f.joint+"\"")
     <<",\"code\":\""<<f.code<<"\""
     <<",\"detected_at\":"<<json_number(f.detected_at)
     <<",\"error_rad\":"<<json_number(f.error_rad)
     <<",\"limit_rad\":"<<json_number(f.limit_rad)
     <<",\"error_deg\":"<<json_number(std::isfinite(f.error_rad)?f.error_rad*degrees:f.error_rad)
     <<",\"limit_deg\":"<<json_number(std::isfinite(f.limit_rad)?f.limit_rad*degrees:f.limit_rad)
     <<",\"target_rad\":"<<json_number(f.target_rad)
     <<",\"actual_rad\":"<<json_number(f.actual_rad)
     <<",\"target_deg\":"<<json_number(std::isfinite(f.target_rad)?f.target_rad*degrees:f.target_rad)
     <<",\"actual_deg\":"<<json_number(std::isfinite(f.actual_rad)?f.actual_rad*degrees:f.actual_rad)
     <<",\"feedback_age_s\":"<<json_number(f.feedback_age)
     <<",\"feedback_valid\":"<<(f.feedback_valid?"true":"false")
     <<",\"encoded_kp\":";
  append_json_array(out,f.encoded_kp);
  out<<",\"encoded_kd\":";
  append_json_array(out,f.encoded_kd);
  out<<",\"encoded_tau_ff_nm\":";
  append_json_array(out,f.encoded_tau_ff);
  out<<",\"request_estimate_nm\":";
  append_json_array(out,f.request_estimate);
  out<<",\"request_estimate_at\":"<<json_number(f.request_estimate_at)
     <<",\"request_estimate_state\":\"LAST_ACTIVE\""
     <<",\"command_id\":"<<f.command_id
     <<",\"cycle_id\":"<<cycle_id
     <<",\"boot_id\":\""<<boot_id<<"\""
     <<",\"last_complete_write_at\":"<<json_number(f.last_complete_write_at)
     <<",\"frozen\":true}";
  return out.str();
}
void MotionSupervisor::clear_feedforward(bool keep_history){
  std::fill(ff_requested_.begin(),ff_requested_.end(),0.);std::fill(ff_candidate_.begin(),ff_candidate_.end(),0.);
  std::fill(ff_encoded_.begin(),ff_encoded_.end(),0.);std::fill(ff_applied_.begin(),ff_applied_.end(),0.);
  ff_active_=false;
  if(!keep_history){std::fill(ff_last_written_.begin(),ff_last_written_.end(),0.);
    std::fill(ff_written_at_.begin(),ff_written_at_.end(),-1.);}
}
void MotionSupervisor::reset_kp_schedule_state(){
  kp_schedule_initialized_=false;
  kp_schedule_last_at_=-1;
  kp_effective_=kp_axis_;
  kp_schedule_target_=kp_axis_;
  std::fill(kp_schedule_segment_.begin(),kp_schedule_segment_.end(),0);
}
void MotionSupervisor::update_kp_schedule(double at,const slaihand_transport::ReadOnlyState& state){
  if(!kp_schedule_active_)return;
  const double pi=std::acos(-1.);
  const std::size_t n=bounds_.size();
  if(!kp_schedule_initialized_){
    for(std::size_t i=0;i<n;++i){
      const auto& feedback=*state.axis(int(i)+1).feedback;
      const double q_deg=feedback.position*180./pi;
      if(!std::isfinite(q_deg))continue;
      const auto& points=p_.kp_schedule[i];
      kp_schedule_target_[i]=interpolate_kp_schedule(points,q_deg);
      kp_effective_[i]=kp_schedule_target_[i];
      int segment=0;while(segment+1<int(points.size())&&q_deg>=points[std::size_t(segment)+1].angle_deg)++segment;
      kp_schedule_segment_[i]=segment;
    }
    kp_schedule_initialized_=true;
    kp_schedule_last_at_=at;
    return;
  }
  const double dt=(kp_schedule_last_at_<0||!std::isfinite(at))?0.:at-kp_schedule_last_at_;
  kp_schedule_last_at_=at;
  if(!(dt>0))return;
  const double bounded_dt=std::min(dt,.05);  // a long audit/loop gap must not jump gains
  for(std::size_t i=0;i<n;++i){
    const auto& feedback=*state.axis(int(i)+1).feedback;
    const double q_deg=feedback.position*180./pi;
    if(!std::isfinite(q_deg))continue;
    const auto& points=p_.kp_schedule[i];
    const double target=interpolate_kp_schedule(points,q_deg);
    kp_schedule_target_[i]=target;
    kp_effective_[i]=slew_kp_gain(kp_effective_[i],target,bounded_dt,p_.kp_schedule_slew);
    int segment=0;while(segment+1<int(points.size())&&q_deg>=points[std::size_t(segment)+1].angle_deg)++segment;
    kp_schedule_segment_[i]=segment;
  }
}
Batch MotionSupervisor::controls(double at,const slaihand_transport::ReadOnlyState& s,int enable_axis){
  const int n=int(bounds_.size());
  update_kp_schedule(at,s);
  std::vector<slaihand_transport::Frame> frames;
  frames.resize(std::size_t(n));
  // Gravity feedforward is computed once per cycle from the measured model angles.
  // ReadOnlyState already maps feedback into the URDF model frame
  // (q_model = sign * (raw - zero)), so no extra mapping is applied here.
  std::vector<double> gravity;
  if(gravity_model_&&gravity_alpha()>0){
    std::array<double,4> q_model{};
    for(int i=1;i<=n;++i){const auto& a=s.axis(i);const auto& f=*a.feedback;
      // Hand-drag mode may cross the URDF soft limit (the actual-position stop is
      // disabled while floating). The gravity model still only accepts angles inside
      // its URDF limits, so clamp the input at the boundary and hold the boundary
      // compensation torque instead of throwing and disabling the arm in the hand.
      q_model[i-1]=floating_?std::clamp(f.position,bounds_[i-1].lower,bounds_[i-1].upper)
                            :f.position;}
    gravity=gravity_model_->torque(q_model);
  }
  // Validate all axes before even a single FC or MIT command is emitted.
  for(int i=1;i<=n;++i){
    const auto& a=s.axis(i);const auto& f=*a.feedback;
    const double gain_scale=confirmed_at_[i-1]<0?0:std::clamp((at-confirmed_at_[i-1])/p_.ramp,0.,1.);
    // Independent feedforward ramp; zero duration means immediate once the axis
    // is confirmed enabled, the gain ramp is not reused for it.
    const double ff_scale=confirmed_at_[i-1]<0?0:
        (gravity_ramp()<=0?1.:std::clamp((at-confirmed_at_[i-1])/gravity_ramp(),0.,1.));
    const double hold_scale=gravity_hold_?hold_gain_scale_:1.;
    // Every gain consumer uses the same effective per-axis value: position ramp/
    // hold, gravity-hold lock/resume interpolation, and the resulting MIT packet.
    const double scheduled_kp=kp_schedule_active_?kp_effective_[i-1]:kp_axis_[i-1];
    const double kp=floating_?0.:(gravity_hold_?hold_scale*scheduled_kp:gain_scale*scheduled_kp);
    last_scaled_kp_[i-1]=kp;last_scaled_at_=at;
    const double kd=floating_?gain_scale*float_kd_
        :(gravity_hold_?float_kd_+hold_scale*(kd_axis_[i-1]-float_kd_):gain_scale*kd_axis_[i-1]);
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
  // The request estimate belongs to this completed controls() cycle. After a
  // stop it is retained as last-active evidence but never labelled current.
  request_estimate_at_=at;
  Batch batch;if(enable_axis)batch.add(native_special(enable_axis,true),Role::ENABLE);
  for(int i=0;i<n;++i)batch.add(frames[i],Role::MIT);
  return batch;
}
Batch MotionSupervisor::stopping(double at,const slaihand_transport::ReadOnlyState& s){
  if(s.axis_count()!=bounds_.size()){stage_=MotionStage::UNKNOWN;return {};}
  const int n=int(bounds_.size());
  bool all=true;
  for(int i=1;i<=n;++i){
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
  Batch b;for(int i=1;i<=n;++i)b.add(native_special(i,false),Role::DISABLE);
  for(int i=1;i<=n;++i){auto p=slaihand_transport::read_packet(i);slaihand_transport::Frame f{0x7FF,{}};
    std::copy_n(p.begin()+21,8,f.data.begin());b.add(f,Role::READ);}return b;
}
void MotionSupervisor::submitted(const Action& a,double){if(a.role==Role::ENABLE)attempted_=true;}
void MotionSupervisor::written(const Action& a,double at){
  if(a.role==Role::READ)return;  // Read replies do not carry a controlled axis id.
  const int action_axis=int(a.frame.can_id)-1;
  if(action_axis<0||action_axis>=int(bounds_.size()))throw std::logic_error("Action axis outside active set");
  if(a.role==Role::ENABLE){enable_at_[action_axis]=at;++next_enable_;}
  if(a.role==Role::DISABLE)fd_at_[action_axis]=at;
  if(a.role==Role::MIT){
    if(action_axis>=0&&action_axis<int(bounds_.size())){
      // Only a frame that reached the serial write path counts as applied. This is
      // still a host write receipt, not a drive torque confirmation.
      const auto decoded=decode_native(action_axis+1,a.frame.data);
      ff_applied_[action_axis]=decoded.tau;
      ff_last_written_[action_axis]=decoded.tau;
      ff_written_at_[action_axis]=at;
      // Decoded fields of the frame that actually reached the write path. These
      // are first-fault evidence ("applied encoded Kp/Kd/tau_ff"), not a drive
      // execution acknowledgement.
      last_mit_kp_[action_axis]=decoded.kp;last_mit_kd_[action_axis]=decoded.kd;
      last_mit_tau_[action_axis]=decoded.tau;last_mit_write_at_[action_axis]=at;
      ff_active_=gravity_model_&&gravity_alpha()>0&&
          (stage_==MotionStage::PRELOAD||stage_==MotionStage::ENABLING||
           stage_==MotionStage::RAMP||stage_==MotionStage::HOLD||stage_==MotionStage::TRACK);
    }
    if(stage_==MotionStage::PRELOAD){
      preload_mask_|=1u<<action_axis;
      if(preload_mask_==(1u<<bounds_.size())-1u){preload_at_=at;stage_=MotionStage::ENABLING;}
    }
  }
}
void MotionSupervisor::latch_gravity_hold(double at,const slaihand_transport::ReadOnlyState& s){
  for(std::size_t i=0;i<bounds_.size();++i)target_[i]=s.axis(int(i)+1).feedback->position;
  hold_target_=target_;hold_error_.assign(bounds_.size(),0.);
  floating_=false;gravity_hold_=true;lock_state_=GravityLockState::LOCKED;
  hold_started_=at;hold_gain_scale_=0.;lock_pending_=false;release_pending_=false;
  reason_="GRAVITY_HOLD_LOCKED";
  std::strncpy(lock_reason_.data(),"LOCKED",lock_reason_.size()-1);lock_code_=3;
}
void MotionSupervisor::apply_lock_request(double at,const slaihand_transport::ReadOnlyState& s){
  const int n=int(bounds_.size());
  const auto reject=[this](int code,const char* why){
    lock_state_=GravityLockState::NONE;lock_pending_=false;lock_code_=code;
    std::strncpy(lock_reason_.data(),why,lock_reason_.size()-1);
  };
  const auto feedback_reason=[&s,this,n](int& code){
    for(int i=1;i<=n;++i){const auto& f=*s.axis(i).feedback;
      if(f.motor_state!=1){code=6;return false;}
      if(f.position<bounds_[i-1].lower||f.position>bounds_[i-1].upper){code=5;return false;}}
    code=0;return true;
  };
  if(lock_pending_){
    if(lock_command_id_!=command_id_||!floating_||stage_!=MotionStage::HOLD){reject(4,"LOCK_REJECTED_COMMAND_OR_STAGE");}
    else{
      int code=0;
      if(!feedback_reason(code)){reject(code,code==6?"LOCK_REJECTED_MOTOR_STATE":"LOCK_REJECTED_ACTUAL_OUTSIDE_LIMITS");}
      else{
        double maxv=0;for(int i=1;i<=n;++i)maxv=std::max(maxv,std::abs(s.axis(i).feedback->velocity));
        if(maxv>.2){lock_pending_=false;lock_state_=GravityLockState::LOCKING;lock_code_=2;
          std::strncpy(lock_reason_.data(),"LOCKING_DECELERATE",lock_reason_.size()-1);}
        else latch_gravity_hold(at,s);
      }
    }
  }else if(lock_state_==GravityLockState::LOCKING){
    if(!floating_||lock_command_id_!=command_id_)reject(4,"LOCK_REJECTED_COMMAND_OR_STAGE");
    else{
      int code=0;
      if(!feedback_reason(code)){reject(code,code==6?"LOCK_REJECTED_MOTOR_STATE":"LOCK_REJECTED_ACTUAL_OUTSIDE_LIMITS");}
      else{
        double maxv=0;for(int i=1;i<=n;++i)maxv=std::max(maxv,std::abs(s.axis(i).feedback->velocity));
        if(maxv<=.2)latch_gravity_hold(at,s);
      }
    }
  }
  if(release_pending_){
    if(!gravity_hold_||lock_command_id_!=command_id_){release_pending_=false;lock_code_=4;
      std::strncpy(lock_reason_.data(),"RELEASE_REJECTED_COMMAND_OR_STAGE",lock_reason_.size()-1);}
    else{release_pending_=false;lock_state_=GravityLockState::RELEASING;release_started_=at;
      release_gain_start_=hold_gain_scale_;lock_code_=7;
      std::strncpy(lock_reason_.data(),"RELEASING_TO_FLOAT",lock_reason_.size()-1);}
  }
  if(lock_state_==GravityLockState::RELEASING){
    // Start the release from the gain actually reached, not from full Kp.
    hold_gain_scale_=release_gain_start_*std::clamp(1.-(at-release_started_)/p_.ramp,0.,1.);
    if(hold_gain_scale_<=0.){
      gravity_hold_=false;floating_=true;lock_state_=GravityLockState::NONE;
      for(int i=0;i<n;++i)target_[i]=s.axis(i+1).feedback->position;
      hold_gain_scale_=0.;release_gain_start_=0.;lock_code_=8;
      std::strncpy(lock_reason_.data(),"FLOAT_RESUMED",lock_reason_.size()-1);
    }
  }
  if(gravity_hold_){
    for(int i=0;i<n;++i){hold_error_[i]=target_[i]-s.axis(i+1).feedback->position;}
  }
}
Batch MotionSupervisor::tick(double at,slaihand_transport::ReadOnlyState& s,const std::vector<double>& ref,bool owned){
  if(!std::isfinite(at)){
    capture_fault(last_tick_,s,0,"NONFINITE_CLOCK",
                  std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::quiet_NaN());
    stop("NONFINITE_CLOCK",last_tick_);return stage_==MotionStage::STOPPING?stopping(last_tick_,s):Batch{};
  }
  if(stage_==MotionStage::STOPPING)return stopping(at,s);
  if(stage_==MotionStage::UNKNOWN||stage_==MotionStage::DISARMED||stage_==MotionStage::DISABLED)return {};
  try{
    const double dt=at-last_tick_;
    const int n=int(bounds_.size());
    if(at<last_tick_||(attempted_&&dt>p_.gap))throw std::runtime_error("CONTROL_CYCLE_GAP");
    last_tick_=at;
    if(at-started_>p_.runtime)throw std::runtime_error("FINITE_SESSION_COMPLETE");
    // A lost client cannot leave zero-stiffness or gravity-hold output running
    // indefinitely. The lock transition reuses started_/float_seconds_ and never
    // resets either one.
    if((floating_||gravity_hold_)&&at-started_>float_seconds_+7.)throw std::runtime_error("GRAVITY_FLOAT_SESSION_COMPLETE");
    if((floating_||gravity_hold_)&&owned)throw std::runtime_error("GRAVITY_FLOAT_REJECTS_TRAJECTORY_OWNER");
    if(stage_==MotionStage::PREFLIGHT){
      if(at-started_>3)throw std::runtime_error("PREFLIGHT_TIMEOUT");
      if(!s.complete(at,p_.age))return {};
      check_observations(at,s,true);
      for(int i=0;i<n;++i)anchor_[i]=target_[i]=s.axis(i+1).feedback->position;
      stage_=MotionStage::PRELOAD;return controls(at,s);
    }
    check_observations(at,s,false);
    apply_lock_request(at,s);
    if(floating_){
      for(int i=0;i<n;++i)target_[i]=s.axis(i+1).feedback->position;
    }
    if(stage_==MotionStage::PRELOAD)return controls(at,s);
    for(int i=0;i<n;++i)if(enable_at_[i]>=0&&confirmed_at_[i]<0){
      const auto& f=*s.axis(i+1).feedback;
      if(f.motor_state==1&&f.received_at>enable_at_[i])confirmed_at_[i]=at;
      else if(at-enable_at_[i]>.08)throw std::runtime_error("ENABLE_STATE_NOT_OBSERVED");
    }
    if(stage_==MotionStage::ENABLING){
      const bool previous=next_enable_==0||(confirmed_at_[next_enable_-1]>=0&&at-enable_at_[next_enable_-1]>=p_.stagger);
      if(next_enable_<n&&previous)return controls(at,s,next_enable_+1);
      if(next_enable_==n&&confirmed_at_[n-1]>=0)stage_=MotionStage::RAMP;
    }
    if(stage_==MotionStage::RAMP&&at-confirmed_at_[n-1]>=p_.ramp){stage_=MotionStage::HOLD;reason_="RAMP_COMPLETE_NOT_POSITION_ACCURACY_ACCEPTANCE";}
    if(stage_==MotionStage::HOLD||stage_==MotionStage::TRACK){
      bool stationary=true;for(int i=1;i<=n;++i)stationary&=std::abs(s.axis(i).feedback->velocity)<=.05;
      if(stationary){if(stable_at_<0)stable_at_=at;}else stable_at_=-1;
      if(gravity_hold_){
        hold_gain_scale_=lock_state_==GravityLockState::RELEASING
          ?release_gain_start_*std::clamp(1.-(at-release_started_)/p_.ramp,0.,1.)
          :std::clamp((at-hold_started_)/p_.ramp,0.,1.);
        hold_ready_=hold_gain_scale_>=1.;
      }else{
        // Hand dragging must not wait for a stationary pose: the operator is
        // deliberately moving the arm. Float readiness is the completed feedforward
        // ramp; position mode keeps the stationary+gain-ramp hold criterion.
        hold_ready_=floating_?at-confirmed_at_[n-1]>=gravity_ramp()
                              :stable_at_>=0&&at-stable_at_>=.5;
      }
      if(owned){
        if(ref.size()!=bounds_.size())throw std::runtime_error("REFERENCE_DIMENSION");
        for(int i=0;i<n;++i){
          if(!std::isfinite(ref[i])||ref[i]<bounds_[i].lower||ref[i]>bounds_[i].upper)
            throw std::runtime_error("TARGET_OUTSIDE_URDF_LIMITS");
          if(dt<=0||std::abs(ref[i]-target_[i])>p_.reference_speed*dt+1e-7)throw std::runtime_error("REFERENCE_SPEED");
        }
        target_=ref;stage_=MotionStage::TRACK;
      }else stage_=MotionStage::HOLD;
    }
    return controls(at,s);
  }catch(const std::exception& e){
    // Keep the earliest specific axis fault if check_observations captured one;
    // otherwise freeze this generic first cause (cycle gap, session/reference
    // fault, ...) before stop() overwrites the stage.
    capture_fault(at,s,0,e.what(),std::numeric_limits<double>::quiet_NaN(),
                  std::numeric_limits<double>::quiet_NaN());
    // Exception strings above are fixed literals; retain a stable bounded reason.
    std::strncpy(reason_buffer_.data(),e.what(),reason_buffer_.size()-1);reason_buffer_.back()=0;
    stop(reason_buffer_.data(),at);return stage_==MotionStage::STOPPING?stopping(at,s):Batch{};
  }
}
}  // namespace slaihand_hardware
