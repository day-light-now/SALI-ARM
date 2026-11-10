// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/canusb_system.hpp"
#include "slaihand_hardware/position_branch.hpp"
#include <unistd.h>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>
#include <locale>
#include <set>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <pluginlib/class_list_macros.hpp>

namespace slaihand_hardware {
using Callback=hardware_interface::CallbackReturn;
using Result=hardware_interface::return_type;
namespace {
const char* state_names[]{"position","velocity","effort","driver_temperature","motor_temperature","motor_state",
  "feedback_age","feedback_valid","control_mode","timeout_count","configuration_conflict","control_state",
  "hold_ready","audit_ok","disabled_observed","command_owner","command_id"};
double number(const std::string& s){std::size_t n;const double v=std::stod(s,&n);
  if(n!=s.size()||!std::isfinite(v))throw std::invalid_argument("Invalid numeric motion parameter");
  return v;}
bool is_hash(const std::string& s){return s.size()==64&&s.find_first_not_of("0123456789abcdef")==std::string::npos;}
int own_interfaces(const std::vector<std::string>& entries){
  std::set<std::string> matching;
  for(const auto& s:entries)if(s.rfind("slaihand_joint",0)==0){
    bool valid=false;for(int i=1;i<=slaihand_transport::kMaxAxes;++i)valid|=s=="slaihand_joint"+std::to_string(i)+"/position";
    if(!valid||!matching.insert(s).second)return -1;
  }
  return int(matching.size());
}
}
double CanUsbSystem::now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
CanUsbSystem::~CanUsbSystem(){release();}
Callback CanUsbSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params){
  if(SystemInterface::on_init(params)!=Callback::SUCCESS)return Callback::ERROR;
  try{
    const auto& info=params.hardware_info;const auto& h=info.hardware_parameters;
    if((info.joints.size()!=4&&info.joints.size()!=5)||h.at("protocol")!=slaihand_transport::kProtocol)
      throw std::invalid_argument("Require four or five explicit CANUSB native axes");
    axis_count_=info.joints.size();
    const auto& profile=h.at("policy_profile");
    if(profile!=(axis_count_==4?kFourAxisMotionProfile:kFiveAxisMotionProfile))
      throw std::invalid_argument("Require the matching explicit four/five-axis native motion policy");
    path_=h.at("port");log_directory_=h.at("log_directory");
    if(path_.rfind("/dev/",0)!=0||!std::filesystem::path(log_directory_).is_absolute())throw std::invalid_argument("Explicit device/audit paths required");
    for(const char* key:{"mapping_sha256","model_sha256","policy_sha256"})if(!is_hash(h.at(key)))throw std::invalid_argument("Motion requires source fingerprints");
    policy_.kp=number(h.at("kp"));policy_.kd=number(h.at("kd"));
    // Additive five-axis independent gains. Old scalar-only policy files leave
    // these parameters empty and keep the scalar for every axis. Length/range
    // validation happens in MotionPolicy::validate with the real axis count.
    const auto optional_text=[&](const char* key)->std::string{
      const auto entry=h.find(key);return entry==h.end()?std::string():entry->second;};
    policy_.kp_axis=parse_gain_axis_parameter(optional_text("kp_axis"),"kp_axis");
    policy_.kd_axis=parse_gain_axis_parameter(optional_text("kd_axis"),"kd_axis");
    policy_.reference_speed=number(h.at("reference_speed_rad_s"));policy_.feedback_speed=number(h.at("feedback_speed_rad_s"));
    policy_.error=number(h.at("following_error_deg"))*std::acos(-1.)/180.;policy_.temperature=number(h.at("temperature_c"));
    policy_.ramp=number(h.at("gain_ramp_s"));policy_.stagger=number(h.at("enable_stagger_s"));
    policy_.age=number(h.at("feedback_max_age_s"));policy_.gap=number(h.at("cycle_max_gap_s"));policy_.runtime=number(h.at("session_timeout_s"));
    const auto optional_number=[&](const char* key,double fallback){
      const auto entry=h.find(key);
      return entry==h.end()||entry->second.empty()?fallback:number(entry->second);};
    policy_.gravity_alpha=optional_number("gravity_ff_alpha",0.);
    policy_.gravity_ramp=optional_number("gravity_ff_ramp_s",1.);
    const double float_enabled=optional_number("gravity_float_enabled",0.);
    if(float_enabled!=0.&&float_enabled!=1.)throw std::invalid_argument("Invalid gravity float switch");
    policy_.gravity_float_enabled=float_enabled==1.;
    policy_.kp_max=optional_number("kp_max",160.);
    policy_.kp_schedule=parse_kp_schedule_parameter(optional_text("kp_schedule"),"kp_schedule");
    policy_.kp_schedule_slew=optional_number("kp_schedule_slew_kp_per_s",80.);
    const auto model_entry=h.find("gravity_model");
    if(model_entry!=h.end()&&!model_entry->second.empty()){
      if(axis_count_!=4)throw std::invalid_argument("Five-axis gravity feedforward is not available; J5 geometry is provisional");
      try{
        gravity_model_=std::make_shared<const GravityFfModel>(GravityFfModel::parse(model_entry->second));
      }catch(const std::exception& error){
        throw std::invalid_argument(std::string("Invalid gravity model parameter: ")+error.what());
      }
      const auto& names=gravity_model_->joint_names();
      for(std::size_t i=0;i<axis_count_;++i)
        if(names[i]!="slaihand_joint"+std::to_string(i+1))
          throw std::invalid_argument("Gravity model joint order must match the canonical joints");
    }
    policy_.guard.assign(axis_count_,0.);
    policy_.motion_qualified.assign(axis_count_,true);
    mappings_.assign(axis_count_,{});
    canonical_mappings_.assign(axis_count_,{});
    resolve_branch_.assign(axis_count_,false);
    position_turns_.assign(axis_count_,0);
    values_.assign(axis_count_,std::array<double,17>{});
    commands_.assign(axis_count_,std::numeric_limits<double>::quiet_NaN());
    bounds_.assign(axis_count_,PositionBounds{std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN()});
    for(std::size_t i=0;i<axis_count_;++i){
      const auto& j=info.joints[i];
      if(j.name!="slaihand_joint"+std::to_string(i+1)||j.command_interfaces.size()!=1||j.command_interfaces[0].name!="position")
        throw std::invalid_argument("Only native position command interfaces are supported");
      if(j.command_interfaces[0].parameters.count("min")||j.command_interfaces[0].parameters.count("max"))throw std::invalid_argument("Do not duplicate URDF limits");
      const auto sign=number(j.parameters.at("hardware_sign"));
      if(number(j.parameters.at("can_id"))!=int(i+1)||(sign!=1&&sign!=-1))throw std::invalid_argument("Invalid canonical mapping");
      mappings_[i]={int(i+1),int(sign),number(j.parameters.at("zero_offset_rad"))};
      canonical_mappings_[i]=mappings_[i];
      // J5 defaults to calibration-pending. A qualified axis must carry an
      // explicit audit artifact hash; nothing here invents zero, sign, limits,
      // or a special current reading. J1-J4 keep their legacy implicit
      // qualification when the parameter is absent.
      const auto qualified=j.parameters.find("calibration_qualified");
      policy_.motion_qualified[i]=qualified==j.parameters.end()?i<4:qualified->second=="true";
      if(i==4&&policy_.motion_qualified[4]){
        const auto artifact=j.parameters.find("calibration_sha256");
        if(artifact==j.parameters.end()||!is_hash(artifact->second))
          throw std::invalid_argument("J5 motion qualification requires a calibration artifact hash");
      }
      const auto mode=j.parameters.find("position_branch_mode");
      std::string branch_mode=mode==j.parameters.end()?"fixed":mode->second;
      if(!policy_.motion_qualified[i])branch_mode="fixed";  // no sector to resolve yet
      if(branch_mode!="fixed"&&branch_mode!="startup_unique_urdf")throw std::invalid_argument("Unknown position branch mode");
      resolve_branch_[i]=branch_mode=="startup_unique_urdf";
      const auto limit_entry=info.limits.find(j.name);
      const bool has_limits=limit_entry!=info.limits.end()&&limit_entry->second.has_position_limits;
      if(!has_limits){
        if(policy_.motion_qualified[i])throw std::invalid_argument("URDF position limits required for a motion-qualified axis");
        // Feedback/stop-only axis: no invented motion sector is required.
      }else{
        const auto& limits=limit_entry->second;
        if(!std::isfinite(limits.min_position)||!std::isfinite(limits.max_position)||limits.min_position>=limits.max_position)
          throw std::invalid_argument("Invalid URDF position limits");
        bounds_[i]={limits.min_position,limits.max_position};
      }
      const auto guard=j.parameters.find("estimate_guard_nm");
      if(guard==j.parameters.end()){
        if(policy_.motion_qualified[i])throw std::invalid_argument("Motion-qualified axis requires an estimate guard setting");
        policy_.guard[i]=0.;
      }else policy_.guard[i]=number(guard->second);
      std::set<std::string> found;for(const auto& s:j.state_interfaces)found.insert(s.name);
      if(j.state_interfaces.size()!=17||found!=std::set<std::string>(std::begin(state_names),std::end(state_names)))
        throw std::invalid_argument("Incomplete motion state interfaces");
    }
    policy_.validate(axis_count_);supervisor_=std::make_unique<MotionSupervisor>(policy_,bounds_,gravity_model_);
    observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
    std::ostringstream meta;meta.imbue(std::locale::classic());meta<<std::setprecision(17)<<"{\"kind\":\"SESSION\",\"backend\":\"openarm_drive_mit\",\"starts_disarmed\":true,\"hardware_motion_accepted\":false,\"driver_current_clamp_verified\":false,\"host_total_torque_clamp\":false,\"physical_torque_off_verified\":false,\"j5_queried\":"<<(axis_count_==5?"true":"false")<<",\"parameter_writes\":0,\"mapping_sha256\":\""<<h.at("mapping_sha256")
      <<"\",\"model_sha256\":\""<<h.at("model_sha256")<<"\",\"policy_sha256\":\""<<h.at("policy_sha256")<<"\",\"kp\":"<<policy_.kp<<",\"kd\":"<<policy_.kd
      <<supervisor_->gain_axis_json()
      <<",\"following_error_deg\":"<<policy_.error*180./std::acos(-1.)
      <<",\"reference_speed_rad_s\":"<<policy_.reference_speed
      <<",\"gravity_ff_alpha\":"<<policy_.gravity_alpha<<",\"gravity_ff_ramp_s\":"<<policy_.gravity_ramp
      <<",\"gravity_model_present\":"<<(gravity_model_&&!gravity_model_->empty()?"true":"false")<<",\"joint_policy\":[";
    const auto json_number=[](double value){
      std::ostringstream text;text.imbue(std::locale::classic());text<<std::setprecision(17);
      if(std::isfinite(value))text<<value;else text<<"null";return text.str();};
    for(std::size_t i=0;i<axis_count_;++i){if(i)meta<<',';meta<<"{\"axis\":"<<i+1<<",\"sign\":"<<mappings_[i].sign<<",\"zero\":"<<mappings_[i].zero
      <<",\"lower\":"<<json_number(bounds_[i].lower)<<",\"upper\":"<<json_number(bounds_[i].upper)
      <<",\"motion_qualified\":"<<(policy_.motion_qualified[i]?"true":"false")
      <<",\"observed_state_estimate_guard_nm\":"<<policy_.guard[i]<<'}';}
    meta<<"]";
    if(axis_count_==5)meta<<",\"active_axes\":5,\"j5_motion_qualified\":"<<(policy_.motion_qualified[4]?"true":"false");
    meta<<"}";metadata_=meta.str();
    const auto prefix=h.at("service_prefix");if(prefix.empty()||prefix.front()!='/')throw std::invalid_argument("Absolute service prefix required");
    const auto node=get_node();if(!node)throw std::runtime_error("Framework-managed hardware node required");
    // Read-only runtime identity: values the driver actually accepted for this
    // session, published so a capture tool can prove it matches the running stack.
    {
      const auto optional_text=[&](const char* key){
        const auto entry=h.find(key);return entry==h.end()?std::string():entry->second;};
      mass_profile_sha256_=optional_text("mass_profile_sha256");
      load_id_=optional_text("load_id");
      gravity_base_={optional_number("gravity_base_x",0.),
                     optional_number("gravity_base_y",9.80665),
                     optional_number("gravity_base_z",0.)};
      j3_offset_=optional_number("gravity_j3_offset_rad",0.);
      boot_id_=std::to_string(static_cast<long long>(now()*1e6))+"-"+std::to_string(::getpid());
      std::ostringstream identity;identity<<std::setprecision(17)
        <<"{\"schema\":\"slaihand_runtime_identity/1\""
        <<",\"mapping_sha256\":\""<<h.at("mapping_sha256")<<"\""
        <<",\"model_sha256\":\""<<h.at("model_sha256")<<"\""
        <<",\"policy_sha256\":\""<<h.at("policy_sha256")<<"\""
        <<",\"mass_profile_sha256\":\""<<mass_profile_sha256_<<"\""
        <<",\"policy_profile\":\""<<h.at("policy_profile")<<"\""
        <<",\"load_id\":\""<<load_id_<<"\""
        <<",\"gravity_base_m_s2\":["<<gravity_base_[0]<<","<<gravity_base_[1]<<","<<gravity_base_[2]<<"]"
        <<",\"j3_offset_rad\":"<<j3_offset_
        <<",\"kp\":"<<policy_.kp<<",\"kd\":"<<policy_.kd
        <<supervisor_->gain_axis_json()
        <<",\"following_error_deg\":"<<policy_.error*180./std::acos(-1.)
        <<",\"reference_speed_rad_s\":"<<policy_.reference_speed
        <<",\"gravity_ff_alpha\":"<<policy_.gravity_alpha
        <<",\"gravity_ff_ramp_s\":"<<policy_.gravity_ramp
        <<",\"gravity_float_enabled\":"<<(policy_.gravity_float_enabled?"true":"false")
        <<",\"active_axes\":"<<axis_count_
        <<",\"j5_motion_qualified\":"<<(axis_count_==5&&policy_.motion_qualified[4]?"true":"false")
        <<",\"j5_geometry\":"<<(axis_count_==5?"false":"false")<<"}";
      identity_json_=identity.str();
      identity_publisher_=node->create_publisher<std_msgs::msg::String>(
        prefix+"/runtime_identity",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      publish_identity();
    }
    ff_publisher_=node->create_publisher<std_msgs::msg::Float64MultiArray>(
      prefix+"/gravity_ff_state",rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    const auto request_arm=[this](int mode,double kd,double seconds,Trigger::Response::SharedPtr r){
      if(std::any_of(policy_.motion_qualified.begin(),policy_.motion_qualified.end(),[](bool qualified){return !qualified;})){
        r->success=false;r->message="Motion refused: an active axis is calibration-pending (J5 sign/zero/limits/TIMEOUT); feedback and stop remain available";return;}
      if(mode==2&&(!policy_.gravity_float_enabled||!gravity_model_)){
        r->success=false;r->message="Gravity float is not configured for this driver";return;}
      if(!std::isfinite(kd)||kd<0||kd>5||!std::isfinite(seconds)||seconds<1||seconds>120){
        r->success=false;r->message="Require finite float Kd in [0,5] and duration in [1,120] seconds";return;}
      std::shared_ptr<const ArmRequest> expected;
      const auto request=std::make_shared<const ArmRequest>(ArmRequest{mode,kd,seconds});
      if(!active_||reported_stage_!=int(MotionStage::DISARMED)||owned_||stop_request_||
         !std::atomic_compare_exchange_strong(&arm_request_,&expected,request)){
        r->success=false;r->message="Arm rejected: require disarmed, unclaimed position interfaces and no pending request";return;}
      r->success=true;r->message="Arm request accepted only; fresh preflight, enable and ramp are not yet confirmed";
      const auto session=std::atomic_load(&audit_path_);if(session)r->message+="; audit_file="+*session;
    };
    arm_service_=node->create_service<Trigger>(prefix+"/arm_supported_current_pose",[request_arm](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request_arm(1,5.,5.,r);});
    float_service_=node->create_service<Trigger>(prefix+"/arm_gravity_float",[request_arm](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){request_arm(2,5.,5.,r);});
    using ConfigureFloat=rcl_interfaces::srv::SetParametersAtomically;
    configured_float_service_=node->create_service<ConfigureFloat>(prefix+"/arm_gravity_float_configured",
      [request_arm](const ConfigureFloat::Request::SharedPtr request,ConfigureFloat::Response::SharedPtr response){
        double kd=std::numeric_limits<double>::quiet_NaN(),seconds=kd;
        bool have_kd=false,have_seconds=false;
        for(const auto& parameter:request->parameters){
          const auto& value=parameter.value;
          if(value.type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE){
            response->result.reason="Float settings must be doubles";return;}
          if(parameter.name=="kd"&&!have_kd){kd=value.double_value;have_kd=true;}
          else if(parameter.name=="seconds"&&!have_seconds){seconds=value.double_value;have_seconds=true;}
          else {response->result.reason="Require unique kd and seconds fields";return;}
        }
        if(!have_kd||!have_seconds){response->result.reason="Require both kd and seconds";return;}
        auto receipt=std::make_shared<Trigger::Response>();
        request_arm(2,kd,seconds,receipt);
        response->result.successful=receipt->success;response->result.reason=receipt->message;
      });
    gravity_lock_service_=node->create_service<ConfigureFloat>(prefix+"/gravity_lock_current_pose",
      [this](const ConfigureFloat::Request::SharedPtr request,ConfigureFloat::Response::SharedPtr response){
        request_gravity_mode(true,request,response);
      });
    gravity_resume_service_=node->create_service<ConfigureFloat>(prefix+"/gravity_resume_float",
      [this](const ConfigureFloat::Request::SharedPtr request,ConfigureFloat::Response::SharedPtr response){
        request_gravity_mode(false,request,response);
      });
    stop_service_=node->create_service<Trigger>(prefix+"/stop_all",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
      stop_request_=true;r->success=true;r->message="Global stop requested; verify fresh disabled observations, not this service receipt";
    });
    reset_service_=node->create_service<Trigger>(prefix+"/reset_disabled",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r){
      if(reported_stage_!=int(MotionStage::DISABLED)||owned_){r->success=false;r->message="Require disabled observations and inactive trajectory controller; UNKNOWN cannot reset";return;}
      reset_request_=true;r->success=true;r->message="Software reset requested; re-enable is a separate manual action";
    });
    publish();return Callback::SUCCESS;
  }catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"%s",e.what());return Callback::ERROR;}
}
MotionRecord CanUsbSystem::record(AuditKind kind)const{
  MotionRecord r;r.kind=kind;r.at=now();r.cycle=cycle_;r.command=supervisor_->command_id();
  r.stage=int(supervisor_->stage());r.target=supervisor_->target();
  r.kp_effective=supervisor_->kp_effective_axis();
  r.kp_schedule_target=supervisor_->kp_schedule_target_axis();
  r.kp_schedule_segment=supervisor_->kp_schedule_segment_axis();
  r.kp_scaled=supervisor_->last_scaled_kp_axis();
  r.kp_encoded=supervisor_->last_mit_kp_axis();
  r.kp_last_written_at=supervisor_->last_mit_write_at_axis();
  const bool output_stage=supervisor_->stage()==MotionStage::PRELOAD||
      supervisor_->stage()==MotionStage::ENABLING||
      supervisor_->stage()==MotionStage::RAMP||
      supervisor_->stage()==MotionStage::HOLD||
      supervisor_->stage()==MotionStage::TRACK;
  const bool any_written=std::any_of(r.kp_last_written_at.begin(),r.kp_last_written_at.end(),
                                     [](double at){return at>=0.;});
  if(output_stage)r.kp_output_state=any_written?"WRITTEN_CURRENT":"NOT_YET_WRITTEN";
  else if(supervisor_->stage()==MotionStage::STOPPING||
          supervisor_->stage()==MotionStage::DISABLED||
          supervisor_->stage()==MotionStage::UNKNOWN)
    r.kp_output_state=any_written?"STALE_DISABLED":"NO_WRITE";
  else r.kp_output_state="NOT_ARMED";
  r.request_estimate=supervisor_->request_estimate();
  r.reference=commands_;
  // Runtime identity binding + first-cause evidence. The snapshot is frozen
  // inside the supervisor before STOPPING; record_stage() only serializes it.
  r.boot_id=boot_id_;
  r.request_estimate_state=supervisor_->request_estimate_state();
  r.request_estimate_at=supervisor_->request_estimate_at();
  if(kind==AuditKind::STAGE)r.first_fault=supervisor_->first_fault_json(boot_id_,r.cycle);
  r.effective_zero.resize(mappings_.size());
  for(std::size_t i=0;i<mappings_.size();++i)r.effective_zero[i]=mappings_[i].zero;
  return r;
}
void CanUsbSystem::record_stage(){
  if(supervisor_->stage()==logged_stage_)return;
  // Latch the detection cycle once; later identity republishes must not advance
  // the frozen first fault's cycle_id with the driver-local read counter.
  if(supervisor_->has_first_fault()&&!fault_cycle_latched_){
    fault_cycle_latched_=true;fault_cycle_=cycle_;
  }
  auto r=record(AuditKind::STAGE);std::strncpy(r.reason.data(),supervisor_->reason(),r.reason.size()-1);audit_.push(r);logged_stage_=supervisor_->stage();
  // Publish the same frozen evidence on the latched runtime identity so a panel
  // or capture tool sees the first cause without re-reading the audit file.
  publish_identity();
  if(logged_stage_==MotionStage::UNKNOWN){
    RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"UNKNOWN: controlled-axis disabled observations NOT confirmed; keep physical support and cut motor power locally. Reason: %s",supervisor_->reason());
  }else if(logged_stage_==MotionStage::DISABLED){
    RCLCPP_WARN(rclcpp::get_logger("canusb_system"),"Control output disabled; host disabled observations=%s; reason=%s",supervisor_->disabled_observed()?"true":"false",supervisor_->reason());
  }
}
void CanUsbSystem::receive(){
  // Finite drain; each returned batch is limited to 4096 serial bytes.
  for(int round=0;round<4;++round){
    const auto frames=port_.receive();if(frames.empty())break;
    const auto at=now();for(const auto& f:frames){auto r=record(AuditKind::RX);r.at=at;r.action={f,Role::READ};audit_.push(r);observations_->feed(f,at);}
  }
}
void CanUsbSystem::send(const Action& a){
  auto r=record(AuditKind::SUBMITTED);r.action=a;
  const bool logged=audit_.push(r);
  if(!logged&&a.role!=Role::DISABLE)throw std::runtime_error("Audit unavailable; no further motion output");
  supervisor_->submitted(a,now());
  if(a.role==Role::READ){
    const auto& b=a.frame.data;const int rid=b[2]==0xCC?-1:b[3];const auto expected=slaihand_transport::read_packet(b[0],rid);
    if(a.frame.can_id!=0x7FF||!std::equal(b.begin(),b.end(),expected.begin()+21))throw std::logic_error("Invalid read action");
    port_.send_read(b[0],rid);
  }else port_.send_motor(a.frame);
  const auto at=now();supervisor_->written(a,at);r.kind=AuditKind::WRITTEN;r.at=at;r.stage=int(supervisor_->stage());
  if(!audit_.push(r)&&a.role!=Role::DISABLE)throw std::runtime_error("Audit failed after serial write; stop all axes");
}
void CanUsbSystem::query_one(){
  // One read request per control cycle. A 30-byte frame may block for up to
  // 5 ms on the shared USB path; sending one per axis in the same cycle made a
  // five-axis cycle burst five writes and delayed the control loop. The
  // round-robin state still refreshes all five axes within 50 ms at 100 Hz.
  const auto [i,rid]=observations_->next_request();
  const auto p=slaihand_transport::read_packet(i,rid);
  slaihand_transport::Frame f{0x7FF,{}};std::copy_n(p.begin()+21,8,f.data.begin());
  send({f,Role::READ});
}
void CanUsbSystem::publish_identity(){
  if(!identity_publisher_||identity_json_.empty())return;
  std::ostringstream identity;identity<<std::setprecision(17)
    <<identity_json_.substr(0,identity_json_.size()-1)
    <<",\"boot_id\":\""<<boot_id_<<"\",\"started_at\":"<<session_started_at_
    <<",\"position_branch_ready\":"<<(position_branch_ready_?"true":"false")
    <<",\"position_branch_modes\":[";
  for(std::size_t i=0;i<axis_count_;++i){if(i)identity<<',';identity<<'"'<<(resolve_branch_[i]?"startup_unique_urdf":"fixed")<<'"';}
  identity<<"],\"model_turns\":[";
  for(std::size_t i=0;i<axis_count_;++i){if(i)identity<<',';identity<<position_turns_[i];}
  identity<<"],\"effective_zero_rad\":[";
  for(std::size_t i=0;i<axis_count_;++i){if(i)identity<<',';identity<<mappings_[i].zero;}
  identity<<"],\"active_axes\":"<<axis_count_
          <<",\"j5_motion_qualified\":"<<(axis_count_==5&&policy_.motion_qualified[4]?"true":"false")
          // Frozen driver first cause and the current request-estimate meaning.
          // first_fault is null before the first unsafe observation and never
          // changes afterwards; request_estimate_nm lives inside it/audit.
          <<",\"first_fault\":"<<(supervisor_?supervisor_->first_fault_json(boot_id_,fault_cycle_latched_?fault_cycle_:cycle_):"null")
          <<",\"request_estimate_state\":\""
          <<(supervisor_?supervisor_->request_estimate_state():"UNAVAILABLE")<<"\""
          <<"}";
  std_msgs::msg::String message;message.data=identity.str();
  identity_publisher_->publish(message);identity_published_at_=now();
}
void CanUsbSystem::request_gravity_mode(bool hold,
    const rcl_interfaces::srv::SetParametersAtomically::Request::SharedPtr& request,
    rcl_interfaces::srv::SetParametersAtomically::Response::SharedPtr response){
  double command=-1;std::string boot;bool have_command=false,have_boot=false;
  for(const auto& parameter:request->parameters){
    const auto& value=parameter.value;
    if(parameter.name=="command_id"&&!have_command){
      if(value.type!=rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE){
        response->result.reason="command_id must be a double";return;}
      command=value.double_value;have_command=true;
    }else if(parameter.name=="boot_id"&&!have_boot){
      if(value.type!=rcl_interfaces::msg::ParameterType::PARAMETER_STRING){
        response->result.reason="boot_id must be a string";return;}
      boot=value.string_value;have_boot=true;
    }else{response->result.reason="Require unique command_id and boot_id fields";return;}
  }
  if(!have_command||!have_boot){response->result.reason="Require command_id and boot_id";return;}
  if(!std::isfinite(command)||command<0||std::floor(command)!=command||
     static_cast<std::uint64_t>(command)>std::numeric_limits<std::uint64_t>::max()){
    response->result.reason="command_id must be a nonnegative integer";return;}
  if(boot!=boot_id_){response->result.reason="Identity changed: boot_id does not match this driver session";return;}
  if(!active_||stop_request_||reported_stage_!=int(MotionStage::HOLD)||owned_){
    response->result.reason="Require active, unowned, stationary HOLD preflight with no pending stop";return;}
  const auto command_id=static_cast<std::uint64_t>(command);
  if(reported_command_id_!=command_id){
    response->result.reason="command_id changed; stale request rejected";return;}
  const int lock=reported_lock_.load();
  if(hold&&(lock!=0||!reported_float_)){
    response->result.reason="Lock requires free-drag mode with no existing lock request";return;}
  if(!hold&&lock!=int(GravityLockState::LOCKED)){
    response->result.reason="Resume requires an actual locked gravity hold";return;}
  std::shared_ptr<const GravityModeRequest> expected;
  const auto request_value=std::make_shared<const GravityModeRequest>(GravityModeRequest{hold,command_id});
  if(!std::atomic_compare_exchange_strong(&gravity_mode_request_,&expected,request_value)){
    response->result.reason="Duplicate gravity mode request already pending";return;}
  response->result.successful=true;
  response->result.reason=hold?"Lock request accepted only; locked state requires a same-cycle fresh latch and write receipt"
                              :"Resume request accepted only; free float requires the gain ramp to finish";
}
void CanUsbSystem::publish(){
  const auto at=now();const auto stage=supervisor_?supervisor_->stage():MotionStage::DISARMED;
  // A new arm/reset clears the frozen first cause in the supervisor; drop the
  // latched detection cycle with it so a later fault can latch again.
  if(supervisor_&&!supervisor_->has_first_fault()){fault_cycle_latched_=false;fault_cycle_=0;}
  reported_stage_=int(stage);ready_=supervisor_&&supervisor_->hold_ready();
  reported_lock_=supervisor_?int(supervisor_->gravity_lock_state()):0;
  reported_float_=supervisor_&&supervisor_->gravity_float();
  reported_command_id_=supervisor_?supervisor_->command_id():0;
  for(std::size_t i=0;i<axis_count_;++i){
    auto& v=values_[i];for(int j=0;j<7;++j)v[j]=std::numeric_limits<double>::quiet_NaN();
    v[7]=0;v[8]=v[9]=std::numeric_limits<double>::quiet_NaN();v[10]=0;
    v[11]=int(stage);v[12]=ready_;v[13]=audit_.good();v[14]=supervisor_&&supervisor_->disabled_observed();v[15]=owned_;
    v[16]=supervisor_?double(supervisor_->command_id()):0.;
    if(!observations_)continue;
    const auto& a=observations_->axis(int(i)+1);v[10]=a.conflict;
    if(a.registers[3])v[8]=*a.registers[3];
    if(a.registers[2])v[9]=*a.registers[2];
    if(a.feedback){const auto& f=*a.feedback;v[6]=at-f.received_at;
      if(observations_->ready(int(i)+1)&&v[6]>=0&&v[6]<=(stage==MotionStage::DISARMED?.25:policy_.age)){
        v[0]=f.position;v[1]=f.velocity;v[2]=f.effort_estimate;v[3]=f.driver_temperature;v[4]=f.motor_temperature;v[5]=f.motor_state;v[7]=1;
      }
    }
  }
  if(ff_publisher_&&supervisor_){
    const double alpha=supervisor_->gravity_alpha();
    const bool active=supervisor_->gravity_active();
    const int lock=int(supervisor_->gravity_lock_state());
    const double at=now();
    if(active||alpha!=ff_published_alpha_||active!=ff_published_active_||lock!=ff_published_lock_||at-ff_published_at_>=1.0){
      std_msgs::msg::Float64MultiArray message;
      // Explicit protocol metadata lets the panel distinguish a five-axis
      // 40/43-field payload from an old four-axis payload without guessing.
      std_msgs::msg::MultiArrayDimension axis_dim;
      axis_dim.label="slaihand_gravity_ff_state/2";
      axis_dim.size=static_cast<std::uint32_t>(axis_count_);
      axis_dim.stride=static_cast<std::uint32_t>(4*axis_count_);
      std_msgs::msg::MultiArrayDimension float_dim;
      float_dim.label="float";
      float_dim.size=policy_.gravity_float_enabled?1u:0u;
      float_dim.stride=3u;
      message.layout.dim={axis_dim,float_dim};
      message.data={alpha,supervisor_->gravity_ramp(),active?1.:0.,
                    double(reported_stage_.load()),double(supervisor_->command_id())};
      const auto append=[&message](const std::vector<double>& values){
        message.data.insert(message.data.end(),values.begin(),values.end());};
      append(supervisor_->ff_requested());
      append(supervisor_->ff_encoded());
      append(supervisor_->ff_applied());
      append(supervisor_->ff_last_written());
      if(policy_.gravity_float_enabled){
        message.data.push_back(supervisor_->gravity_float()?1.:0.);
        message.data.push_back(supervisor_->float_kd());
        message.data.push_back(supervisor_->float_seconds());
      }
      // Gravity-hold extension (always appended, 12 fields). The latch target and
      // error are model-frame radians; kp_scale is the actual ramp progress, not
      // a claim that the drive measured the commanded torque.
      message.data.push_back(double(lock));
      message.data.push_back(supervisor_->hold_gain_scale());
      message.data.push_back(supervisor_->remaining_s(at));
      append(supervisor_->hold_target());
      append(supervisor_->hold_error());
      message.data.push_back(double(supervisor_->lock_reason_code()));
      message.data.push_back(double(supervisor_->command_id()));
      ff_publisher_->publish(message);
      ff_published_alpha_=alpha;ff_published_active_=active;ff_published_lock_=lock;ff_published_at_=at;
    }
  }
  // Liveness: a single latched identity proves configuration, not that the
  // publisher is still alive, so republish the same session identity periodically.
  if(identity_publisher_&&now()-identity_published_at_>=2.0)publish_identity();
}
Callback CanUsbSystem::on_configure(const rclcpp_lifecycle::State&){
  try{
    if(!supervisor_)return Callback::ERROR;
    if(supervisor_->stage()==MotionStage::UNKNOWN)throw std::runtime_error("UNKNOWN remains locked until physical review and process restart");
    release();supervisor_=std::make_unique<MotionSupervisor>(policy_,bounds_,gravity_model_);
    fault_cycle_latched_=false;fault_cycle_=0;
    mappings_=canonical_mappings_;position_turns_.assign(axis_count_,0);position_branch_ready_=false;
    session_started_at_=now();boot_id_=std::to_string(static_cast<long long>(now()*1e6))+"-"+std::to_string(::getpid());
    publish_identity();
    observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);owned_=false;
    std::atomic_store(&arm_request_,std::shared_ptr<const ArmRequest>{});
    std::atomic_store(&gravity_mode_request_,std::shared_ptr<const GravityModeRequest>{});
    stop_request_=false;reset_request_=false;
    logged_stage_=MotionStage::UNKNOWN;audit_.open(log_directory_,metadata_);
    std::atomic_store(&audit_path_,std::make_shared<const std::string>(audit_.path()));
    port_.open(path_);
    RCLCPP_INFO(rclcpp::get_logger("canusb_system"),
      "SERIAL_LOW_LATENCY get=%d set=%d readback=%s errno=%d",
      port_.low_latency_get_result(),port_.low_latency_set_result(),
      port_.low_latency_readback()?"true":"false",port_.low_latency_errno());
    RCLCPP_INFO(rclcpp::get_logger("canusb_system"),"DISARMED; motion requires explicit arm service. Audit: %s",audit_.path().c_str());
    const auto end=now()+5.;double query_at=0;bool branch_selected=false;
    while(now()<end){receive();if(now()>=query_at){query_one();query_at=now()+.002;}
      if(observations_->conflicted())throw std::runtime_error("Device register conflict");
      if(observations_->complete(now(),policy_.age)){
        if(!branch_selected){
          auto effective=canonical_mappings_;std::vector<int> turns(axis_count_,0);
          for(std::size_t i=0;i<axis_count_;++i)if(resolve_branch_[i]){
            const auto& feedback=*observations_->axis(int(i)+1).feedback;
            if(feedback.motor_state!=0||std::abs(feedback.velocity)>.05)
              throw std::runtime_error("Position branch selection requires disabled stationary feedback");
            const auto& canonical=canonical_mappings_[i];
            const double raw=canonical.zero+canonical.sign*feedback.position;
            const auto profile=dm::Motor::get_limit_param(native_motor(int(i)+1).get_motor_type());
            const auto branch=startup_position_branch(canonical,raw,bounds_[i].lower,bounds_[i].upper,profile.pMax);
            effective[i]=branch.mapping;turns[i]=branch.model_turns;
          }
          mappings_=effective;position_turns_=turns;branch_selected=true;
          // Re-read in the locked mapping. Do not relabel a cached feedback or
          // publish the old branch as a current joint pose.
          observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
          auto branch_record=record(AuditKind::STAGE);
          std::strncpy(branch_record.reason.data(),"STARTUP_POSITION_BRANCH_SELECTED",branch_record.reason.size()-1);
          audit_.push(branch_record);query_at=0;continue;
        }
        position_branch_ready_=true;publish_identity();
        for(std::size_t i=0;i<axis_count_;++i)commands_[i]=observations_->axis(int(i)+1).feedback->position;
        record_stage();publish();return Callback::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error(axis_count_==4?"Initial four-axis read-only preflight timeout"
                                            :"Initial five-axis read-only preflight timeout; J5 motion remains refused");
  }catch(const std::exception& e){RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"%s",e.what());release();return Callback::ERROR;}
}
Callback CanUsbSystem::on_activate(const rclcpp_lifecycle::State&){
  if(!port_.is_open())return Callback::ERROR;
  active_=true;return Callback::SUCCESS;
}
bool CanUsbSystem::safe_stop(){
  if(!supervisor_||!port_.is_open())return !supervisor_||!supervisor_->attempted();
  if(supervisor_->disabled_observed())return true;
  if(!supervisor_->attempted()&&supervisor_->stage()!=MotionStage::STOPPING&&supervisor_->stage()!=MotionStage::UNKNOWN)return true;
  supervisor_->stop("LIFECYCLE_STOP",now());const auto end=now()+.32;
  while(now()<end&&supervisor_->stage()==MotionStage::STOPPING){
    try{receive();}catch(...){}
    const auto b=supervisor_->tick(now(),*observations_,commands_,false);
    for(std::size_t j=0;j<b.count;++j)try{send(b.actions[j]);}catch(...){}
    record_stage();std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  publish();return supervisor_->disabled_observed();
}
void CanUsbSystem::release(){
  active_=false;
  if(port_.is_open()&&supervisor_&&(supervisor_->attempted()||supervisor_->stage()==MotionStage::STOPPING||supervisor_->stage()==MotionStage::UNKNOWN)&&!supervisor_->disabled_observed()){
    try{safe_stop();}catch(...){}
    if(!supervisor_->disabled_observed())for(std::size_t i=1;i<=axis_count_;++i)try{port_.send_motor(native_special(int(i),false));}catch(...){}
  }
  port_.close();audit_.close();
}
Callback CanUsbSystem::on_deactivate(const rclcpp_lifecycle::State&){
  const bool stopped=safe_stop();active_=false;owned_=false;
  if(!stopped){RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"UNKNOWN: disabled observations not confirmed; physical support and local power intervention required");return Callback::ERROR;}
  return Callback::SUCCESS;
}
Callback CanUsbSystem::on_cleanup(const rclcpp_lifecycle::State&){const bool ok=safe_stop();release();return ok?Callback::SUCCESS:Callback::ERROR;}
Callback CanUsbSystem::on_shutdown(const rclcpp_lifecycle::State& s){return on_cleanup(s);}
Callback CanUsbSystem::on_error(const rclcpp_lifecycle::State&){release();return Callback::SUCCESS;}
std::vector<hardware_interface::StateInterface> CanUsbSystem::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> out;
  for(std::size_t i=0;i<axis_count_;++i)for(int j=0;j<17;++j)out.emplace_back("slaihand_joint"+std::to_string(i+1),state_names[j],&values_[i][j]);
  return out;
}
std::vector<hardware_interface::CommandInterface> CanUsbSystem::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> out;
  for(std::size_t i=0;i<axis_count_;++i)out.emplace_back("slaihand_joint"+std::to_string(i+1),"position",&commands_[i]);
  return out;
}
Result CanUsbSystem::prepare_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop){
  const int n=int(axis_count_),a=own_interfaces(start),b=own_interfaces(stop);
  if(a<0||b<0||(a!=0&&a!=n)||(b!=0&&b!=n))return Result::ERROR;
  if(a&&(reported_stage_!=int(MotionStage::HOLD)||!ready_||owned_))return Result::ERROR;
  return Result::OK;
}
Result CanUsbSystem::perform_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop){
  const int n=int(axis_count_);
  if(own_interfaces(stop)==n)owned_=false;
  if(own_interfaces(start)==n){if(reported_stage_!=int(MotionStage::HOLD)||!ready_||supervisor_->gravity_float()||supervisor_->gravity_hold())return Result::ERROR;owned_=true;}
  return Result::OK;
}
Result CanUsbSystem::read(const rclcpp::Time&,const rclcpp::Duration&){
  if(!active_)return Result::OK;
  ++cycle_;
  try{
    receive();
    // Keep feedback moving during HOLD/TRACK too, but only one request per
    // cycle. STOPPING already gets its READ batch from the supervisor.
    const auto stage=supervisor_->stage();
    if(stage!=MotionStage::STOPPING&&stage!=MotionStage::UNKNOWN)query_one();
  }catch(...){supervisor_->stop("SERIAL_READ_FAILURE",now());}
  publish();return Result::OK;
}
Result CanUsbSystem::write(const rclcpp::Time&,const rclcpp::Duration&){
  if(!active_)return Result::OK;
  try{
    if(stop_request_.exchange(false)){
      std::atomic_store(&arm_request_,std::shared_ptr<const ArmRequest>{});
      std::atomic_store(&gravity_mode_request_,std::shared_ptr<const GravityModeRequest>{});
      supervisor_->stop("OPERATOR_GLOBAL_STOP",now(),true);}
    if(reset_request_.exchange(false)&&!owned_)supervisor_->reset(now(),*observations_);
    const auto arm=std::atomic_exchange(&arm_request_,std::shared_ptr<const ArmRequest>{});
    if(arm){
      if(supervisor_->arm(now(),arm->mode==2,arm->kd,arm->seconds))observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
    }
    const auto mode=std::atomic_exchange(&gravity_mode_request_,std::shared_ptr<const GravityModeRequest>{});
    if(mode)supervisor_->request_gravity_mode(mode->hold,mode->command_id,now());
    if(!audit_.good())supervisor_->stop("AUDIT_QUEUE_OR_IO_FAILURE",now());
    const auto previous=supervisor_->stage();
    const auto b=supervisor_->tick(now(),*observations_,commands_,owned_);
    if(previous==MotionStage::PREFLIGHT&&supervisor_->stage()==MotionStage::PRELOAD)commands_=supervisor_->anchor();
    record_stage();audit_.push(record(AuditKind::CYCLE));
    for(std::size_t j=0;j<b.count;++j){
      try{send(b.actions[j]);}catch(const std::exception& error){
        RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),
                     "SERIAL_OR_AUDIT_WRITE_FAILURE action=%s: %s",
                     (b.actions[j].role==Role::READ?"READ":"MOTOR"),error.what());
        supervisor_->stop("SERIAL_OR_AUDIT_WRITE_FAILURE",now());
        // Try every FD even if one fails; no more enable/MIT from this batch.
        if(supervisor_->attempted()||supervisor_->stage()==MotionStage::STOPPING||supervisor_->stage()==MotionStage::UNKNOWN)
          for(std::size_t i=1;i<=axis_count_;++i)try{send({native_special(int(i),false),Role::DISABLE});}catch(const std::exception& disable_error){
            RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"DISABLE_AFTER_WRITE_FAILURE failed: %s",disable_error.what());
          }
        break;
      }
    }
    record_stage();publish();return Result::OK;
  }catch(...){supervisor_->stop("WRITE_SUPERVISION_FAILURE",now());
    if(supervisor_->attempted()||supervisor_->stage()==MotionStage::STOPPING||supervisor_->stage()==MotionStage::UNKNOWN)
      for(std::size_t i=1;i<=axis_count_;++i)try{send({native_special(int(i),false),Role::DISABLE});}catch(...){}
    record_stage();publish();return Result::OK;
  }
}
}  // namespace slaihand_hardware
PLUGINLIB_EXPORT_CLASS(slaihand_hardware::CanUsbSystem,hardware_interface::SystemInterface)
