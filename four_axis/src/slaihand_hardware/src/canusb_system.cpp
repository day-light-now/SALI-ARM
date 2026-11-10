// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/canusb_system.hpp"
#include "slaihand_hardware/position_branch.hpp"
#include <unistd.h>
#include <filesystem>
#include <limits>
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
    bool valid=false;for(int i=1;i<=4;++i)valid|=s=="slaihand_joint"+std::to_string(i)+"/position";
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
    if(info.joints.size()!=4||h.at("protocol")!=slaihand_transport::kProtocol||h.at("policy_profile")!="supported_four_axis_native_mit_b1")
      throw std::invalid_argument("Require explicit four-axis CANUSB native policy");
    path_=h.at("port");log_directory_=h.at("log_directory");
    if(path_.rfind("/dev/",0)!=0||!std::filesystem::path(log_directory_).is_absolute())throw std::invalid_argument("Explicit device/audit paths required");
    for(const char* key:{"mapping_sha256","model_sha256","policy_sha256"})if(!is_hash(h.at(key)))throw std::invalid_argument("Motion requires source fingerprints");
    policy_.kp=number(h.at("kp"));policy_.kd=number(h.at("kd"));
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
    const auto model_entry=h.find("gravity_model");
    if(model_entry!=h.end()&&!model_entry->second.empty()){
      try{
        gravity_model_=std::make_shared<const GravityFfModel>(GravityFfModel::parse(model_entry->second));
      }catch(const std::exception& error){
        throw std::invalid_argument(std::string("Invalid gravity model parameter: ")+error.what());
      }
      const auto& names=gravity_model_->joint_names();
      for(int i=0;i<4;++i)
        if(names[i]!="slaihand_joint"+std::to_string(i+1))
          throw std::invalid_argument("Gravity model joint order must match the four canonical joints");
    }
    for(int i=0;i<4;++i){
      const auto& j=info.joints[i];
      if(j.name!="slaihand_joint"+std::to_string(i+1)||j.command_interfaces.size()!=1||j.command_interfaces[0].name!="position")
        throw std::invalid_argument("Only four native position command interfaces are supported");
      if(j.command_interfaces[0].parameters.count("min")||j.command_interfaces[0].parameters.count("max"))throw std::invalid_argument("Do not duplicate URDF limits");
      const auto sign=number(j.parameters.at("hardware_sign"));
      if(number(j.parameters.at("can_id"))!=i+1||(sign!=1&&sign!=-1))throw std::invalid_argument("Invalid canonical mapping");
      mappings_[i]={i+1,int(sign),number(j.parameters.at("zero_offset_rad"))};
      canonical_mappings_[i]=mappings_[i];
      const auto mode=j.parameters.find("position_branch_mode");
      const std::string branch_mode=mode==j.parameters.end()?"fixed":mode->second;
      if(branch_mode!="fixed"&&branch_mode!="startup_unique_urdf")throw std::invalid_argument("Unknown position branch mode");
      resolve_branch_[i]=branch_mode=="startup_unique_urdf";
      const auto& limits=info.limits.at(j.name);
      if(!limits.has_position_limits)throw std::invalid_argument("URDF position limits required");
      bounds_[i]={limits.min_position,limits.max_position};policy_.guard[i]=number(j.parameters.at("estimate_guard_nm"));
      std::set<std::string> found;for(const auto& s:j.state_interfaces)found.insert(s.name);
      if(j.state_interfaces.size()!=17||found!=std::set<std::string>(std::begin(state_names),std::end(state_names)))
        throw std::invalid_argument("Incomplete motion state interfaces");
    }
    policy_.validate();supervisor_=std::make_unique<MotionSupervisor>(policy_,bounds_,gravity_model_);
    observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
    commands_.fill(std::numeric_limits<double>::quiet_NaN());
    std::ostringstream meta;meta<<std::setprecision(17)<<"{\"kind\":\"SESSION\",\"backend\":\"openarm_drive_mit\",\"starts_disarmed\":true,\"hardware_motion_accepted\":false,\"driver_current_clamp_verified\":false,\"host_total_torque_clamp\":false,\"physical_torque_off_verified\":false,\"j5_queried\":false,\"parameter_writes\":0,\"mapping_sha256\":\""<<h.at("mapping_sha256")
      <<"\",\"model_sha256\":\""<<h.at("model_sha256")<<"\",\"policy_sha256\":\""<<h.at("policy_sha256")<<"\",\"kp\":"<<policy_.kp<<",\"kd\":"<<policy_.kd
      <<",\"reference_speed_rad_s\":"<<policy_.reference_speed
      <<",\"gravity_ff_alpha\":"<<policy_.gravity_alpha<<",\"gravity_ff_ramp_s\":"<<policy_.gravity_ramp
      <<",\"gravity_model_present\":"<<(gravity_model_&&!gravity_model_->empty()?"true":"false")<<",\"joint_policy\":[";
    for(int i=0;i<4;++i){if(i)meta<<',';meta<<"{\"axis\":"<<i+1<<",\"sign\":"<<mappings_[i].sign<<",\"zero\":"<<mappings_[i].zero
      <<",\"lower\":"<<bounds_[i].lower<<",\"upper\":"<<bounds_[i].upper<<",\"observed_state_estimate_guard_nm\":"<<policy_.guard[i]<<'}';}
    meta<<"]}";metadata_=meta.str();
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
        <<",\"reference_speed_rad_s\":"<<policy_.reference_speed
        <<",\"gravity_ff_alpha\":"<<policy_.gravity_alpha
        <<",\"gravity_ff_ramp_s\":"<<policy_.gravity_ramp
        <<",\"gravity_float_enabled\":"<<(policy_.gravity_float_enabled?"true":"false")<<"}";
      identity_json_=identity.str();
      identity_publisher_=node->create_publisher<std_msgs::msg::String>(
        prefix+"/runtime_identity",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
      publish_identity();
    }
    ff_publisher_=node->create_publisher<std_msgs::msg::Float64MultiArray>(
      prefix+"/gravity_ff_state",rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    const auto request_arm=[this](int mode,double kd,double seconds,Trigger::Response::SharedPtr r){
      if(mode==2&&(!policy_.gravity_float_enabled||!gravity_model_)){
        r->success=false;r->message="Gravity float is not configured for this driver";return;}
      if(!std::isfinite(kd)||kd<0||kd>5||!std::isfinite(seconds)||seconds<1||seconds>120){
        r->success=false;r->message="Require finite float Kd in [0,5] and duration in [1,120] seconds";return;}
      std::shared_ptr<const ArmRequest> expected;
      const auto request=std::make_shared<const ArmRequest>(ArmRequest{mode,kd,seconds});
      if(!active_||reported_stage_!=int(MotionStage::DISARMED)||owned_||stop_request_||
         !std::atomic_compare_exchange_strong(&arm_request_,&expected,request)){
        r->success=false;r->message="Arm rejected: require disarmed, unclaimed position interfaces and no pending request";return;}
      r->success=true;r->message="Arm request accepted only; four-axis fresh preflight, enable and ramp are not yet confirmed";
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
  r.request_estimate=supervisor_->request_estimate();
  r.reference=commands_;
  for(int i=0;i<4;++i)r.effective_zero[i]=mappings_[i].zero;
  return r;
}
void CanUsbSystem::record_stage(){
  if(supervisor_->stage()==logged_stage_)return;
  auto r=record(AuditKind::STAGE);std::strncpy(r.reason.data(),supervisor_->reason(),r.reason.size()-1);audit_.push(r);logged_stage_=supervisor_->stage();
  if(logged_stage_==MotionStage::UNKNOWN){
    RCLCPP_ERROR(rclcpp::get_logger("canusb_system"),"UNKNOWN: four-axis disabled observations NOT confirmed; keep physical support and cut motor power locally. Reason: %s",supervisor_->reason());
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
void CanUsbSystem::query_four(){
  for(int j=0;j<4;++j){const auto [i,rid]=observations_->next_request();const auto p=slaihand_transport::read_packet(i,rid);
    slaihand_transport::Frame f{0x7FF,{}};std::copy_n(p.begin()+21,8,f.data.begin());send({f,Role::READ});}
}
void CanUsbSystem::publish_identity(){
  if(!identity_publisher_||identity_json_.empty())return;
  std::ostringstream identity;identity<<std::setprecision(17)
    <<identity_json_.substr(0,identity_json_.size()-1)
    <<",\"boot_id\":\""<<boot_id_<<"\",\"started_at\":"<<session_started_at_
    <<",\"position_branch_ready\":"<<(position_branch_ready_?"true":"false")
    <<",\"position_branch_modes\":[";
  for(int i=0;i<4;++i){if(i)identity<<',';identity<<'"'<<(resolve_branch_[i]?"startup_unique_urdf":"fixed")<<'"';}
  identity<<"],\"model_turns\":[";
  for(int i=0;i<4;++i){if(i)identity<<',';identity<<position_turns_[i];}
  identity<<"],\"effective_zero_rad\":[";
  for(int i=0;i<4;++i){if(i)identity<<',';identity<<mappings_[i].zero;}
  identity<<"]}";
  std_msgs::msg::String message;message.data=identity.str();
  identity_publisher_->publish(message);identity_published_at_=now();
}
void CanUsbSystem::publish(){
  const auto at=now();const auto stage=supervisor_?supervisor_->stage():MotionStage::DISARMED;
  reported_stage_=int(stage);ready_=supervisor_&&supervisor_->hold_ready();
  for(int i=0;i<4;++i){
    auto& v=values_[i];for(int j=0;j<7;++j)v[j]=std::numeric_limits<double>::quiet_NaN();
    v[7]=0;v[8]=v[9]=std::numeric_limits<double>::quiet_NaN();v[10]=0;
    v[11]=int(stage);v[12]=ready_;v[13]=audit_.good();v[14]=supervisor_&&supervisor_->disabled_observed();v[15]=owned_;
    v[16]=supervisor_?double(supervisor_->command_id()):0.;
    if(!observations_)continue;
    const auto& a=observations_->axis(i+1);v[10]=a.conflict;
    if(a.registers[3])v[8]=*a.registers[3];
    if(a.registers[2])v[9]=*a.registers[2];
    if(a.feedback){const auto& f=*a.feedback;v[6]=at-f.received_at;
      if(observations_->ready(i+1)&&v[6]>=0&&v[6]<=(stage==MotionStage::DISARMED?.25:policy_.age)){
        v[0]=f.position;v[1]=f.velocity;v[2]=f.effort_estimate;v[3]=f.driver_temperature;v[4]=f.motor_temperature;v[5]=f.motor_state;v[7]=1;
      }
    }
  }
  if(ff_publisher_&&supervisor_){
    const double alpha=supervisor_->gravity_alpha();
    const bool active=supervisor_->gravity_active();
    const double at=now();
    if(active||alpha!=ff_published_alpha_||active!=ff_published_active_||at-ff_published_at_>=1.0){
      std_msgs::msg::Float64MultiArray message;
      message.data={alpha,supervisor_->gravity_ramp(),active?1.:0.,
                    double(reported_stage_.load()),double(supervisor_->command_id())};
      const auto append=[&message](const std::array<double,4>& values){
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
      ff_publisher_->publish(message);
      ff_published_alpha_=alpha;ff_published_active_=active;ff_published_at_=at;
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
    mappings_=canonical_mappings_;position_turns_.fill(0);position_branch_ready_=false;
    session_started_at_=now();boot_id_=std::to_string(static_cast<long long>(now()*1e6))+"-"+std::to_string(::getpid());
    publish_identity();
    observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);owned_=false;
    std::atomic_store(&arm_request_,std::shared_ptr<const ArmRequest>{});stop_request_=false;reset_request_=false;
    logged_stage_=MotionStage::UNKNOWN;audit_.open(log_directory_,metadata_);
    std::atomic_store(&audit_path_,std::make_shared<const std::string>(audit_.path()));port_.open(path_);
    RCLCPP_INFO(rclcpp::get_logger("canusb_system"),"DISARMED; motion requires explicit arm service. Audit: %s",audit_.path().c_str());
    const auto end=now()+5.;double query_at=0;bool branch_selected=false;
    while(now()<end){receive();if(now()>=query_at){query_four();query_at=now()+.002;}
      if(observations_->conflicted())throw std::runtime_error("Device register conflict");
      if(observations_->complete(now(),policy_.age)){
        if(!branch_selected){
          auto effective=canonical_mappings_;std::array<int,4> turns{};
          for(int i=0;i<4;++i)if(resolve_branch_[i]){
            const auto& feedback=*observations_->axis(i+1).feedback;
            if(feedback.motor_state!=0||std::abs(feedback.velocity)>.05)
              throw std::runtime_error("Position branch selection requires disabled stationary feedback");
            const auto& canonical=canonical_mappings_[i];
            const double raw=canonical.zero+canonical.sign*feedback.position;
            const auto profile=dm::Motor::get_limit_param(native_motor(i+1).get_motor_type());
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
        for(int i=0;i<4;++i)commands_[i]=observations_->axis(i+1).feedback->position;
        record_stage();publish();return Callback::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("Initial four-axis read-only preflight timeout");
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
    if(!supervisor_->disabled_observed())for(int i=1;i<=4;++i)try{port_.send_motor(native_special(i,false));}catch(...){}
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
  std::vector<hardware_interface::StateInterface> out;for(int i=0;i<4;++i)for(int j=0;j<17;++j)out.emplace_back("slaihand_joint"+std::to_string(i+1),state_names[j],&values_[i][j]);return out;
}
std::vector<hardware_interface::CommandInterface> CanUsbSystem::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> out;for(int i=0;i<4;++i)out.emplace_back("slaihand_joint"+std::to_string(i+1),"position",&commands_[i]);return out;
}
Result CanUsbSystem::prepare_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop){
  const int a=own_interfaces(start),b=own_interfaces(stop);
  if(a<0||b<0||(a!=0&&a!=4)||(b!=0&&b!=4))return Result::ERROR;
  if(a&&(reported_stage_!=int(MotionStage::HOLD)||!ready_||owned_))return Result::ERROR;
  return Result::OK;
}
Result CanUsbSystem::perform_command_mode_switch(const std::vector<std::string>& start,const std::vector<std::string>& stop){
  if(own_interfaces(stop)==4)owned_=false;
  if(own_interfaces(start)==4){if(reported_stage_!=int(MotionStage::HOLD)||!ready_||supervisor_->gravity_float())return Result::ERROR;owned_=true;}
  return Result::OK;
}
Result CanUsbSystem::read(const rclcpp::Time&,const rclcpp::Duration&){
  if(!active_)return Result::OK;
  ++cycle_;
  try{receive();}catch(...){supervisor_->stop("SERIAL_READ_FAILURE",now());}
  publish();return Result::OK;
}
Result CanUsbSystem::write(const rclcpp::Time&,const rclcpp::Duration&){
  if(!active_)return Result::OK;
  try{
    if(stop_request_.exchange(false)){
      std::atomic_store(&arm_request_,std::shared_ptr<const ArmRequest>{});supervisor_->stop("OPERATOR_GLOBAL_STOP",now(),true);}
    if(reset_request_.exchange(false)&&!owned_)supervisor_->reset(now(),*observations_);
    const auto arm=std::atomic_exchange(&arm_request_,std::shared_ptr<const ArmRequest>{});
    if(arm){
      if(supervisor_->arm(now(),arm->mode==2,arm->kd,arm->seconds))observations_=std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
    }
    if(!audit_.good())supervisor_->stop("AUDIT_QUEUE_OR_IO_FAILURE",now());
    const auto previous=supervisor_->stage();
    const auto b=supervisor_->tick(now(),*observations_,commands_,owned_);
    if(previous==MotionStage::PREFLIGHT&&supervisor_->stage()==MotionStage::PRELOAD)commands_=supervisor_->anchor();
    record_stage();audit_.push(record(AuditKind::CYCLE));
    for(std::size_t j=0;j<b.count;++j){
      try{send(b.actions[j]);}catch(...){
        supervisor_->stop("SERIAL_OR_AUDIT_WRITE_FAILURE",now());
        // Try every FD even if one fails; no more enable/MIT from this batch.
        if(supervisor_->attempted()||supervisor_->stage()==MotionStage::STOPPING||supervisor_->stage()==MotionStage::UNKNOWN)
          for(int i=1;i<=4;++i)try{send({native_special(i,false),Role::DISABLE});}catch(...){}
        break;
      }
    }
    const auto stage=supervisor_->stage();
    if(stage==MotionStage::DISARMED||stage==MotionStage::PREFLIGHT||stage==MotionStage::DISABLED)query_four();
    record_stage();publish();return Result::OK;
  }catch(...){supervisor_->stop("WRITE_SUPERVISION_FAILURE",now());
    if(supervisor_->attempted()||supervisor_->stage()==MotionStage::STOPPING||supervisor_->stage()==MotionStage::UNKNOWN)
      for(int i=1;i<=4;++i)try{send({native_special(i,false),Role::DISABLE});}catch(...){}
    record_stage();publish();return Result::OK;
  }
}
}  // namespace slaihand_hardware
PLUGINLIB_EXPORT_CLASS(slaihand_hardware::CanUsbSystem,hardware_interface::SystemInterface)
