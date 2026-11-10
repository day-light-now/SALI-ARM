// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "gravity_ff.hpp"
#include "motion_audit.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

namespace slaihand_hardware {
class CanUsbSystem : public hardware_interface::SystemInterface {
 public:
  ~CanUsbSystem()override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams&)override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&)override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&)override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&)override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&)override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&)override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&)override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>&,const std::vector<std::string>&)override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>&,const std::vector<std::string>&)override;
  hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&)override;
  hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration&)override;
 private:
  static double now();
  void receive();
  void send(const Action& action);
  void query_four();
  void publish();
  void publish_identity();
  void record_stage();
  MotionRecord record(AuditKind kind)const;
  bool safe_stop();
  void release();
  std::array<slaihand_transport::Mapping,4> mappings_{};
  std::array<slaihand_transport::Mapping,4> canonical_mappings_{};
  std::array<bool,4> resolve_branch_{};
  std::array<int,4> position_turns_{};
  bool position_branch_ready_=false;
  double session_started_at_=0;
  std::array<std::array<double,17>,4> values_{};
  std::array<double,4> commands_{};
  std::array<PositionBounds,4> bounds_{};
  std::unique_ptr<slaihand_transport::ReadOnlyState> observations_;
  std::unique_ptr<MotionSupervisor> supervisor_;
  std::shared_ptr<const GravityFfModel> gravity_model_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ff_publisher_;
  double ff_published_alpha_=-1;
  bool ff_published_active_=false;
  double ff_published_at_=0;
  // Read-only runtime identity (driver-accepted configuration, not repo files).
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr identity_publisher_;
  std::string identity_json_, mass_profile_sha256_, load_id_, boot_id_;
  std::array<double,3> gravity_base_{};
  double j3_offset_=0;
  double identity_published_at_=0;
  slaihand_transport::MotionPort port_;
  MotionAudit audit_;
  MotionPolicy policy_;
  std::string path_,log_directory_,metadata_;
  std::shared_ptr<const std::string> audit_path_;
  std::uint64_t cycle_=0;
  MotionStage logged_stage_=MotionStage::UNKNOWN;
  std::atomic<int> reported_stage_{0};
  struct ArmRequest {int mode;double kd,seconds;};
  std::shared_ptr<const ArmRequest> arm_request_;
  std::atomic<bool> stop_request_{false},reset_request_{false},ready_{false},owned_{false};
  std::atomic<bool> active_{false};
  using Trigger=std_srvs::srv::Trigger;
  rclcpp::Service<Trigger>::SharedPtr arm_service_,float_service_,stop_service_,reset_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr configured_float_service_;
};
}  // namespace slaihand_hardware
