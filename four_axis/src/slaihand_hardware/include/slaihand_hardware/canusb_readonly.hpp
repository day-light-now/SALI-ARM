// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <memory>
#include <cstdio>
#include <hardware_interface/system_interface.hpp>
#include <slaihand_transport/canusb_readonly.hpp>

namespace slaihand_hardware {
class CanUsbReadOnly : public hardware_interface::SystemInterface {
 public:
  ~CanUsbReadOnly() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override { return {}; }
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override {
    return hardware_interface::return_type::OK;  // No TX; no command interfaces.
  }
 private:
  static double now();
  void poll_once(double at);
  void publish(double at);
  void invalidate();
  void open_audit();
  void close_audit() noexcept;
  void audit_frame(bool tx, const slaihand_transport::Frame& frame);
  std::array<slaihand_transport::Mapping, 4> mappings_;
  std::array<std::array<double, 11>, 4> values_;
  std::unique_ptr<slaihand_transport::ReadOnlyState> state_;
  slaihand_transport::ReadOnlyPort port_;
  std::string path_;
  std::string log_directory_, mapping_hash_, model_hash_;
  std::FILE* audit_ = nullptr;
  double next_poll_ = 0.;
  bool active_ = false;
};
}  // namespace slaihand_hardware
