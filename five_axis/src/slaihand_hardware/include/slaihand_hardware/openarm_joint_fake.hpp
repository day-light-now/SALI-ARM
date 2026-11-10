// Copyright 2025 Enactic, Inc. (upstream interface structure)
// Copyright 2026 SLAIHand project (configurable offline adaptation)
// SPDX-License-Identifier: Apache-2.0
// Adapted from OpenArmHW's lifecycle and position/state interface structure.
#pragma once
#include <string>
#include <vector>
#include <hardware_interface/system_interface.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>
#include "slaihand_hardware/mapping.hpp"

namespace slaihand_hardware {
class OpenArmJointFake : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  std::vector<std::string> names_;
  std::vector<double> positions_, velocities_, commands_;
  std::vector<double> lower_, upper_, max_velocity_;
  std::vector<JointMapping> mappings_;
  std::vector<openarm::damiao_motor::Motor> motors_;
  std::vector<openarm::damiao_motor::CANPacket> offline_packets_;
};
}  // namespace slaihand_hardware
