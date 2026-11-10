// Copyright 2025 Enactic, Inc. (upstream interface structure)
// Copyright 2026 SLAIHand project (configurable offline adaptation)
// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/openarm_joint_fake.hpp"
#include <algorithm>
#include <limits>
#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace slaihand_hardware {
namespace dm = openarm::damiao_motor;
using Callback = hardware_interface::CallbackReturn;
using Result = hardware_interface::return_type;

Callback OpenArmJointFake::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != Callback::SUCCESS) return Callback::ERROR;
  const auto transport = info.hardware_parameters.find("transport");
  if (transport == info.hardware_parameters.end() || transport->second != "fake" ||
      (info.joints.size() != 4 && info.joints.size() != 5)) {
    RCLCPP_ERROR(rclcpp::get_logger("slaihand_hardware"),
                 "Offline adapter requires transport=fake and four or five configured joints.");
    return Callback::ERROR;
  }
  // These are upstream codec profiles, NOT calibrated 2EC motor specifications.
  const auto count = info.joints.size();
  names_.resize(count); positions_.assign(count, 0.0); velocities_.assign(count, 0.0);
  commands_.assign(count, 0.0); lower_.resize(count); upper_.resize(count);
  max_velocity_.resize(count); mappings_.resize(count); offline_packets_.resize(count);
  motors_.clear();
  try {
    for (std::size_t i = 0; i < count; ++i) {
      const auto joint = info.joints.begin() + i;
      names_[i] = joint->name;
      if (joint->command_interfaces.size() != 1 ||
          joint->command_interfaces[0].name != "position" || joint->state_interfaces.size() != 2) {
        throw std::invalid_argument("Unexpected joint names or interfaces");
      }
      bool has_position = false, has_velocity = false;
      for (const auto& interface : joint->state_interfaces) {
        has_position |= interface.name == "position";
        has_velocity |= interface.name == "velocity";
      }
      if (!has_position || !has_velocity) throw std::invalid_argument("Missing state interface");
      const bool virtual_joint = joint->parameters.count("virtual_placeholder") &&
        joint->parameters.at("virtual_placeholder") == "true";
      // Jazzy parses these from the URDF joint limits. Do not maintain another
      // min/max table in the ros2_control command interfaces.
      if (virtual_joint) {
        lower_[i] = -std::numeric_limits<double>::infinity();
        upper_[i] = std::numeric_limits<double>::infinity();
      } else {
        const auto& limits = info.limits.at(joint->name);
        if (!limits.has_position_limits) throw std::invalid_argument("Missing URDF position limits");
        lower_[i] = limits.min_position;
        upper_[i] = limits.max_position;
      }
      // Explicit simulation speed, not a motor rating.
      max_velocity_[i] = std::stod(joint->parameters.at("sim_max_velocity"));
      if (!std::isfinite(max_velocity_[i]) || max_velocity_[i] <= 0.0) {
        throw std::invalid_argument("Invalid simulation speed limit");
      }
      if (!virtual_joint && (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) ||
                    lower_[i] > 0.0 || upper_[i] < 0.0)) {
        throw std::invalid_argument("Invalid limits or zero outside joint limits");
      }
      const auto sign = joint->parameters.find("sim_sign");
      const auto zero = joint->parameters.find("sim_zero_offset");
      mappings_[i].sign = sign == joint->parameters.end() ? 1.0 : std::stod(sign->second);
      mappings_[i].zero = zero == joint->parameters.end() ? 0.0 : std::stod(zero->second);
      mappings_[i].validate();
      // ID 0 is a synthetic offline identifier, never transmitted on a bus.
      const auto& profile = joint->parameters.at("codec_profile");
      dm::MotorType type;
      if (profile == "DM10010L") type = dm::MotorType::DM10010L;
      else if (profile == "DM8009") type = dm::MotorType::DM8009;
      else if (profile == "DM4340") type = dm::MotorType::DM4340;
      else throw std::invalid_argument("Unsupported offline codec profile");
      motors_.emplace_back(type, 0, 0);
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("slaihand_hardware"), "%s", error.what());
    return Callback::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("slaihand_hardware"),
    "OpenArm offline adapter loaded with %zu configured joints; no CAN socket.", count);
  return Callback::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OpenArmJointFake::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < names_.size(); ++i) {
    interfaces.emplace_back(names_[i], "position", &positions_[i]);
    interfaces.emplace_back(names_[i], "velocity", &velocities_[i]);
  }
  return interfaces;
}
std::vector<hardware_interface::CommandInterface> OpenArmJointFake::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < names_.size(); ++i) interfaces.emplace_back(names_[i], "position", &commands_[i]);
  return interfaces;
}
Callback OpenArmJointFake::on_activate(const rclcpp_lifecycle::State&) {
  commands_ = positions_;  // Hold the current state; no automatic return-to-zero.
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
  return Callback::SUCCESS;
}
Callback OpenArmJointFake::on_deactivate(const rclcpp_lifecycle::State&) {
  commands_ = positions_;
  std::fill(velocities_.begin(), velocities_.end(), 0.0);
  return Callback::SUCCESS;
}
Result OpenArmJointFake::read(const rclcpp::Time&, const rclcpp::Duration&) {
  return Result::OK;  // Virtual feedback is updated in write().
}
Result OpenArmJointFake::write(const rclcpp::Time&, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) return Result::ERROR;
  // Validate the whole command before updating any axis.
  for (std::size_t i = 0; i < names_.size(); ++i) {
    if (!std::isfinite(commands_[i])) return Result::ERROR;
    const double target = std::clamp(commands_[i], lower_[i], upper_[i]);
    if (std::abs(mappings_[i].to_motor(target)) > dm::Motor::get_limit_param(motors_[i].get_motor_type()).pMax) {
      return Result::ERROR;  // Codec range only, not a claimed mechanical limit.
    }
  }
  for (std::size_t i = 0; i < names_.size(); ++i) {
    const double target = std::clamp(commands_[i], lower_[i], upper_[i]);
    const double next = positions_[i] + std::clamp(target - positions_[i], -max_velocity_[i]*dt, max_velocity_[i]*dt);
    const double motor_position = mappings_[i].to_motor(next);
    velocities_[i] = (next - positions_[i]) / dt;
    // Reuse the actual upstream MIT encoder. Packets stay in memory.
    offline_packets_[i] = dm::CanPacketEncoder::create_mit_control_command(
      motors_[i], {0.0, 0.0, motor_position, mappings_[i].sign * velocities_[i], 0.0});
    positions_[i] = mappings_[i].to_joint(motor_position);
  }
  return Result::OK;
}
}  // namespace slaihand_hardware
PLUGINLIB_EXPORT_CLASS(slaihand_hardware::OpenArmJointFake, hardware_interface::SystemInterface)
