// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/canusb_readonly.hpp"
#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <locale>
#include <unistd.h>
#include <set>
#include <thread>
#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace slaihand_hardware {
using Callback = hardware_interface::CallbackReturn;
using Result = hardware_interface::return_type;
namespace {
constexpr const char* names[] = {"position", "velocity", "effort", "driver_temperature",
  "motor_temperature", "motor_state", "feedback_age", "feedback_valid", "control_mode",
  "timeout_count", "configuration_conflict"};
constexpr double kMaxAge = 0.25;  // Read-only validity, not a motion watchdog.
double number(const std::string& text) {
  std::size_t end = 0;
  const auto value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) throw std::invalid_argument("Invalid calibration number");
  return value;
}
}
double CanUsbReadOnly::now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
CanUsbReadOnly::~CanUsbReadOnly() { close_audit(); }
void CanUsbReadOnly::close_audit() noexcept {
  if (audit_) { std::fclose(audit_); audit_ = nullptr; }
}
void CanUsbReadOnly::open_audit() {
  close_audit();
  std::filesystem::create_directories(log_directory_);
  auto filename = (std::filesystem::path(log_directory_)/"canusb_XXXXXX.jsonl").string();
  const auto fd = ::mkstemps(filename.data(), 6);  // Atomic, exclusive new file.
  if (fd < 0) throw std::runtime_error("Cannot create CANUSB audit file");
  audit_ = ::fdopen(fd, "w");
  if (!audit_) { ::close(fd); throw std::runtime_error("Cannot open CANUSB audit stream"); }
  std::ostringstream row; row.imbue(std::locale::classic()); row << std::setprecision(17);
  row << "{\"kind\":\"SESSION\",\"protocol\":\"" << slaihand_transport::kProtocol
      << "\",\"readonly\":true,\"mapping_sha256\":\"" << mapping_hash_
      << "\",\"model_sha256\":\"" << model_hash_ << "\",\"mapping\":[";
  for (int i=0; i<4; ++i) {
    if (i) row << ',';
    row << "{\"axis\":" << i+1 << ",\"sign\":" << mappings_[i].sign << ",\"zero\":" << mappings_[i].zero << '}';
  }
  row << "]}\n";
  const auto text = row.str();
  if (std::fwrite(text.data(),1,text.size(),audit_) != text.size() || std::fflush(audit_) != 0) {
    close_audit(); throw std::runtime_error("Cannot write CANUSB session audit");
  }
  RCLCPP_INFO(rclcpp::get_logger("canusb_readonly"), "Read-only CANUSB audit: %s", filename.c_str());
}
void CanUsbReadOnly::audit_frame(bool tx, const slaihand_transport::Frame& frame) {
  if (!audit_) throw std::runtime_error("CANUSB audit is not open");
  const auto unix_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  std::ostringstream row; row.imbue(std::locale::classic()); row << std::setprecision(17);
  row << "{\"kind\":\"" << (tx ? "TX_READ_WRITTEN" : "RX_FRAME")
      << "\",\"direction\":\"" << (tx ? "tx" : "rx") << "\",\"time\":" << unix_s
      << ",\"steady_s\":" << now() << ",\"can_id\":" << frame.can_id << ",\"payload\":\"";
  for (std::size_t i=0;i<frame.data.size();++i) {
    if (i) row << ' ';
    row << std::hex << std::setfill('0') << std::setw(2) << int(frame.data[i]);
  }
  row << "\"";
  if (tx) row << ",\"serial_bytes_written\":30,\"drive_execution_ack\":false";
  row << "}\n";
  const auto text = row.str();
  if (std::fwrite(text.data(),1,text.size(),audit_) != text.size() || std::fflush(audit_) != 0) {
    throw std::runtime_error("Cannot persist CANUSB frame audit");
  }
}
void CanUsbReadOnly::invalidate() {
  for (auto& row : values_) {
    row.fill(std::numeric_limits<double>::quiet_NaN()); row[7] = 0.; row[10] = 0.;
  }
}
Callback CanUsbReadOnly::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != Callback::SUCCESS) return Callback::ERROR;
  invalidate();
  try {
    if (info.hardware_parameters.at("protocol") != slaihand_transport::kProtocol || info.joints.size() != 4) {
      throw std::invalid_argument("Require the verified CANUSB wire profile and exactly J1-J4");
    }
    path_ = info.hardware_parameters.at("port");
    if (path_.rfind("/dev/",0) != 0) throw std::invalid_argument("Explicit Linux/WSL serial path required; COM9 is a Windows name");
    log_directory_ = info.hardware_parameters.at("log_directory");
    if (!std::filesystem::path(log_directory_).is_absolute()) throw std::invalid_argument("Absolute audit directory required");
    auto hash = [&](const char* key) {
      const auto item=info.hardware_parameters.find(key);
      const auto value=item==info.hardware_parameters.end() ? std::string{} : item->second;
      if (!value.empty() && (value.size()!=64 || value.find_first_not_of("0123456789abcdef")!=std::string::npos)) {
        throw std::invalid_argument("Invalid source SHA256");
      }
      return value;
    };
    mapping_hash_=hash("mapping_sha256"); model_hash_=hash("model_sha256");
    for (int i = 0; i < 4; ++i) {
      const auto& joint = info.joints[i];
      if (joint.name != "slaihand_joint"+std::to_string(i+1) || !joint.command_interfaces.empty()) {
        throw std::invalid_argument("Read-only plugin refuses command interfaces or extra joints");
      }
      std::set<std::string> actual;
      for (const auto& s : joint.state_interfaces) actual.insert(s.name);
      if (actual != std::set<std::string>(std::begin(names), std::end(names)) || joint.state_interfaces.size() != 11) {
        throw std::invalid_argument("Incomplete or duplicate read-only state interfaces");
      }
      const auto id = number(joint.parameters.at("can_id"));
      const auto sign = number(joint.parameters.at("hardware_sign"));
      if (id != i+1 || (sign != -1 && sign != 1)) throw std::invalid_argument("Invalid CANUSB mapping");
      mappings_[i] = {i+1, int(sign), number(joint.parameters.at("zero_offset_rad"))};
    }
    state_ = std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("canusb_readonly"), "%s", e.what()); return Callback::ERROR;
  }
  return Callback::SUCCESS;  // on_init never opens the serial descriptor.
}
void CanUsbReadOnly::poll_once(double at) {
  for (const auto& frame : port_.receive()) { audit_frame(false, frame); state_->feed(frame, at); }
  if (state_->conflicted()) throw std::runtime_error("Conflicting/invalid device registers; reconnect after checking IDs");
  if (at >= next_poll_) {
    const auto request = state_->next_request();
    port_.send_read(request.first, request.second);
    const auto packet=slaihand_transport::read_packet(request.first,request.second);
    slaihand_transport::Frame frame{0x7FF,{}};
    std::copy_n(packet.begin()+21,8,frame.data.begin()); audit_frame(true,frame);
    next_poll_ = at + 0.01;  // Target one read query/10ms, not four-axis 100Hz.
  }
  publish(at);
}
void CanUsbReadOnly::publish(double at) {
  for (int i = 1; i <= 4; ++i) {
    auto& row = values_[i-1];
    const auto& a = state_->axis(i);
    row[7] = 0.; row[10] = a.conflict;
    if (a.registers[3]) row[8] = *a.registers[3];
    if (a.registers[2]) row[9] = *a.registers[2];
    if (a.feedback) row[6] = at - a.feedback->received_at;
    if (a.feedback && state_->ready(i) && row[6] >= 0 && row[6] <= kMaxAge) {
      const auto& f = *a.feedback;
      row[0] = f.position; row[1] = f.velocity; row[2] = f.effort_estimate;
      row[3] = f.driver_temperature; row[4] = f.motor_temperature; row[5] = f.motor_state;
      row[7] = 1.;
    } else {
      // Never present stale q/dq/effort as current measurements.
      for (int j = 0; j < 6; ++j) row[j] = std::numeric_limits<double>::quiet_NaN();
    }
  }
}
Callback CanUsbReadOnly::on_configure(const rclcpp_lifecycle::State&) {
  try {
    if (!state_) throw std::runtime_error("Read-only hardware is not initialized");
    port_.close(); close_audit(); invalidate(); active_ = false;
    state_ = std::make_unique<slaihand_transport::ReadOnlyState>(mappings_);
    open_audit(); port_.open(path_); next_poll_ = 0.;
    const auto deadline = now()+5.;
    while (now() < deadline) {
      poll_once(now());
      if (state_->complete(now(), kMaxAge)) return Callback::SUCCESS;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    throw std::runtime_error("Four-axis read-only configuration/feedback timeout; no motor command sent");
  } catch (const std::exception& e) {
    port_.close(); close_audit(); invalidate();
    RCLCPP_ERROR(rclcpp::get_logger("canusb_readonly"), "%s", e.what()); return Callback::ERROR;
  }
}
Callback CanUsbReadOnly::on_activate(const rclcpp_lifecycle::State&) {
  if (!port_.is_open()) return Callback::ERROR;
  active_ = true; return Callback::SUCCESS;  // Activates reporting, not motors.
}
Callback CanUsbReadOnly::on_deactivate(const rclcpp_lifecycle::State&) {
  active_ = false; invalidate(); return Callback::SUCCESS;
}
Callback CanUsbReadOnly::on_cleanup(const rclcpp_lifecycle::State&) {
  active_ = false; port_.close(); close_audit(); invalidate(); return Callback::SUCCESS;
}
Callback CanUsbReadOnly::on_shutdown(const rclcpp_lifecycle::State& s) { return on_cleanup(s); }
Callback CanUsbReadOnly::on_error(const rclcpp_lifecycle::State& s) { return on_cleanup(s); }
std::vector<hardware_interface::StateInterface> CanUsbReadOnly::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (int i = 0; i < 4; ++i) for (int j = 0; j < 11; ++j) {
    result.emplace_back("slaihand_joint"+std::to_string(i+1), names[j], &values_[i][j]);
  }
  return result;
}
Result CanUsbReadOnly::read(const rclcpp::Time&, const rclcpp::Duration&) {
  if (!active_) return Result::OK;
  try { poll_once(now()); return Result::OK; }
  catch (const std::exception& e) {
    active_ = false; port_.close(); close_audit(); invalidate();
    RCLCPP_ERROR(rclcpp::get_logger("canusb_readonly"), "%s", e.what()); return Result::ERROR;
  }
}
}  // namespace slaihand_hardware
PLUGINLIB_EXPORT_CLASS(slaihand_hardware::CanUsbReadOnly, hardware_interface::SystemInterface)
