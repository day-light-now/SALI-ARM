// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace slaihand_transport {
// A wire-format identifier, not a claim about the adapter's manufacturer/model.
inline constexpr char kProtocol[] = "slai_canusb_tx30_rx16_v1";
inline constexpr std::array<int, 7> kRegisters{7, 8, 9, 10, 21, 22, 23};
struct Frame { std::uint32_t can_id; std::array<std::uint8_t, 8> data; };
std::array<std::uint8_t, 30> read_packet(int axis, int reg = -1);

class Stream {
 public:
  std::vector<Frame> feed(const std::uint8_t* data, std::size_t length);
 private:
  std::vector<std::uint8_t> buffer_;
};

struct Mapping { int can_id; int sign; double zero; };
struct Feedback {
  double position, velocity, effort_estimate;
  double driver_temperature, motor_temperature, motor_state;
  double received_at;
};
struct AxisState {
  Mapping mapping;
  std::array<std::optional<double>, 7> registers;
  std::optional<Feedback> feedback;
  bool conflict = false;
};

class ReadOnlyState {
 public:
  explicit ReadOnlyState(const std::array<Mapping, 4>& mapping);
  void feed(const Frame& frame, double received_at);
  bool ready(int axis) const;
  bool complete(double now, double max_age) const;
  bool conflicted() const;
  const AxisState& axis(int axis) const;
  std::pair<int, int> next_request();
 private:
  std::array<AxisState, 4> axes_;
  std::size_t register_cursor_ = 0, feedback_cursor_ = 0;
};

// Linux/WSL serial descriptor. No generic TX, enable, FD, register-write or J5
// interface is exposed. Construction does not open a port; close sends nothing.
class ReadOnlyPort {
 public:
  ReadOnlyPort() = default;
  ~ReadOnlyPort();
  ReadOnlyPort(const ReadOnlyPort&) = delete;
  ReadOnlyPort& operator=(const ReadOnlyPort&) = delete;
  void open(const std::string& path);
  void close() noexcept;
  std::vector<Frame> receive();
  void send_read(int axis, int reg = -1);
  bool is_open() const { return fd_ >= 0; }
 private:
  friend class MotionPort;
  void write_serial(const std::array<std::uint8_t, 30>& packet);
  int fd_ = -1;
  bool exclusive_ = false;
  Stream stream_;
};

// Separate capability: only a motion owner can send MIT/FC/FD to IDs 1-4.
// The proven ReadOnlyPort API remains read-only.
class MotionPort {
 public:
  void open(const std::string& path) { port_.open(path); }
  void close() noexcept { port_.close(); }
  bool is_open() const { return port_.is_open(); }
  std::vector<Frame> receive() { return port_.receive(); }
  void send_read(int axis, int reg=-1) { port_.send_read(axis,reg); }
  void send_motor(const Frame& frame);
 private:
  ReadOnlyPort port_;
};
}  // namespace slaihand_transport
