// SPDX-License-Identifier: Apache-2.0
#include "slaihand_transport/canusb_readonly.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#include <unistd.h>

namespace slaihand_transport {
namespace {
void check_axis(int axis) {
  if (axis < 1 || axis > 5) throw std::invalid_argument("Only J1-J5 read requests are supported");
}
std::uint32_t little_u32(const std::uint8_t* p) {
  return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
    (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
[[noreturn]] void io_error(const char* message) {
  throw std::system_error(errno, std::generic_category(), message);
}
}

std::array<std::uint8_t, 30> read_packet(int axis, int reg) {
  check_axis(axis);
  if (reg != -1 && std::find(kRegisters.begin(), kRegisters.end(), reg) == kRegisters.end()) {
    throw std::invalid_argument("Register is outside the read-only whitelist");
  }
  // Byte-for-byte equivalent to verification/hardware/four_axis_monitor.py.
  std::array<std::uint8_t, 30> result{
    0x55,0xAA,0x1E,3,1,0,0,0,0x0A,0,0,0,0,0xFF,7,0,0,0,8,0,0,0,0,0,0,0,0,0,0,0};
  result[21] = axis;
  result[23] = reg == -1 ? 0xCC : 0x33;
  result[24] = reg == -1 ? 0 : reg;
  return result;
}

std::vector<Frame> Stream::feed(const std::uint8_t* data, std::size_t length) {
  std::vector<Frame> result;
  // Process incrementally: arbitrary input size cannot grow an unbounded buffer.
  for (std::size_t i = 0; i < length; ++i) {
    buffer_.push_back(data[i]);
    while (!buffer_.empty()) {
      if (buffer_[0] != 0xAA) { buffer_.erase(buffer_.begin()); continue; }
      if (buffer_.size() < 16) break;
      if (buffer_[1] != 0x11 || buffer_[2] != 8 || buffer_[15] != 0x55) {
        buffer_.erase(buffer_.begin()); continue;
      }
      Frame frame{little_u32(buffer_.data()+3), {}};
      std::copy_n(buffer_.begin()+7, 8, frame.data.begin());
      // This profile is standard CAN only, matching the already verified link.
      if (frame.can_id <= 0x7FF) result.push_back(frame);
      buffer_.erase(buffer_.begin(), buffer_.begin()+16);
    }
  }
  return result;
}

ReadOnlyState::ReadOnlyState(const std::vector<Mapping>& mapping) {
  if (mapping.empty() || mapping.size() > 5) throw std::invalid_argument("ReadOnlyState requires 1..5 axes");
  for (std::size_t i = 0; i < mapping.size(); ++i) {
    if (mapping[i].can_id != int(i+1) || (mapping[i].sign != 1 && mapping[i].sign != -1) ||
        !std::isfinite(mapping[i].zero)) throw std::invalid_argument("Invalid canonical calibration");
    AxisState axis; axis.mapping = mapping[i]; axes_.push_back(axis);
  }
}
const AxisState& ReadOnlyState::axis(int i) const {
  if (i < 1 || std::size_t(i) > axes_.size()) throw std::invalid_argument("Axis outside configured count");
  return axes_[i-1];
}
bool ReadOnlyState::ready(int i) const {
  const auto& a = axis(i);
  return !a.conflict && std::all_of(a.registers.begin(), a.registers.end(), [](auto& r) {return r.has_value();});
}
bool ReadOnlyState::conflicted() const {
  return std::any_of(axes_.begin(), axes_.end(), [](const auto& a) {return a.conflict;});
}
bool ReadOnlyState::complete(double now, double max_age) const {
  for (std::size_t i = 1; i <= axes_.size(); ++i) {
    const auto& a = axis(int(i));
    if (!ready(int(i)) || !a.feedback || now < a.feedback->received_at ||
        now - a.feedback->received_at > max_age) return false;
  }
  return true;
}
void ReadOnlyState::feed(const Frame& frame, double received_at) {
  if (!std::isfinite(received_at) || frame.can_id != 0) return;
  const auto& p = frame.data;
  const auto rid = std::find(kRegisters.begin(), kRegisters.end(), p[3]);
  if (p[0] >= 1 && p[0] <= axes_.size() && p[1] == 0 && p[2] == 0x33 && rid == kRegisters.end()) return;
  if (p[0] >= 1 && p[0] <= axes_.size() && p[1] == 0 && p[2] == 0x33 && rid != kRegisters.end()) {
    auto& a = axes_[p[0]-1];
    const auto bits = little_u32(p.data()+4);
    double value = bits;
    if (p[3] >= 21) { float f; std::memcpy(&f, &bits, sizeof(f)); value = f; }
    const auto index = std::distance(kRegisters.begin(), rid);
    const auto& old = a.registers[index];
    if (!std::isfinite(value) || (p[3] >= 21 && value <= 0) ||
        (p[3] == 7 && value != 0) || (p[3] == 8 && value != p[0]) ||
        (old && *old != value)) {
      a.conflict = true; a.feedback.reset(); return;
    }
    a.registers[index] = value;
    return;
  }
  const int i = p[0] & 0xF;
  if (i < 1 || std::size_t(i) > axes_.size() || !ready(i)) return;
  auto& a = axes_[i-1];
  const double pmax = *a.registers[4], vmax = *a.registers[5], tmax = *a.registers[6];
  const auto position = ((p[1] << 8) | p[2]) * (2*pmax)/65535.0 - pmax;
  const auto velocity = ((p[3] << 4) | (p[4] >> 4)) * (2*vmax)/4095.0 - vmax;
  const auto torque = (((p[4] & 15) << 8) | p[5]) * (2*tmax)/4095.0 - tmax;
  a.feedback = Feedback{a.mapping.sign*(position-a.mapping.zero), a.mapping.sign*velocity,
    a.mapping.sign*torque, double(p[6]), double(p[7]), double(p[0] >> 4), received_at};
}
std::pair<int, int> ReadOnlyState::next_request() {
  const std::size_t slots = axes_.size() * kRegisters.size();
  for (std::size_t n = 0; n < slots; ++n) {
    const auto slot = register_cursor_++ % slots;
    const std::size_t i = slot % axes_.size();
    const auto reg = slot / axes_.size();
    if (!axes_[i].registers[reg]) return {int(i+1), kRegisters[reg]};
  }
  return {int(feedback_cursor_++ % axes_.size())+1, -1};
}

ReadOnlyPort::~ReadOnlyPort() { close(); }
void ReadOnlyPort::open(const std::string& path) {
  if (fd_ >= 0 || path.empty()) throw std::invalid_argument("Explicit unused CANUSB serial path required");
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) io_error("Cannot open CANUSB serial port");
  try {
    if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) io_error("CANUSB port is already owned");
    if (::ioctl(fd_, TIOCEXCL) < 0) io_error("Cannot claim exclusive CANUSB port");
    exclusive_ = true;
    termios settings{};
    if (::tcgetattr(fd_, &settings) < 0) io_error("Cannot read serial attributes");
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    settings.c_cc[VMIN] = 0; settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, B921600) < 0 || ::cfsetospeed(&settings, B921600) < 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) < 0) io_error("Cannot set 921600 8N1");
    if (::tcflush(fd_, TCIFLUSH) < 0) io_error("Cannot clear stale serial input");
    // Best-effort USB-serial latency reduction. cdc_acm may reject this ioctl;
    // failure is deliberately non-fatal and never changes the protocol.
    serial_struct serial{};
    low_latency_get_result_ = ::ioctl(fd_, TIOCGSERIAL, &serial);
    low_latency_errno_ = low_latency_get_result_ < 0 ? errno : 0;
    if (low_latency_get_result_ == 0) {
      serial.flags |= ASYNC_LOW_LATENCY;
      low_latency_set_result_ = ::ioctl(fd_, TIOCSSERIAL, &serial);
      low_latency_errno_ = low_latency_set_result_ < 0 ? errno : 0;
      serial_struct readback{};
      low_latency_readback_ = ::ioctl(fd_, TIOCGSERIAL, &readback) == 0 &&
          (readback.flags & ASYNC_LOW_LATENCY) != 0;
    }
    stream_ = Stream{};
  } catch (...) { close(); throw; }
}
void ReadOnlyPort::close() noexcept {
  if (fd_ >= 0) {
    if (exclusive_) ::ioctl(fd_, TIOCNXCL);
    ::close(fd_); fd_ = -1; exclusive_ = false;
  }
}
std::vector<Frame> ReadOnlyPort::receive() {
  if (fd_ < 0) throw std::runtime_error("CANUSB is closed");
  pollfd descriptor{fd_, POLLIN, 0};
  if (::poll(&descriptor, 1, 0) > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {
    throw std::runtime_error("CANUSB disconnected");
  }
  std::uint8_t bytes[4096];
  const auto count = ::read(fd_, bytes, sizeof(bytes));
  if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) io_error("CANUSB read failed");
  return count > 0 ? stream_.feed(bytes, count) : std::vector<Frame>{};
}
void ReadOnlyPort::send_read(int axis, int reg) {
  const auto packet = read_packet(axis, reg);  // whitelist enforced before any write
  write_serial(packet);
}
void ReadOnlyPort::write_serial(const std::array<std::uint8_t,30>& packet) {
  if (fd_ < 0) throw std::runtime_error("CANUSB is closed");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
  std::size_t sent = 0;
  while (sent < packet.size()) {
    const auto count = ::write(fd_, packet.data()+sent, packet.size()-sent);
    if (count > 0) sent += count;
    else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) io_error("CANUSB write failed");
    if (sent == packet.size()) break;
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("CANUSB incomplete serial packet write");
    pollfd descriptor{fd_, POLLOUT, 0};
    ::poll(&descriptor, 1, 1);
  }
}
void MotionPort::send_motor(const Frame& frame) {
  check_axis(frame.can_id);
  const bool special=std::all_of(frame.data.begin(),frame.data.begin()+7,[](auto b){return b==0xFF;});
  if (special && frame.data[7]!=0xFC && frame.data[7]!=0xFD) {
    throw std::invalid_argument("Zero/clear/other motor commands are not authorized");
  }
  auto packet=read_packet(frame.can_id);
  packet[13]=frame.can_id; packet[14]=0;
  std::copy(frame.data.begin(),frame.data.end(),packet.begin()+21);
  port_.write_serial(packet);
}
}  // namespace slaihand_transport
