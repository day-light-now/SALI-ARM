// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "motion_supervisor.hpp"
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

namespace slaihand_hardware {
enum class AuditKind { RX, SUBMITTED, WRITTEN, CYCLE, STAGE };
struct MotionRecord {
  AuditKind kind{};double at=0;
  std::uint64_t cycle=0,command=0;
  Action action{};int stage=0;
  // Vector storage follows the active axis count (four or five). No record is
  // padded to a historical width; consumers must read the JSON array length.
  std::vector<double> target{};
  std::vector<double> kp_effective{};
  std::vector<double> kp_schedule_target{};
  std::vector<int> kp_schedule_segment{};
  std::vector<double> kp_scaled{};
  std::vector<double> kp_encoded{};
  std::vector<double> kp_last_written_at{};
  std::string kp_output_state{"UNKNOWN"};
  std::vector<double> request_estimate{};
  std::vector<double> effective_zero{};
  std::vector<double> reference{};
  // Session identity binding and first-fault evidence. STAGE records carry the
  // frozen first_fault object (or "null"); every non-RX record carries whether
  // the request estimate is still current output or only last-active evidence.
  std::string boot_id{};
  std::string request_estimate_state{"UNAVAILABLE"};
  double request_estimate_at=-1;
  std::string first_fault{"null"};
  std::array<char,128> reason{};
};
// One control-loop producer; all JSON formatting and file I/O is off-cycle.
// Overflow is an observable fault, not permission to silently drop audit data.
class MotionAudit {
 public:
  ~MotionAudit(){close();}
  void open(const std::string& directory,const std::string& metadata);
  bool push(const MotionRecord& record);
  bool good()const{return !failed_.load();}
  void close();
  const std::string& path()const{return path_;}
 private:
  void run();
  std::string format(const MotionRecord& record) const;
  std::array<MotionRecord,8192> queue_{};
  std::atomic<std::size_t> head_{0},tail_{0};
  std::atomic<bool> failed_{false},closing_{false};
  std::FILE* file_=nullptr;std::string path_;
  std::thread worker_;std::mutex wait_mutex_;std::condition_variable wake_;
};
}  // namespace slaihand_hardware
