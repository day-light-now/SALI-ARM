// SPDX-License-Identifier: Apache-2.0
#include "slaihand_hardware/motion_audit.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <locale>
#include <cmath>
#include <unistd.h>

namespace slaihand_hardware {
void MotionAudit::open(const std::string& directory,const std::string& metadata){
  close();std::filesystem::create_directories(directory);
  path_=(std::filesystem::path(directory)/"canusb_motion_XXXXXX.jsonl").string();
  const int fd=::mkstemps(path_.data(),6);if(fd<0)throw std::runtime_error("Cannot create motion audit");
  file_=::fdopen(fd,"w");if(!file_){::close(fd);throw std::runtime_error("Cannot open motion audit stream");}
  const auto first=metadata+"\n";
  if(std::fwrite(first.data(),1,first.size(),file_)!=first.size()||std::fflush(file_)!=0){
    std::fclose(file_);file_=nullptr;throw std::runtime_error("Cannot persist motion metadata");
  }
  head_=0;tail_=0;failed_=false;closing_=false;worker_=std::thread([this]{run();});
}
bool MotionAudit::push(const MotionRecord& r){
  if(failed_||closing_||!file_)return false;
  const auto h=head_.load(std::memory_order_relaxed),next=(h+1)%queue_.size();
  if(next==tail_.load(std::memory_order_acquire)){failed_=true;wake_.notify_one();return false;}
  queue_[h]=r;head_.store(next,std::memory_order_release);wake_.notify_one();return true;
}
std::string MotionAudit::format(const MotionRecord& r) const{
  std::ostringstream s;s.imbue(std::locale::classic());s<<std::setprecision(17);
  const char* names[]{"RX","TX_SUBMITTED","TX_WRITTEN","CONTROL_CYCLE","STAGE"};
  s<<"{\"kind\":\""<<names[int(r.kind)]<<"\",\"steady_s\":"<<r.at<<",\"cycle_id\":"<<r.cycle
   <<",\"command_id\":"<<r.command<<",\"control_state\":"<<r.stage;
  if(r.kind==AuditKind::RX||r.kind==AuditKind::SUBMITTED||r.kind==AuditKind::WRITTEN){
    s<<",\"role\":"<<int(r.action.role)<<",\"can_id\":"<<r.action.frame.can_id<<",\"payload\":\"";
    for(std::size_t i=0;i<8;++i){if(i)s<<' ';s<<std::hex<<std::setw(2)<<std::setfill('0')<<int(r.action.frame.data[i]);}
    s<<std::dec<<"\",\"drive_execution_ack\":false";
    if(r.kind==AuditKind::WRITTEN)s<<",\"serial_bytes_written\":30";
  }else{
    s<<",\"target\":[";for(std::size_t i=0;i<r.target.size();++i){if(i)s<<',';s<<r.target[i];}s<<']';
    if(r.kind==AuditKind::CYCLE||r.kind==AuditKind::STAGE){
      s<<",\"kp_scheduled\":[";
      for(std::size_t i=0;i<r.kp_effective.size();++i){if(i)s<<',';s<<r.kp_effective[i];}
      s<<"]";
      s<<",\"kp_schedule_target\":[";
      for(std::size_t i=0;i<r.kp_schedule_target.size();++i){if(i)s<<',';s<<r.kp_schedule_target[i];}
      s<<"]";
      s<<",\"kp_schedule_segment\":[";
      for(std::size_t i=0;i<r.kp_schedule_segment.size();++i){if(i)s<<',';s<<r.kp_schedule_segment[i];}
      s<<"]";
      s<<",\"kp_scaled\":[";
      for(std::size_t i=0;i<r.kp_scaled.size();++i){if(i)s<<',';if(std::isfinite(r.kp_scaled[i])&&r.kp_scaled[i]>=0)s<<r.kp_scaled[i];else s<<"null";}
      s<<"]";
      s<<",\"kp_encoded\":[";
      for(std::size_t i=0;i<r.kp_encoded.size();++i){if(i)s<<',';if(std::isfinite(r.kp_encoded[i])&&r.kp_encoded[i]>=0)s<<r.kp_encoded[i];else s<<"null";}
      s<<"]";
      s<<",\"kp_last_written_at\":[";
      for(std::size_t i=0;i<r.kp_last_written_at.size();++i){if(i)s<<',';if(std::isfinite(r.kp_last_written_at[i])&&r.kp_last_written_at[i]>=0)s<<r.kp_last_written_at[i];else s<<"null";}
      s<<"]";
      s<<",\"kp_output_state\":\""<<r.kp_output_state<<"\"";
    }
    if(r.kind==AuditKind::CYCLE){
      s<<",\"request_estimate_nm\":[";
      for(std::size_t i=0;i<r.request_estimate.size();++i){if(i)s<<',';s<<r.request_estimate[i];}s<<']';
      // The values above are retained after a stop; this flag says whether they
      // are still this cycle's output or only the last active cycle's estimate.
      s<<",\"request_estimate_state\":\""<<r.request_estimate_state<<"\"";
      s<<",\"request_estimate_at\":";
      if(std::isfinite(r.request_estimate_at)&&r.request_estimate_at>=0)s<<r.request_estimate_at;
      else s<<"null";
    }
    if(r.kind==AuditKind::STAGE){
      s<<",\"reason\":\""<<r.reason.data()<<'"'<<",\"effective_zero_rad\":[";
      for(std::size_t i=0;i<r.effective_zero.size();++i){if(i)s<<',';s<<r.effective_zero[i];}s<<']';
      s<<",\"reference_rad\":[";
      for(std::size_t i=0;i<r.reference.size();++i){if(i)s<<',';if(std::isfinite(r.reference[i]))s<<r.reference[i];else s<<"null";}s<<']';
      // Frozen first cause plus session binding. last_complete_write_at etc. are
      // inside the first_fault object; this record is the durable evidence that
      // the operator report asks for.
      s<<",\"boot_id\":\""<<r.boot_id<<"\"";
      s<<",\"first_fault\":"<<(r.first_fault.empty()?"null":r.first_fault);
      s<<",\"request_estimate_state\":\""<<r.request_estimate_state<<"\"";
    }
  }
  s<<"}\n";return s.str();
}
void MotionAudit::run(){
  std::string buffer;
  auto flush_buffer=[&](){
    if(buffer.empty())return;
    if(std::fwrite(buffer.data(),1,buffer.size(),file_)!=buffer.size())failed_=true;
    buffer.clear();
  };
  for(;;){
    bool drained=false;
    auto t=tail_.load(std::memory_order_relaxed);
    while(t!=head_.load(std::memory_order_acquire)&&!failed_){
      buffer+=format(queue_[t]);
      t=(t+1)%queue_.size();
      tail_.store(t,std::memory_order_release);
      drained=true;
      if(buffer.size()>=262144)flush_buffer();
    }
    if(drained){flush_buffer();if(std::fflush(file_)!=0)failed_=true;}
    if(failed_)break;
    if(closing_&&tail_.load(std::memory_order_acquire)==head_.load(std::memory_order_acquire))break;
    if(!drained){
      std::unique_lock<std::mutex> lock(wait_mutex_);
      wake_.wait_for(lock,std::chrono::milliseconds(10),[this]{return closing_||head_!=tail_;});
    }
  }
  flush_buffer();
  if(file_&&std::fflush(file_)!=0)failed_=true;
}
void MotionAudit::close(){
  closing_=true;wake_.notify_one();if(worker_.joinable())worker_.join();
  if(file_){std::fclose(file_);file_=nullptr;}
}
}  // namespace slaihand_hardware
