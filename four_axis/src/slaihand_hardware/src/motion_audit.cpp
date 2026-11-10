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
void MotionAudit::write(const MotionRecord& r){
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
    s<<",\"target\":[";for(int i=0;i<4;++i){if(i)s<<',';s<<r.target[i];}s<<']';
    if(r.kind==AuditKind::CYCLE){
      s<<",\"request_estimate_nm\":[";
      for(int i=0;i<4;++i){if(i)s<<',';s<<r.request_estimate[i];}s<<']';
    }
    if(r.kind==AuditKind::STAGE){
      s<<",\"reason\":\""<<r.reason.data()<<'"'<<",\"effective_zero_rad\":[";
      for(int i=0;i<4;++i){if(i)s<<',';s<<r.effective_zero[i];}s<<']';
      s<<",\"reference_rad\":[";
      for(int i=0;i<4;++i){if(i)s<<',';if(std::isfinite(r.reference[i]))s<<r.reference[i];else s<<"null";}s<<']';
    }
  }
  s<<"}\n";const auto text=s.str();
  if(std::fwrite(text.data(),1,text.size(),file_)!=text.size())failed_=true;
}
void MotionAudit::run(){
  for(;;){
    auto t=tail_.load(std::memory_order_relaxed);
    if(t!=head_.load(std::memory_order_acquire)){
      auto r=queue_[t];tail_.store((t+1)%queue_.size(),std::memory_order_release);write(r);continue;
    }
    if(std::fflush(file_)!=0)failed_=true;
    if(closing_)break;
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wake_.wait_for(lock,std::chrono::milliseconds(10),[this]{return closing_||head_!=tail_;});
  }
}
void MotionAudit::close(){
  closing_=true;wake_.notify_one();if(worker_.joinable())worker_.join();
  if(file_){std::fclose(file_);file_=nullptr;}
}
}  // namespace slaihand_hardware
