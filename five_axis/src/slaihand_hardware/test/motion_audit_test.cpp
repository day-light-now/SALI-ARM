// Deterministic MotionAudit shape test. Uses a temporary directory, no device.
#include "slaihand_hardware/motion_audit.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
using namespace slaihand_hardware;
void require(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
std::size_t array_length(const std::string& text,const std::string& key){
  const auto marker="\""+key+"\":[";
  const auto start=text.find(marker);
  if(start==std::string::npos)throw std::runtime_error("audit JSON is missing "+key);
  const auto first=start+marker.size(),end=text.find(']',first);
  if(end==std::string::npos)throw std::runtime_error("audit JSON array is unterminated: "+key);
  if(first==end)return 0;
  std::size_t count=1;
  for(auto i=first;i<end;++i)if(text[i]==',')++count;
  return count;
}
int main(){try{
  const auto directory=std::filesystem::temp_directory_path()/"slaihand_motion_audit_test";
  std::filesystem::remove_all(directory);
  MotionAudit audit;
  audit.open(directory.string(),"{\"kind\":\"SESSION\",\"test\":true}");
  MotionRecord record;
  record.kind=AuditKind::STAGE;record.at=1.;record.cycle=2;record.command=3;record.stage=4;
  record.target={1,2,3,4,5};
  record.effective_zero={.01,.02,.03,.04,.05};
  record.reference={.6,.7,.8,.9,1.};
  record.boot_id="boot-audit-test";
  record.request_estimate_state="LAST_ACTIVE_STALE";
  record.request_estimate_at=.5;
  record.first_fault="{\"schema\":\"slaihand_first_fault/1\",\"layer\":\"DRIVER\",\"axis\":2}";
  std::strncpy(record.reason.data(),"FIVE_AXIS_TEST",record.reason.size()-1);
  require(audit.push(record),"audit rejected a five-axis stage record");
  MotionRecord cycle;
  cycle.kind=AuditKind::CYCLE;cycle.at=1.1;cycle.cycle=2;cycle.command=3;cycle.stage=4;
  cycle.target={1,2,3,4,5};
  cycle.request_estimate={.1,.2,.3,.4,.5};
  cycle.request_estimate_state="CURRENT";
  cycle.request_estimate_at=1.5;
  require(audit.push(cycle),"audit rejected a five-axis cycle record");
  audit.close();
  std::ifstream input(audit.path());
  require(input.good(),"cannot re-read the audit file");
  const std::string text((std::istreambuf_iterator<char>(input)),{});
  require(array_length(text,"target")==5,"five-axis target was padded or truncated");
  require(array_length(text,"request_estimate_nm")==5,"five-axis request estimate was padded or truncated");
  require(array_length(text,"effective_zero_rad")==5,"five-axis effective zero was padded or truncated");
  require(array_length(text,"reference_rad")==5,"five-axis reference was padded or truncated");
  require(text.find("FIVE_AXIS_TEST")!=std::string::npos,"audit dropped the stage reason");
  require(text.find("\"boot_id\":\"boot-audit-test\"")!=std::string::npos,
          "audit dropped the stage boot_id binding");
  require(text.find("\"first_fault\":{\"schema\":\"slaihand_first_fault/1\",\"layer\":\"DRIVER\",\"axis\":2}")!=std::string::npos,
          "audit dropped the frozen first-fault snapshot");
  require(text.find("\"request_estimate_state\":\"LAST_ACTIVE_STALE\"")!=std::string::npos,
          "audit did not mark post-stop request estimates stale");
  require(text.find("\"request_estimate_state\":\"CURRENT\"")!=std::string::npos,
          "audit did not mark a current-cycle request estimate");
  require(text.find("\"request_estimate_at\":1.5")!=std::string::npos,
          "audit dropped the request-estimate timestamp");
  std::filesystem::remove_all(directory);
  std::cout<<"PASS five-axis audit records keep five target/estimate/zero/reference values without padding and bind boot/first-fault/stale-estimate evidence\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
