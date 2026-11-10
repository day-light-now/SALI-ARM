#include "slaihand_hardware/position_branch.hpp"
#include "slaihand_hardware/native_codec.hpp"
#include <iostream>
#include <limits>
using namespace slaihand_hardware;
int main(){try{
  const double pi=std::acos(-1.),turn=2*pi;
  for(int sign:{-1,1})for(int n:{-1,0,1})for(double q:{-.8,0.,.8}){
    const slaihand_transport::Mapping canonical{3,sign,-2.5423437857633324};
    const double raw=canonical.zero+sign*(q+turn*n);
    if(std::abs(raw)>12.5)continue;
    const auto branch=startup_position_branch(canonical,raw,-pi/2,pi/2,12.5);
    if(branch.model_turns!=n||std::abs(sign*(raw-branch.mapping.zero)-q)>1e-12)return 1;
    const auto frame=native_command(3,branch.mapping,q,160,5);
    const auto decoded=decode_native(3,frame.data);
    if(std::abs(decoded.q-raw)>25./65535.+1e-12)return 2;
    const auto next=decode_native(3,native_command(3,branch.mapping,q+.01,160,5).data);
    if(std::abs((next.q-decoded.q)-sign*.01)>2*25./65535.)return 3;
    // A different raw branch later remains a discontinuity in the pinned mapping.
    if(std::abs(sign*(raw+sign*turn-branch.mapping.zero)-q-turn)>1e-12)return 4;
  }
  const slaihand_transport::Mapping j3{3,1,-2.5423437857633324};
  for(double raw:{3.3834973678187232,-2.556436558174411}){
    const auto b=startup_position_branch(j3,raw,-pi/2,pi/2,12.5);
    const double q=raw-b.mapping.zero;
    if(q<-.4||q>.1)return 5;
  }
  bool missing=false,ambiguous=false,invalid=false;
  try{startup_position_branch({3,1,0},pi,-pi/2,pi/2,12.5);}catch(...){missing=true;}
  try{startup_position_branch({3,1,0},0,0,turn,12.5);}catch(...){ambiguous=true;}
  try{startup_position_branch(j3,std::numeric_limits<double>::quiet_NaN(),-pi/2,pi/2,12.5);}catch(...){invalid=true;}
  if(!missing||!ambiguous||!invalid)return 6;
  std::cout<<"PASS startup branches +/-2pi, signs, pinned RX/TX and official MIT takeover; invalid/ambiguous refused\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 7;}}
