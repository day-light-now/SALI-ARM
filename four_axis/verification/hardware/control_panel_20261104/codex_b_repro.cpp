// Offline reviewer counterexamples: use the production supervisor and fixture.
#define main original_fixture_main
#include "../../../src/slaihand_hardware/test/motion_supervisor_test.cpp"
#undef main
#include <memory>
std::shared_ptr<const GravityFfModel> reviewer_model(){
  return std::make_shared<const GravityFfModel>(GravityFfModel::parse(
    "G:0:0:-9.8|L:link1:0.1:0.5:0:0"
    "|J:slaihand_joint1:base:link1:0:0:0:0:0:0:0:1:0:0:-4:4:r"
    "|J:slaihand_joint2:link1:link2:0:0:0:0:0:0:0:0:1:0:-4:4:r"
    "|J:slaihand_joint3:link2:link3:0:0:0:0:0:0:0:0:1:0:-4:4:r"
    "|J:slaihand_joint4:link3:link4:0:0:0:0:0:0:0:0:1:0:-4:4:r"));
}
int main(){
  for(double ramp:{1.,10.}){
    Fixture f;MotionPolicy p;p.gravity_alpha=.5;p.gravity_ramp=ramp;
    f.core=MotionSupervisor(p,Fixture::bounds(),reviewer_model());f.hold();
    std::cout<<"configured_ff_ramp="<<ramp<<" ff_after_2.2s="<<f.core.ff_applied()[0]<<'\n';
    f.core.stop("TEST_STOP",f.at,true);for(int n=0;n<8;++n)f.tick();
    std::cout<<"after_stop stage="<<int(f.core.stage())<<" active="<<f.core.gravity_active()
             <<" ff_applied="<<f.core.ff_applied()[0]<<'\n';
  }
  int crossed=0;
  for(int axis=0;axis<4;++axis)for(bool upper:{false,true}){
    Fixture f;MotionPolicy p;p.gravity_alpha=.5;
    const auto bounds=Fixture::bounds();double boundary=upper?bounds[axis].upper:bounds[axis].lower;
    f.q[axis]=boundary+(upper?-1:1)*3*25./65535.;f.fresh();
    f.core=MotionSupervisor(p,bounds,reviewer_model());f.hold();auto ref=f.core.target();
    for(int n=0;n<20;++n){ref[axis]=upper?std::min(boundary,ref[axis]+.0001):std::max(boundary,ref[axis]-.0001);
      auto b=f.tick(.01,true,ref);
      for(std::size_t j=0;j<b.count;++j)if(b.actions[j].role==Role::MIT){
        int i=b.actions[j].frame.can_id-1;double q=Fixture::maps()[i].sign*decode_native(i+1,b.actions[j].frame.data).q;
        if(q<bounds[i].lower-1e-12||q>bounds[i].upper+1e-12){++crossed;std::cout<<"OUTSIDE_ENCODED_LIMIT axis="<<i+1<<" q="<<q<<'\n';}
      }
    }
  }
  std::cout<<"boundary_crossings="<<crossed<<'\n';
}
