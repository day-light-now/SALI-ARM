// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "gravity_ff.hpp"
#include "native_codec.hpp"
#include <array>
#include <memory>
#include <string>

namespace slaihand_hardware {
enum class MotionStage { DISARMED, PREFLIGHT, PRELOAD, ENABLING, RAMP, HOLD, TRACK, STOPPING, DISABLED, UNKNOWN };
enum class Role { READ, MIT, ENABLE, DISABLE };
struct Action {slaihand_transport::Frame frame;Role role;};
struct Batch {
  std::array<Action,8> actions{};std::size_t count=0;
  void add(const slaihand_transport::Frame& frame,Role role){
    if(count==actions.size())throw std::logic_error("Control batch overflow");
    actions[count++]={frame,role};
  }
};
struct MotionPolicy {
  double kp=160,kd=5;
  // Zero disables the host torque estimate/feedback threshold (not zero output).
  std::array<double,4> guard{0,0,0,0};
  double reference_speed=1,feedback_speed=1,error=.06981317007977318,temperature=50;
  double ramp=1,stagger=.03,age=.06,gap=.05,runtime=180;
  // Explicit gravity feedforward coefficients; zero disables it entirely.
  double gravity_alpha=0,gravity_ramp=1;
  bool gravity_float_enabled=false;
  void validate() const;
};
struct PositionBounds {double lower,upper;};

// Supervision only. The drive runs PD via the official MIT codec; trajectories
// come from the standard JTC. No gravity model, host PD, or auto gain increase.
class MotionSupervisor {
 public:
  MotionSupervisor(MotionPolicy policy,std::array<PositionBounds,4> bounds,
                   std::shared_ptr<const GravityFfModel> gravity_model=nullptr);
  bool arm(double at,bool gravity_float=false,double float_kd=5.,double float_seconds=5.);
  bool reset(double at,const slaihand_transport::ReadOnlyState& state);
  void stop(const char* reason,double at,bool explicit_stop=false);
  Batch tick(double at,slaihand_transport::ReadOnlyState& state,
             const std::array<double,4>& reference,bool owned);
  void submitted(const Action& action,double at);
  void written(const Action& action,double at);
  MotionStage stage() const{return stage_;}
  bool hold_ready() const{return hold_ready_;}
  bool attempted() const{return attempted_;}
  bool disabled_observed() const{return stage_==MotionStage::DISABLED&&stop_confirmed_;}
  const char* reason() const{return reason_;}
  const std::array<double,4>& anchor()const{return anchor_;}
  const std::array<double,4>& target()const{return target_;}
  const std::array<double,4>& request_estimate()const{return request_estimate_;}
  std::uint64_t command_id()const{return command_id_;}
  bool gravity_active()const{return ff_active_;}
  double gravity_alpha()const{return gravity_model_?(floating_?1.:p_.gravity_alpha):0.;}
  double gravity_ramp()const{return floating_?3.:p_.gravity_ramp;}
  bool gravity_float()const{return floating_;}
  double float_kd()const{return float_kd_;}
  double float_seconds()const{return float_seconds_;}
  const std::array<double,4>& ff_requested()const{return ff_requested_;}
  const std::array<double,4>& ff_candidate()const{return ff_candidate_;}
  const std::array<double,4>& ff_encoded()const{return ff_encoded_;}
  const std::array<double,4>& ff_applied()const{return ff_applied_;}
  const std::array<double,4>& ff_last_written()const{return ff_last_written_;}
  const std::array<double,4>& ff_written_at()const{return ff_written_at_;}
 private:
  Batch stopping(double at,const slaihand_transport::ReadOnlyState& state);
  void check_observations(double at,const slaihand_transport::ReadOnlyState& state,bool initial);
  Batch controls(double at,const slaihand_transport::ReadOnlyState& state,int enable_axis=0);
  MotionPolicy p_;
  std::array<PositionBounds,4> bounds_;
  std::shared_ptr<const GravityFfModel> gravity_model_;
  // requested: model request; candidate: after guard-headroom saturation;
  // encoded: final packet field; applied: last successfully written packet.
  std::array<double,4> ff_requested_{},ff_candidate_{},ff_encoded_{},ff_applied_{};
  std::array<double,4> ff_last_written_{},ff_written_at_{{-1.,-1.,-1.,-1.}};
  bool ff_active_=false;
  bool floating_=false;
  double float_kd_=5.,float_seconds_=5.;
  std::array<double,4> request_estimate_{};
  void clear_feedforward(bool keep_history);
  MotionStage stage_=MotionStage::DISARMED;
  const char* reason_="NOT_ARMED";
  std::array<char,128> reason_buffer_{};
  std::array<double,4> anchor_{},target_{},enable_at_{},confirmed_at_{},fd_at_{},stop_seen_{};
  std::array<unsigned,4> stop_samples_{};
  double started_=0,last_tick_=0,preload_at_=0,stable_at_=-1,stop_at_=0;
  unsigned preload_mask_=0;int next_enable_=0;
  bool attempted_=false,hold_ready_=false,stop_confirmed_=false;
  std::uint64_t command_id_=0;
};
}  // namespace slaihand_hardware
