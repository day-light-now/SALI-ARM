// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "gravity_ff.hpp"
#include "native_codec.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace slaihand_hardware {
enum class MotionStage { DISARMED, PREFLIGHT, PRELOAD, ENABLING, RAMP, HOLD, TRACK, STOPPING, DISABLED, UNKNOWN };
enum class GravityLockState { NONE=0, LOCKING=1, LOCKED=2, RELEASING=3 };
enum class Role { READ, MIT, ENABLE, DISABLE };
struct Action {slaihand_transport::Frame frame;Role role;};
struct Batch {
  // Worst case is five DISABLE plus five READ in the stopping path. Overflow is
  // a hard fault, never a silent drop.
  std::array<Action,16> actions{};std::size_t count=0;
  void add(const slaihand_transport::Frame& frame,Role role){
    if(count==actions.size())throw std::logic_error("Control batch overflow");
    actions[count++]={frame,role};
  }
};
struct KpSchedulePoint {
  double angle_deg=0;
  double kp=0;
};
struct MotionPolicy {
  double kp=160,kd=5;
  // Explicit adopted position-gain ceiling. Defaults preserve the legacy 160
  // policy; experimental branches must opt in by setting kp_max explicitly.
  // The protocol encoding ceiling (500) is a separate wire-format bound and is
  // never treated as a validated mechanical cap.
  double kp_max=160;
  // Optional five-axis independent position gains, J1..J5 order. An empty list
  // keeps the legacy scalar kp/kd for every active axis, so old four-axis and
  // five-axis policy files/parameters stay valid. Both lists are additive:
  // an absent list falls back to its scalar. They are accepted only for the
  // five-axis active set; a four-axis session must keep the scalar policy.
  // Kp is bounded by the explicit policy kp_max (default 160; an experiment
  // must set kp_max explicitly). Kd stays at the adopted 5 cap. NaN/Inf, wrong
  // length and out-of-range entries are refused.
  std::vector<double> kp_axis,kd_axis;
  // Optional angle-scheduled Kp. Empty disables scheduling; otherwise there is
  // one strictly increasing point list per active axis. Angles are in URDF/model
  // degrees (the same frame as the position state), and Kp is bounded by the
  // same adopted cap as kp_axis. Linear interpolation between points, clamp at
  // both ends, and a rate-limited transition to the interpolated target.
  std::vector<std::vector<KpSchedulePoint>> kp_schedule;
  double kp_schedule_slew=80.0;  // Kp units per second
  // Zero disables the host torque estimate/feedback threshold (not zero output).
  // A five-axis caller must supply five entries; the four-axis default is kept
  // for source compatibility.
  std::vector<double> guard{0,0,0,0};
  // Per-axis motion qualification. Empty retains the legacy four-axis meaning
  // (J1-J4 qualified, any J5 is calibration-pending). A five-axis caller must
  // supply exactly five entries; a false entry refuses the whole-arm preflight
  // before any enable, and its URDF bounds may be left undefined because they
  // are never used for motion.
  std::vector<bool> motion_qualified;
  double reference_speed=1,feedback_speed=1,error=.06981317007977318,temperature=50;
  double ramp=1,stagger=.03,age=.06,gap=.05,runtime=180;
  // Explicit gravity feedforward coefficients; zero disables it entirely.
  double gravity_alpha=0,gravity_ramp=1;
  bool gravity_float_enabled=false;
  void validate(std::size_t axis_count=0) const;
};
struct PositionBounds {double lower,upper;};

// Parses the optional list form a generated URDF passes as a hardware
// parameter: "a b c d e", "a,b,c,d,e" or "[a, b, c, d, e]". Empty text means
// "not supplied" and returns an empty vector. Every token must be a complete
// finite number; malformed text (including empty elements) is refused rather
// than partially accepted. Length/range/count policy is enforced by
// MotionPolicy::validate, not here.
std::vector<double> parse_gain_axis_parameter(const std::string& text,const char* name);
// Parses the optional per-axis angle schedule parameter:
//   "0:160,20:220,40:280;0:160,20:240,40:280;..."
// Semicolon separates axes (J1..Jn order), comma separates points, and each
// point is angle_deg:kp. Empty text disables scheduling. The parser rejects
// malformed/empty elements, non-finite values, duplicate angles and unsorted
// points; length and range policy is checked by MotionPolicy::validate.
std::vector<std::vector<KpSchedulePoint>> parse_kp_schedule_parameter(
    const std::string& text,const char* name);
// Linear interpolation with endpoint clamping. Pure function so the bounds and
// interpolation contract can be unit-tested without a supervisor or hardware.
double interpolate_kp_schedule(const std::vector<KpSchedulePoint>& points,
                               double angle_deg);
// One-cycle rate-limited transition. Non-finite or non-positive inputs throw.
double slew_kp_gain(double current,double target,double dt,double max_rate);

// Frozen driver first-cause evidence. It is captured exactly once, before the
// STOPPING transition can overwrite the observation that caused the stop, so a
// later OPERATOR_GLOBAL_STOP or downstream JTC error cannot replace it. All
// values are the last complete active cycle / last complete MIT write batch,
// never a post-stop current-output claim.
struct FaultSnapshot {
  bool valid=false;
  // 1..axis_count for an axis-specific fault; 0 when the fault is not axis
  // specific (stale/conflicted feedback, cycle gap, dimension errors, ...).
  int axis=0;
  std::string joint;
  std::string code;
  double detected_at=0;
  // NaN when not applicable/available; JSON renders non-finite as null.
  double error_rad=0,limit_rad=0,target_rad=0,actual_rad=0;
  double feedback_age=0;
  bool feedback_valid=false;
  // steady_clock seconds of the last controls() computation and of the last
  // MIT write batch that wrote every active axis; -1 when none happened yet.
  double request_estimate_at=-1,last_complete_write_at=-1;
  std::vector<double> encoded_kp,encoded_kd,encoded_tau_ff,request_estimate;
  std::uint64_t command_id=0;
};

// Supervision only. The drive runs PD via the official MIT codec; trajectories
// come from the standard JTC. No gravity model, host PD, or auto gain increase.
class MotionSupervisor {
 public:
  MotionSupervisor(MotionPolicy policy,const std::vector<PositionBounds>& bounds,
                   std::shared_ptr<const GravityFfModel> gravity_model=nullptr);
  // Source compatibility for the established four-axis callers and tests.
  template <std::size_t N>
  MotionSupervisor(MotionPolicy policy,const std::array<PositionBounds,N>& bounds,
                   std::shared_ptr<const GravityFfModel> gravity_model=nullptr)
      :MotionSupervisor(std::move(policy),std::vector<PositionBounds>(bounds.begin(),bounds.end()),
                        std::move(gravity_model)){
    static_assert(N==4||N==5,"Motion supervision supports four or five active axes");
  }
  bool arm(double at,bool gravity_float=false,double float_kd=5.,double float_seconds=5.);
  bool reset(double at,const slaihand_transport::ReadOnlyState& state);
  void stop(const char* reason,double at,bool explicit_stop=false);
  // In-session gravity-float -> position-hold switch (and back). Both requests
  // are bound to the current command_id; the actual latch happens in tick().
  bool request_gravity_mode(bool hold,std::uint64_t command_id,double at);
  Batch tick(double at,slaihand_transport::ReadOnlyState& state,
             const std::vector<double>& reference,bool owned);
  template <std::size_t N>
  Batch tick(double at,slaihand_transport::ReadOnlyState& state,
             const std::array<double,N>& reference,bool owned){
    return tick(at,state,std::vector<double>(reference.begin(),reference.end()),owned);
  }
  void submitted(const Action& action,double at);
  void written(const Action& action,double at);
  MotionStage stage() const{return stage_;}
  std::size_t axis_count() const{return bounds_.size();}
  bool hold_ready() const{return hold_ready_;}
  bool attempted() const{return attempted_;}
  bool disabled_observed() const{return stage_==MotionStage::DISABLED&&stop_confirmed_;}
  const char* reason() const{return reason_;}
  const std::vector<double>& anchor()const{return anchor_;}
  const std::vector<double>& target()const{return target_;}
  const std::vector<double>& request_estimate()const{return request_estimate_;}
  std::uint64_t command_id()const{return command_id_;}
  bool gravity_active()const{return ff_active_;}
  double gravity_alpha()const{return gravity_model_?((floating_||gravity_hold_)?1.:p_.gravity_alpha):0.;}
  double gravity_ramp()const{return (floating_||gravity_hold_)?3.:p_.gravity_ramp;}
  bool gravity_float()const{return floating_;}
  bool gravity_hold()const{return gravity_hold_;}
  GravityLockState gravity_lock_state()const{return lock_state_;}
  const std::vector<double>& hold_target()const{return hold_target_;}
  const std::vector<double>& hold_error()const{return hold_error_;}
  double hold_gain_scale()const{return hold_gain_scale_;}
  double remaining_s(double at)const{return std::max(0.,float_seconds_-(at-started_));}
  const char* lock_reason()const{return lock_reason_.data();}
  int lock_reason_code()const{return lock_code_;}
  double float_kd()const{return float_kd_;}
  double float_seconds()const{return float_seconds_;}
  const std::vector<double>& ff_requested()const{return ff_requested_;}
  const std::vector<double>& ff_candidate()const{return ff_candidate_;}
  const std::vector<double>& ff_encoded()const{return ff_encoded_;}
  const std::vector<double>& ff_applied()const{return ff_applied_;}
  const std::vector<double>& ff_last_written()const{return ff_last_written_;}
  const std::vector<double>& ff_written_at()const{return ff_written_at_;}
  // Effective per-axis position gains actually used by the supervisor. For a
  // legacy scalar policy every entry equals the scalar. This is the steady
  // (full-ramp) value; the per-cycle gain ramp scales it monotonically.
  const std::vector<double>& kp_axis()const{return kp_axis_;}
  const std::vector<double>& kd_axis()const{return kd_axis_;}
  bool kp_schedule_active()const{return kp_schedule_active_;}
  const std::vector<double>& kp_effective_axis()const{return kp_effective_;}
  const std::vector<double>& kp_schedule_target_axis()const{return kp_schedule_target_;}
  const std::vector<int>& kp_schedule_segment_axis()const{return kp_schedule_segment_;}
  // Post-ramp Kp used to build the latest MIT packet, before protocol quantization.
  const std::vector<double>& last_scaled_kp_axis()const{return last_scaled_kp_;}
  double last_scaled_at()const{return last_scaled_at_;}
  // Quantized Kp decoded from the latest packet that reached the serial write path.
  const std::vector<double>& last_mit_kp_axis()const{return last_mit_kp_;}
  const std::vector<double>& last_mit_write_at_axis()const{return last_mit_write_at_;}
  // Static configuration fragment for runtime identity/session metadata. It
  // contains configured gains and schedule, not dynamic effective values.
  // Starts with a comma so callers can append it inside a JSON object.
  std::string gain_axis_json()const;
  // Frozen first unsafe observation; not replaced by later stops.
  bool has_first_fault()const{return first_fault_.valid;}
  const FaultSnapshot& first_fault()const{return first_fault_;}
  // Frozen first-cause JSON object, or the literal "null" when none was
  // captured. boot_id/cycle_id are supplied by the driver runtime, which owns
  // them; they do not change the frozen snapshot itself.
  std::string first_fault_json(const std::string& boot_id,std::uint64_t cycle_id)const;
  // "UNAVAILABLE" before any controls() output, "CURRENT" only while the
  // supervisor is still producing output, and "LAST_ACTIVE_STALE" after the
  // session stopped. request_estimate() values are retained but must be read as
  // the last active estimate, never as a current powered output.
  const char* request_estimate_state()const;
  double request_estimate_at()const{return request_estimate_at_;}
 private:
  Batch stopping(double at,const slaihand_transport::ReadOnlyState& state);
  void check_observations(double at,const slaihand_transport::ReadOnlyState& state,bool initial);
  // First-cause evidence capture. A second call is a no-op: the first unsafe
  // observation wins until a new session starts.
  void capture_fault(double at,const slaihand_transport::ReadOnlyState& state,int axis,
                     const char* code,double error_rad,double limit_rad);
  void clear_first_fault();
  void apply_lock_request(double at,const slaihand_transport::ReadOnlyState& state);
  void latch_gravity_hold(double at,const slaihand_transport::ReadOnlyState& state);
  void update_kp_schedule(double at,const slaihand_transport::ReadOnlyState& state);
  void reset_kp_schedule_state();
  Batch controls(double at,const slaihand_transport::ReadOnlyState& state,int enable_axis=0);
  MotionPolicy p_;
  std::vector<PositionBounds> bounds_;
  std::shared_ptr<const GravityFfModel> gravity_model_;
  // requested: model request; candidate: after guard-headroom saturation;
  // encoded: final packet field; applied: last successfully written packet.
  std::vector<double> ff_requested_,ff_candidate_,ff_encoded_,ff_applied_;
  std::vector<double> ff_last_written_,ff_written_at_;
  bool ff_active_=false;
  bool floating_=false;
  bool gravity_hold_=false;
  GravityLockState lock_state_=GravityLockState::NONE;
  bool lock_pending_=false,release_pending_=false;
  std::uint64_t lock_command_id_=0;
  std::vector<double> hold_target_,hold_error_;
  double hold_started_=0,release_started_=0,hold_gain_scale_=0,release_gain_start_=0;
  int lock_code_=0;
  std::array<char,96> lock_reason_{};
  double float_kd_=5.,float_seconds_=5.;
  std::vector<double> request_estimate_;
  // Effective per-axis gains (J1..Jn) after scalar fallback. The booleans keep
  // the configured source, not the scalar-equal values, for identity output.
  std::vector<double> kp_axis_,kd_axis_;
  bool kp_axis_per_axis_=false,kd_axis_per_axis_=false;
  // Angle-scheduled Kp state. The effective vector is what is actually encoded;
  // the target and segment are retained for audit and diagnostics.
  bool kp_schedule_active_=false,kp_schedule_initialized_=false;
  std::vector<double> kp_effective_,kp_schedule_target_;
  std::vector<int> kp_schedule_segment_;
  double kp_schedule_last_at_=-1;
  std::vector<double> last_scaled_kp_;
  double last_scaled_at_=-1;
  // Decoded fields of the last MIT packets that reached the write path, and the
  // times they did. min(last_mit_write_at_) is the last batch that covered all
  // axes; it is audit evidence, not a drive execution acknowledgement.
  std::vector<double> last_mit_kp_,last_mit_kd_,last_mit_tau_,last_mit_write_at_;
  double request_estimate_at_=-1;
  FaultSnapshot first_fault_;
  void clear_feedforward(bool keep_history);
  MotionStage stage_=MotionStage::DISARMED;
  const char* reason_="NOT_ARMED";
  std::array<char,128> reason_buffer_{};
  std::vector<double> anchor_,target_,enable_at_,confirmed_at_,fd_at_,stop_seen_;
  std::vector<unsigned> stop_samples_;
  double started_=0,last_tick_=0,preload_at_=0,stable_at_=-1,stop_at_=0;
  unsigned preload_mask_=0;int next_enable_=0;
  bool attempted_=false,hold_ready_=false,stop_confirmed_=false;
  std::uint64_t command_id_=0;
};
}  // namespace slaihand_hardware
