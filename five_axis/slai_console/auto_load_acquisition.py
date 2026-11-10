"""Finite automatic load-data acquisition plans; NO hardware adapter.

Reuse the wrist's finite route, bidirectional samples, checkpoints and failure
semantics. Robot-specific torque handling is deliberately not borrowed from
the cable wrist. Requires measured mechanical limits; URDF is only a check.
"""
from dataclasses import asdict,dataclass
import hashlib
import json
import math

from .core import LIMITS


@dataclass(frozen=True)
class ScanPoint:
    index: int
    target_pose_rad: tuple
    duration_s: float
    approach: str
    collect: bool


@dataclass(frozen=True)
class AutoLoadPlan:
    joint: int
    initial_pose_rad: tuple
    mechanical_limits_deg: tuple
    scan_limits_deg: tuple
    mapping_sha256: str
    physical_limit_evidence: str
    points: tuple
    reference_speed_rad_s: float
    settle_budget_s: float
    total_budget_s: float
    measured_limit_override: bool = False
    arrival_settle_s: float = 8.
    bias_cap_nm: float = 4.
    takeover_settle_s: float = 30.

    def serializable(self):
        return dict(asdict(self),status='PLAN_ONLY_NOT_HARDWARE_VALIDATED',
            hardware_motion_authorized=False,simulated_mass_used=False,
            automatic_resume=False,all_arm_calibration_complete=False)

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.serializable(),sort_keys=True,
            allow_nan=False).encode('utf-8')).hexdigest()


def build_plan(*,joint,initial_pose_rad,mechanical_limits_deg,
               mapping_sha256,physical_limit_evidence,physical_limits_confirmed,
               step_deg=5.,speed_rad_s=.02,margin_deg=10.,settle_budget_s=30.,
               max_budget_s=3600.,measured_limit_override=False,scan_limits_deg=None):
    if type(joint) is not int or joint not in (1,2,3,4):
        raise ValueError('Only J1-J4, one selected axis per batch')
    if physical_limits_confirmed is not True or not physical_limit_evidence.strip():
        raise ValueError('Measured physical limit values and provenance required')
    pose=tuple(initial_pose_rad);limits=tuple(mechanical_limits_deg)
    if len(pose)!=4 or len(limits)!=2:
        raise ValueError('Need four measured angles and two actual mechanical limits')
    if not all(math.isfinite(v) for v in (*pose,*limits,step_deg,speed_rad_s,margin_deg,settle_budget_s,max_budget_s)):
        raise ValueError('Nonfinite automatic acquisition settings')
    if not 1<=step_deg<=10 or not 0<speed_rad_s<=.02 or margin_deg<10 or not 2<=settle_budget_s<=60:
        raise ValueError('Automatic scan step/speed/margin/settle outside current policy')
    if not 0<max_budget_s<=3600 or len(mapping_sha256)!=64 or any(c not in '0123456789abcdef' for c in mapping_sha256):
        raise ValueError('Finite campaign budget and mapping digest required')
    low,high=limits[0]+margin_deg,limits[1]-margin_deg
    if scan_limits_deg is not None:
        requested=tuple(scan_limits_deg)
        if len(requested)!=2 or not all(math.isfinite(v) for v in requested):
            raise ValueError('Two finite requested scan bounds required')
        if requested[0]<low or requested[1]>high:
            raise ValueError('Requested scan exceeds inward mechanical margins')
        low,high=requested
    if low>=high:raise ValueError('No usable physical range after inward margins')
    if measured_limit_override is not False:
        raise ValueError('Limit override retired; reconcile canonical model coordinates before scanning')
    if low<LIMITS[joint-1][0] or high>LIMITS[joint-1][1]:
        raise ValueError('Requested measured scan range exceeds current model limits; review frames first')
    for i,q in enumerate(pose):
        if not LIMITS[i][0]<=math.degrees(q)<=LIMITS[i][1]:
            raise ValueError('Initial measured pose outside current model bounds')
    start=math.degrees(pose[joint-1])
    if not low<=start<=high:
        raise ValueError('Initial pose outside inward-margin range; no automatic recovery move')
    # Keep arrival tolerance/quantization away from the boundary itself.
    if high-low<=1.:raise ValueError('Scan too narrow for endpoint arrival margins')
    target_low,target_high=low+.5,high-.5
    levels=[target_low]
    while levels[-1]+step_deg<target_high-1e-10:levels.append(levels[-1]+step_deg)
    levels.append(target_high)
    if len(levels)>80:raise ValueError('Too many scan levels')
    # Approach lower endpoint once, ascend, descend, return to original pose.
    # Interior levels get both approach directions. Endpoints are not falsely
    # labelled paired: reversal has zero approach until actual movement occurs.
    # Approach the first endpoint in <=step-sized legs, rather than one large
    # sweep from the current pose. These transit points do not count as samples.
    transit=[];at=start
    while at-target_low>step_deg+1e-10:
        at-=step_deg;transit.append((at,False))
    route=transit+[(target_low,True)]+[(x,True) for x in levels[1:]]+[(x,True) for x in reversed(levels[:-1])]
    at=target_low
    while start-at>step_deg+1e-10:
        at+=step_deg;route.append((at,False))
    route.append((start,False))
    points=[];previous=start;budget=31.
    for index,(target,collect) in enumerate(route):
        delta=target-previous
        approach='positive' if delta>1e-10 else 'negative' if delta<-1e-10 else 'unknown'
        # The preview/start freshness check allows <=0.5deg initial mismatch.
        # Budget that mismatch in the first leg's peak reference speed.
        reserve=.5 if index==0 else 0.
        duration=max(1.,1.875*math.radians(abs(delta)+reserve)/speed_rad_s)
        q=list(pose);q[joint-1]=math.radians(target)
        points.append(ScanPoint(index,tuple(q),duration,approach,collect))
        budget+=duration+8.+(settle_budget_s if collect else 2.)
        previous=target
    if budget>max_budget_s:raise ValueError('Plan exceeds whole-campaign time budget; split into reviewed batches')
    return AutoLoadPlan(joint,pose,limits,(low,high),mapping_sha256,
        physical_limit_evidence,tuple(points),speed_rad_s,settle_budget_s,budget,measured_limit_override,
        8.,4. if joint==1 else 2.)


class AcquisitionLedger:
    """Exactly one outstanding move/sample; a halt never auto-resumes.

    Actions are plan data only, not transport capability. A future live adapter
    must independently bind its enable/STOP authorization to this fingerprint.
    """
    def __init__(self,plan):
        if not isinstance(plan,AutoLoadPlan):raise ValueError('Explicit finite plan required')
        self.plan=plan;self.index=0;self.phase='READY';self.samples=[];self.reason=None

    def next_move(self,mapping_sha256):
        if mapping_sha256!=self.plan.mapping_sha256:
            self.halt('mapping_changed');raise ValueError('Zero/sign changed; new batch required')
        if self.phase!='READY':raise ValueError('Cannot issue move in current acquisition phase')
        if self.index>=len(self.plan.points):
            self.phase='AWAITING_STOP';return None
        point=self.plan.points[self.index];self.phase='MOVING'
        return point

    def arrived(self,*,fresh,within_tolerance):
        if self.phase!='MOVING':raise ValueError('No outstanding move')
        if fresh is not True or within_tolerance is not True:
            self.halt('arrival_not_verified');raise ValueError('Fresh actual arrival required')
        if self.plan.points[self.index].collect:self.phase='SAMPLING'
        else:self.index+=1;self.phase='READY'

    def record(self,point):
        if self.phase!='SAMPLING':raise ValueError('No outstanding stable sample')
        if (point.get('accepted') is not True or point.get('joint')!=self.plan.joint or
            point.get('mapping_sha256')!=self.plan.mapping_sha256):
            self.halt('sample_not_qualified');raise ValueError('Stable selected-axis sample required')
        actual=point.get('pose_rad',())
        target=self.plan.points[self.index].target_pose_rad
        if len(actual)!=4 or not all(math.isfinite(q) for q in actual):
            self.halt('invalid_actual_pose');raise ValueError('Full measured pose required')
        if any(abs(q-t)>math.radians(.5) for q,t in zip(actual,target)):
            self.halt('actual_pose_not_at_waypoint');raise ValueError('Measured pose outside waypoint tolerance')
        if not self.plan.scan_limits_deg[0]<=math.degrees(actual[self.plan.joint-1])<=self.plan.scan_limits_deg[1]:
            self.halt('actual_pose_outside_inward_margin');raise ValueError('Measured pose outside inward-margin boundary')
        self.samples.append(dict(point,waypoint_index=self.index,
            approach=self.plan.points[self.index].approach,plan_sha256=self.plan.fingerprint))
        self.index+=1;self.phase='READY'

    def confirm_stop(self,disabled):
        if self.phase!='AWAITING_STOP':raise ValueError('Batch has not completed its finite route')
        if disabled is not True:
            self.halt('stop_unconfirmed');return
        self.phase='COMPLETE'

    def halt(self,reason):
        self.phase='HALTED';self.reason=str(reason)

    def checkpoint(self):
        return dict(plan_sha256=self.plan.fingerprint,mapping_sha256=self.plan.mapping_sha256,
            phase=self.phase,next_index=self.index,samples=self.samples,
            reason=self.reason,automatic_resume=False,
            full_range_gravity_model_validated=False,hardware_motion_authorized=False)
