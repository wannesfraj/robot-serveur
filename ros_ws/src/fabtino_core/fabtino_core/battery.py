"""Battery model, trajectory energy estimate and accept/refuse decision.

ROS-independent. Units: metres, radians, seconds, watt-hours, percent.

The coefficients below are PLACEHOLDERS: calibrate ``wh_per_m``, ``wh_per_rad``
and ``idle_w`` on the real robot (log battery voltage/current over a known
distance) before trusting the thresholds.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class BatteryConfig:
    capacity_wh: float = 480.0        # e.g. 24 V x 20 Ah
    wh_per_m: float = 0.10            # drive energy per metre travelled
    wh_per_rad: float = 0.02          # extra energy per radian of rotation
    idle_w: float = 20.0              # electronics (lidar, compute) while moving or not
    avg_speed_mps: float = 0.20       # used to turn path length into trip time
    reserve_pct: float = 15.0         # refuse a trajectory that would end below this
    warning_pct: float = 20.0         # warn when an accepted route ends below this
    safety_factor: float = 1.25       # margin on the estimated cost
    resample_m: float = 1.00          # turns are measured on chords this long (hides A* grid staircase)
    turn_deadband_rad: float = 0.25   # heading changes below this are not counted as turns

    @classmethod
    def from_dict(cls, data):
        known = cls.__dataclass_fields__
        return cls(**{k: float(v) for k, v in dict(data).items() if k in known})

    def to_dict(self):
        return asdict(self)


def path_length(points):
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


def _resample(points, step):
    """Keep points at least ``step`` apart so 8-connected grid zig-zag is not counted as turning."""
    if len(points) < 3:
        return list(points)
    kept = [points[0]]
    for point in points[1:-1]:
        if math.hypot(point[0]-kept[-1][0], point[1]-kept[-1][1]) >= step:
            kept.append(point)
    kept.append(points[-1])
    return kept


def path_turning(points, step=1.0, deadband=0.25):
    pts = _resample(points, step)
    total = 0.0
    for a, b, c in zip(pts, pts[1:], pts[2:]):
        h1 = math.atan2(b[1]-a[1], b[0]-a[0])
        h2 = math.atan2(c[1]-b[1], c[0]-b[0])
        turn = abs(math.atan2(math.sin(h2-h1), math.cos(h2-h1)))
        if turn > deadband:
            total += turn
    return total


def estimate_path_cost(points, config: BatteryConfig, initial_turn_rad=0.0, idle_duration_s=0.0):
    """Energy needed to drive ``points`` (list of (x, y)). Returns a dict."""
    distance = path_length(points)
    turning = path_turning(points, config.resample_m, config.turn_deadband_rad) + abs(initial_turn_rad)
    duration = distance/max(1e-6, config.avg_speed_mps)
    idle_duration_s = max(0.0, float(idle_duration_s))
    energy = (config.wh_per_m*distance + config.wh_per_rad*turning
              + config.idle_w*(duration+idle_duration_s)/3600.0)
    return dict(distance_m=distance, turn_rad=turning, duration_s=duration+idle_duration_s,
                wait_s=idle_duration_s,
                energy_wh=energy, percent=100.0*energy/config.capacity_wh)


def evaluate_trip(points, soc_pct, config: BatteryConfig, initial_turn_rad=0.0,
                  idle_duration_s=0.0):
    """Accept the trajectory only if the battery stays above the reserve.

    ``required_pct`` already includes the safety factor.
    """
    cost = estimate_path_cost(points, config, initial_turn_rad, idle_duration_s)
    required = cost['percent']*config.safety_factor
    after = soc_pct-required
    accepted = after >= config.reserve_pct
    warning = accepted and after <= config.warning_pct
    reason = '' if accepted else (
        f"Battery too low for this trajectory: {soc_pct:.1f}% available, "
        f"{required:.1f}% needed ({cost['distance_m']:.1f} m), "
        f"would end at {after:.1f}% < reserve {config.reserve_pct:.0f}%")
    warning_reason = (f"Low battery warning: the route uses {cost['distance_m']:.1f} m "
                      f"and is expected to end at {after:.1f}%; recharge after the mission"
                      if warning else '')
    return dict(accepted=accepted, reason=reason, soc_pct=soc_pct, required_pct=required,
                soc_after_pct=after, reserve_pct=config.reserve_pct,
                warning=warning, warning_reason=warning_reason, **cost)


class BatteryState:
    """Integrates consumption from measured velocity (not from pose jumps)."""

    def __init__(self, config: BatteryConfig, initial_soc_pct=100.0, consumption_scale=1.0):
        self.config = config
        self.energy_wh = config.capacity_wh*max(0.0, min(100.0, initial_soc_pct))/100.0
        self.scale = float(consumption_scale)
        self.last_time = None

    @property
    def soc_pct(self):
        return 100.0*self.energy_wh/self.config.capacity_wh

    @property
    def low(self):
        return self.soc_pct < self.config.reserve_pct

    def set_soc(self, soc_pct):
        self.energy_wh = self.config.capacity_wh*max(0.0, min(100.0, float(soc_pct)))/100.0

    def update(self, now, linear_mps, angular_rps):
        if self.last_time is None:
            self.last_time = now
            return self.soc_pct
        dt = max(0.0, min(0.5, now-self.last_time))   # ignore long gaps / clock jumps
        self.last_time = now
        used = (self.config.wh_per_m*abs(linear_mps)*dt
                + self.config.wh_per_rad*abs(angular_rps)*dt
                + self.config.idle_w*dt/3600.0)*self.scale
        self.energy_wh = max(0.0, self.energy_wh-used)
        return self.soc_pct
