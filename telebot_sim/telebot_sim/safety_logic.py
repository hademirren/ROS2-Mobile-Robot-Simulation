"""Decision logic for safety_controller.

Kept free of ROS imports so it can be unit tested with plain pytest.
"""
import math
from dataclasses import dataclass
from typing import NamedTuple, Optional, Sequence

CLEAR = 'CLEAR'
SLOWING = 'SLOWING'
BLOCKED = 'BLOCKED'
TIMEOUT = 'TIMEOUT'


@dataclass(frozen=True)
class SafetyParams:
    stop_distance: float = 0.5     # m, forward motion blocked below this
    slow_distance: float = 1.0     # m, forward speed scaled down below this
    front_angle_deg: float = 30.0  # half-width of the front sector
    cmd_timeout: float = 0.3       # s without teleop before we stop
    scan_timeout: float = 0.5      # s without a scan before forward is blocked
    # without a floor the robot only approaches stop_distance asymptotically
    # and never actually reaches BLOCKED
    creep_speed: float = 0.05      # m/s
    # margin needed to relax a state, so readings near a threshold don't flicker
    hysteresis: float = 0.05       # m

    def __post_init__(self):
        if not 0.0 < self.stop_distance < self.slow_distance:
            raise ValueError('need 0 < stop_distance < slow_distance')
        if not 0.0 < self.front_angle_deg <= 180.0:
            raise ValueError('front_angle_deg must be in (0, 180]')
        if self.cmd_timeout <= 0.0 or self.scan_timeout <= 0.0:
            raise ValueError('timeouts must be positive')
        if self.creep_speed < 0.0 or self.hysteresis < 0.0:
            raise ValueError('creep_speed and hysteresis must be >= 0')


class Decision(NamedTuple):
    linear: float
    angular: float
    state: str
    reason: str


def front_min_range(ranges: Sequence[float], angle_min: float,
                    angle_increment: float, range_min: float,
                    range_max: float, half_angle: float) -> Optional[float]:
    """Closest valid range within +/-half_angle (rad) of straight ahead.

    Returns None if nothing valid is in the sector.
    """
    best = None
    for i, r in enumerate(ranges):
        # inf/nan or out-of-spec returns say nothing about distance
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        # wrap so 0..2pi scanners work the same as -pi..pi ones
        a = math.atan2(math.sin(a), math.cos(a))
        if abs(a) <= half_angle and (best is None or r < best):
            best = r
    return best


def _fresh(age: Optional[float], timeout: float) -> bool:
    # negative age means the clock jumped back (sim reset); don't trust it
    return age is not None and 0.0 <= age <= timeout


def classify(front_range: Optional[float], prev_state: Optional[str],
             p: SafetyParams) -> str:
    if front_range is None:
        return CLEAR
    stop = p.stop_distance
    slow = p.slow_distance
    if prev_state == BLOCKED:
        stop += p.hysteresis
    if prev_state in (BLOCKED, SLOWING):
        slow += p.hysteresis
    if front_range < stop:
        return BLOCKED
    if front_range < slow:
        return SLOWING
    return CLEAR


def decide(linear: float, angular: float, front_range: Optional[float],
           cmd_age: Optional[float], scan_age: Optional[float],
           prev_state: Optional[str], p: SafetyParams) -> Decision:
    """Filter one teleop command.

    Ages are seconds since the last message, or None if none arrived yet.
    Only forward motion is ever limited; turning and reversing pass through
    so the operator can always steer away from an obstacle.
    """
    if not _fresh(cmd_age, p.cmd_timeout):
        # replaying a stale command is how a dropped link drives into a wall
        if cmd_age is None:
            reason = 'no teleop command received yet, holding zero velocity'
        else:
            reason = f'no teleop command for {cmd_age:.2f} s, holding zero velocity'
        return Decision(0.0, 0.0, TIMEOUT, reason)

    if not _fresh(scan_age, p.scan_timeout):
        # can't see ahead, so don't drive ahead
        state = BLOCKED
        if scan_age is None:
            reason = 'no scan received yet, forward motion blocked'
        else:
            reason = f'no scan for {scan_age:.2f} s, forward motion blocked'
    else:
        state = classify(front_range, prev_state, p)
        if state == BLOCKED:
            reason = (f'obstacle {front_range:.2f} m ahead, forward motion '
                      'blocked (turn/reverse allowed)')
        elif state == SLOWING:
            reason = f'obstacle {front_range:.2f} m ahead, limiting forward speed'
        else:
            reason = f'nothing within {p.slow_distance:.2f} m ahead'

    out = linear
    if linear > 0.0:
        if state == BLOCKED:
            out = 0.0
        elif state == SLOWING:
            span = p.slow_distance - p.stop_distance
            scale = min(max((front_range - p.stop_distance) / span, 0.0), 1.0)
            out = min(linear, max(linear * scale, p.creep_speed))
    return Decision(out, angular, state, reason)
