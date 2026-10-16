"""Decision logic for safety_controller.

No ROS imports, so it can be unit tested with plain pytest.
"""
import math
from dataclasses import dataclass
from typing import NamedTuple

CLEAR = 'CLEAR'
SLOWING = 'SLOWING'
BLOCKED = 'BLOCKED'
TIMEOUT = 'TIMEOUT'


@dataclass(frozen=True)
class SafetyParams:
    stop_distance: float = 0.5     # m, block forward motion below this
    slow_distance: float = 1.0     # m, start slowing down below this
    front_angle_deg: float = 30.0  # look +/- this many degrees ahead
    cmd_timeout: float = 0.3       # s without teleop before we stop
    scan_timeout: float = 0.5      # s without a scan before forward is blocked
    creep_speed: float = 0.05      # m/s, slowest speed while slowing
    hysteresis: float = 0.05       # m, extra room needed to leave a state

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


def front_min_range(ranges, angle_min, angle_increment,
                    range_min, range_max, half_angle):
    """Closest valid reading within +/-half_angle of straight ahead, or None."""
    closest = None
    for i, r in enumerate(ranges):
        # skip inf, nan and out-of-spec readings
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))  # wrap to -pi..pi
        if abs(angle) <= half_angle and (closest is None or r < closest):
            closest = r
    return closest


def is_fresh(age, timeout):
    # None = never received, negative = clock jumped back
    return age is not None and 0.0 <= age <= timeout


def classify(front_range, prev_state, p):
    """CLEAR, SLOWING or BLOCKED based on the closest thing ahead."""
    if front_range is None:
        return CLEAR
    stop = p.stop_distance
    slow = p.slow_distance
    # hysteresis: leaving a state needs a bit more room than entering it
    if prev_state == BLOCKED:
        stop += p.hysteresis
    if prev_state in (BLOCKED, SLOWING):
        slow += p.hysteresis
    if front_range < stop:
        return BLOCKED
    if front_range < slow:
        return SLOWING
    return CLEAR


def decide(linear, angular, front_range, cmd_age, scan_age, prev_state, p):
    """Filter one teleop command.

    cmd_age / scan_age: seconds since the last message, None if none yet.
    Only forward motion is limited, so the operator can always turn or reverse.
    """
    # 1. teleop went quiet: stop everything
    if not is_fresh(cmd_age, p.cmd_timeout):
        return Decision(0.0, 0.0, TIMEOUT, 'no recent teleop command, stopping')

    # 2. work out the state
    if not is_fresh(scan_age, p.scan_timeout):
        state = BLOCKED
        reason = 'no recent lidar scan, forward blocked'
    else:
        state = classify(front_range, prev_state, p)
        if front_range is None:
            reason = 'nothing ahead'
        else:
            reason = f'closest obstacle ahead {front_range:.2f} m'

    # 3. limit forward speed only
    if linear > 0.0:
        if state == BLOCKED:
            linear = 0.0
        elif state == SLOWING:
            # 0 at stop_distance, 1 at slow_distance
            scale = (front_range - p.stop_distance) / (p.slow_distance - p.stop_distance)
            scale = min(max(scale, 0.0), 1.0)
            # creep floor, or we'd never actually reach the stop line
            linear = min(linear, max(linear * scale, p.creep_speed))

    return Decision(linear, angular, state, reason)
