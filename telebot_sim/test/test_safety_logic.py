"""Unit tests for safety_logic. No ROS needed: pytest test/"""
import math
import os
import sys

import pytest

# run straight from the source tree without a colcon build
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from telebot_sim.safety_logic import (  # noqa: E402
    BLOCKED, CLEAR, SLOWING, TIMEOUT, SafetyParams, decide, front_min_range)

P = SafetyParams()  # stop 0.5, slow 1.0, +/-30 deg, timeout 0.3
FRESH = 0.05        # cmd/scan age well inside the timeouts
HALF = math.radians(30.0)


def run(linear, angular, front, cmd_age=FRESH, scan_age=FRESH, prev=None, p=P):
    return decide(linear, angular, front, cmd_age, scan_age, prev, p)


def scan(ranges, angle_min=-math.pi, range_min=0.12, range_max=8.0):
    # evenly spaced over a full turn, one reading per degree like the sim lidar
    inc = 2.0 * math.pi / len(ranges)
    return front_min_range(ranges, angle_min, inc, range_min, range_max, HALF)


def ring(n=360, default=float('inf')):
    return [default] * n


def idx(deg, n=360):
    # index of the reading at `deg` for a scan starting at -pi
    return int(round((math.radians(deg) + math.pi) / (2.0 * math.pi / n))) % n


# ---- command filtering ----

def test_clear_path_passes_command_through():
    d = run(0.5, 0.3, front=3.0)
    assert d.state == CLEAR
    assert d.linear == pytest.approx(0.5)
    assert d.angular == pytest.approx(0.3)


def test_nothing_in_sector_is_clear():
    d = run(0.5, 0.0, front=None)
    assert d.state == CLEAR
    assert d.linear == pytest.approx(0.5)


def test_slowing_scales_forward_linearly():
    # 0.75 m is halfway between stop (0.5) and slow (1.0)
    d = run(0.6, 0.2, front=0.75)
    assert d.state == SLOWING
    assert d.linear == pytest.approx(0.3)
    assert d.angular == pytest.approx(0.2)


def test_slowing_closer_means_slower():
    far = run(0.8, 0.0, front=0.9).linear
    near = run(0.8, 0.0, front=0.6).linear
    assert 0.0 < near < far < 0.8


def test_slowing_creep_floor_near_stop_line():
    d = run(0.8, 0.0, front=0.51)
    assert d.state == SLOWING
    assert d.linear == pytest.approx(P.creep_speed)


def test_slowing_never_speeds_up_a_slow_command():
    d = run(0.02, 0.0, front=0.51)
    assert d.linear == pytest.approx(0.02)


def test_blocked_stops_forward():
    d = run(0.5, 0.0, front=0.4)
    assert d.state == BLOCKED
    assert d.linear == 0.0


def test_blocked_allows_reverse():
    d = run(-0.3, 0.0, front=0.3)
    assert d.state == BLOCKED
    assert d.linear == pytest.approx(-0.3)


def test_blocked_allows_turning():
    d = run(0.5, 1.2, front=0.3)
    assert d.state == BLOCKED
    assert d.linear == 0.0
    assert d.angular == pytest.approx(1.2)


def test_reverse_not_scaled_while_slowing():
    d = run(-0.4, 0.0, front=0.6)
    assert d.state == SLOWING
    assert d.linear == pytest.approx(-0.4)


# ---- watchdog ----

def test_timeout_zeroes_everything():
    d = run(0.5, 0.5, front=5.0, cmd_age=0.31)
    assert d.state == TIMEOUT
    assert (d.linear, d.angular) == (0.0, 0.0)


def test_no_command_yet_is_timeout():
    d = run(0.5, 0.5, front=5.0, cmd_age=None)
    assert d.state == TIMEOUT
    assert (d.linear, d.angular) == (0.0, 0.0)


def test_command_inside_timeout_is_not_timeout():
    assert run(0.5, 0.0, front=5.0, cmd_age=0.29).state == CLEAR


def test_timeout_beats_reverse_and_turn():
    d = run(-0.5, 1.0, front=0.3, cmd_age=1.0)
    assert d.state == TIMEOUT
    assert (d.linear, d.angular) == (0.0, 0.0)


def test_clock_jump_back_is_treated_as_stale():
    assert run(0.5, 0.0, front=5.0, cmd_age=-1.0).state == TIMEOUT


# ---- lost lidar ----

def test_no_scan_blocks_forward_but_allows_reverse_and_turn():
    fwd = run(0.5, 0.4, front=None, scan_age=None)
    assert fwd.state == BLOCKED
    assert fwd.linear == 0.0
    assert fwd.angular == pytest.approx(0.4)
    assert run(-0.2, 0.0, front=None, scan_age=None).linear == pytest.approx(-0.2)


def test_stale_scan_blocks_forward():
    d = run(0.5, 0.0, front=5.0, scan_age=0.8)
    assert d.state == BLOCKED
    assert d.linear == 0.0


# ---- hysteresis ----

def test_blocked_holds_until_past_hysteresis():
    assert run(0.5, 0.0, front=0.52, prev=BLOCKED).state == BLOCKED
    assert run(0.5, 0.0, front=0.56, prev=BLOCKED).state == SLOWING


def test_slowing_holds_until_past_hysteresis():
    assert run(0.5, 0.0, front=1.02, prev=SLOWING).state == SLOWING
    assert run(0.5, 0.0, front=1.06, prev=SLOWING).state == CLEAR


def test_no_hysteresis_coming_out_of_timeout():
    assert run(0.5, 0.0, front=0.52, prev=TIMEOUT).state == SLOWING


# ---- scan parsing ----

def test_front_min_picks_closest_in_sector():
    r = ring()
    r[idx(0)] = 2.0
    r[idx(20)] = 1.5
    r[idx(-25)] = 1.2
    assert scan(r) == pytest.approx(1.2)


def test_front_min_ignores_outside_sector():
    r = ring()
    r[idx(90)] = 0.2   # right beside the robot
    r[idx(180)] = 0.2  # behind
    r[idx(40)] = 0.3   # just outside +/-30
    assert scan(r) is None


def test_front_min_ignores_inf_and_nan():
    r = ring()
    r[idx(0)] = float('nan')
    r[idx(5)] = float('inf')
    r[idx(-5)] = float('-inf')
    r[idx(10)] = 0.9
    assert scan(r) == pytest.approx(0.9)


def test_front_min_ignores_out_of_range():
    r = ring()
    r[idx(0)] = 0.05   # below range_min (self hits / noise)
    r[idx(5)] = 9.0    # above range_max
    r[idx(-5)] = 0.7
    assert scan(r) == pytest.approx(0.7)


def test_front_min_all_invalid_is_none():
    assert scan(ring(default=float('nan'))) is None
    assert scan(ring()) is None


def test_front_min_handles_zero_to_two_pi_scanners():
    # reading at 359 deg is 1 deg right of straight ahead
    n = 360
    r = ring(n)
    r[359] = 0.8
    assert scan(r, angle_min=0.0) == pytest.approx(0.8)


def test_inf_scan_feeds_through_as_clear():
    front = scan(ring())
    d = run(0.5, 0.0, front=front)
    assert d.state == CLEAR
    assert d.linear == pytest.approx(0.5)


# ---- params ----

@pytest.mark.parametrize('kwargs', [
    dict(stop_distance=1.0, slow_distance=0.5),
    dict(stop_distance=0.5, slow_distance=0.5),
    dict(stop_distance=0.0),
    dict(cmd_timeout=0.0),
    dict(front_angle_deg=0.0),
    dict(creep_speed=-0.1),
])
def test_bad_params_rejected(kwargs):
    with pytest.raises(ValueError):
        SafetyParams(**kwargs)
