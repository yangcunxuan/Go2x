"""Gate, clustering, tracking and goal-conversion tests (no ROS required)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from patrol_global_localization.navigation_gate import (  # noqa: E402
    angle_diff, cluster_hypotheses, convert_goal, navigation_gate,
    track_update, uniqueness_decision, verify_consistent)

NOW = 1000.0


def loc_state(state='LOCALIZED', map_id='factory_a', age=0.5, ok=True):
    return {'state': state, 'map_id': map_id, 'updated_at': NOW - age,
            'ok_for_navigation': ok}


# ---------- gate: fail-closed (P0 audit #11) ----------
def test_gate_missing_state_blocks():
    allowed, reason = navigation_gate(None, 'factory_a', now=NOW)
    assert not allowed and '缺失' in reason


def test_gate_stale_state_blocks():
    allowed, _ = navigation_gate(loc_state(age=5.0), 'factory_a', now=NOW)
    assert not allowed


def test_gate_not_localized_blocks():
    for state in ('SEARCHING', 'VERIFYING', 'AMBIGUOUS', 'DEGRADED', 'LOST'):
        allowed, _ = navigation_gate(loc_state(state=state), 'factory_a', now=NOW)
        assert not allowed, state


def test_gate_map_id_mismatch_blocks():
    allowed, reason = navigation_gate(loc_state(), 'warehouse_b', now=NOW)
    assert not allowed and '不一致' in reason


def test_gate_ok_passes():
    allowed, _ = navigation_gate(loc_state(), 'factory_a', now=NOW)
    assert allowed


# ---------- clustering & uniqueness ----------
def cand(x, y, yaw, fitness, map_id='factory_a'):
    return {'x': x, 'y': y, 'yaw': yaw, 'fitness': fitness, 'map': map_id}


def test_adjacent_keyframes_merge_into_one_cluster():
    clusters = cluster_hypotheses([
        cand(1.0, 1.0, 0.1, 0.8), cand(1.5, 1.2, 0.15, 0.75),  # same place
        cand(9.0, 9.0, -1.0, 0.3),
    ])
    assert len(clusters) == 2
    best, ambiguous, _ = uniqueness_decision(clusters, 0.05)
    assert best['x'] == 1.0 and not ambiguous


def test_ambiguous_candidates_rejected():
    """Two DIFFERENT places with close quality -> AMBIGUOUS (never guess)."""
    clusters = cluster_hypotheses([
        cand(0.0, 0.0, 0.0, 0.72), cand(20.0, 20.0, 2.0, 0.70, map_id='factory_a'),
    ])
    best, ambiguous, margin = uniqueness_decision(clusters, 0.05)
    assert ambiguous and margin < 0.05


def test_clear_winner_not_ambiguous():
    clusters = cluster_hypotheses([
        cand(0.0, 0.0, 0.0, 0.85), cand(20.0, 20.0, 2.0, 0.40),
    ])
    best, ambiguous, margin = uniqueness_decision(clusters, 0.05)
    assert best['fitness'] == 0.85 and not ambiguous and margin > 0.4


# ---------- tracking / LOST ----------
def identity():
    return np.eye(4)


def test_track_converges_and_stays():
    count, lost = track_update(0, True, 0.1, 0.05)
    assert count == 0 and not lost


def test_track_bad_frames_reach_lost():
    # Each bad frame adds 3 units; the 6-unit threshold means two
    # consecutive bad frames trigger LOST.
    count, lost = track_update(0, False, 0.0, 0.0)
    assert count == 3 and not lost
    count, lost = track_update(count, False, 0.0, 0.0)
    assert lost, 'two consecutive bad frames (6 units) must trigger LOST'


def test_track_huge_jump_is_immediate_lost():
    count, lost = track_update(0, True, 1.5, 0.05)
    assert lost and count == 0


def test_track_yaw_jump_wraparound():
    """179 -> -179 deg is a 2 deg jump, never 358 (P0 audit yaw wrap)."""
    assert angle_diff(math.radians(179), math.radians(-179)) < math.radians(2)
    count, lost = track_update(0, True, 0.1, math.radians(2))
    assert not lost


# ---------- goal conversion ----------
def test_convert_goal_roundtrip():
    t = yaw_matrix(3.0, -2.0, 0.5, 0.7)
    local = (1.2, -0.4, 0.1)
    world = t[:3, :3] @ np.array(local) + t[:3, 3]
    back = convert_goal(t, *world)
    assert all(abs(a - b) < 1e-9 for a, b in zip(back, local))


def yaw_matrix(x, y, z, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    t = np.eye(4)
    t[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    t[:3, 3] = [x, y, z]
    return t


# ---------- GICP known-transform recovery (needs small_gicp) ----------
def test_gicp_known_transform_recovery():
    small_gicp = pytest.importorskip('small_gicp')
    from patrol_global_localization.localization_manager import Registration
    from patrol_global_localization.navigation_gate import angle_diff
    rng = np.random.default_rng(9)
    target = np.column_stack([
        rng.uniform(-10, 10, 20000), rng.uniform(-10, 10, 20000),
        rng.uniform(0, 3, 20000)])
    true_t = yaw_matrix(1.2, -0.8, 0.05, math.radians(12))
    source = target[::2] @ true_t[:3, :3].T + true_t[:3, 3]
    source = source + rng.normal(0, 0.01, source.shape)
    reg = Registration.__new__(Registration)
    # perturbed initial guess: 0.5 m and 5 deg off
    init = true_t @ yaw_matrix(0.5, 0.3, 0.0, math.radians(-5))
    result = reg.align(target, source, init, 0.5, 2.5)
    pos_err = np.linalg.norm(result['T'][:3, 3] - true_t[:3, 3])
    yaw_err = angle_diff(yaw_of(result['T']), yaw_of(true_t))
    assert pos_err < 0.15, pos_err
    assert yaw_err < math.radians(3), yaw_err
    assert result['quality_ok'], result
