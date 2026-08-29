"""Pure navigation-gate logic (no ROS imports) so it is unit-testable.

The motion bridge, the web goal layer and the nav manager all enforce the
same rule set. Decisions here are FAIL-CLOSED: any missing, stale,
non-LOCALIZED or mismatched input blocks motion.
"""
import math
import time

STATE_FRESH_SEC = 2.0


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def navigation_gate(loc_state, active_map_id, now=None,
                    require_map_binding=True):
    """Evaluate the localization hard gate.

    loc_state: parsed localization_state.json (or None if unreadable)
    active_map_id: active map id from active_map.json (None = unknown)
    Returns (allowed: bool, reason: str).
    """
    now = time.time() if now is None else now
    if not isinstance(loc_state, dict) or not loc_state.get("updated_at"):
        return False, "localization_state.json 缺失：定位未运行"
    if now - float(loc_state["updated_at"]) > STATE_FRESH_SEC:
        return False, "定位状态过期"
    if loc_state.get("state") != "LOCALIZED":
        return False, f"定位状态非LOCALIZED: {loc_state.get('state')}"
    if not loc_state.get("ok_for_navigation"):
        return False, "ok_for_navigation=false"
    if require_map_binding and active_map_id and \
            loc_state.get("map_id") != active_map_id:
        return False, (f"定位地图({loc_state.get('map_id')})与"
                       f"活动地图({active_map_id})不一致")
    return True, "LOCALIZED"


def cluster_hypotheses(scored, pos_tol=1.0, yaw_tol=math.radians(10.0)):
    """Group registration hypotheses by final pose (position + yaw).

    scored: iterable of dicts with x, y, yaw, map, fitness.
    Returns clusters: list of lists, each cluster = one place hypothesis.
    Candidates in the same cluster belong to the same place (adjacent
    keyframes); different clusters are genuinely different places.
    """
    clusters = []
    for cand in sorted(scored, key=lambda c: -float(c.get("fitness", 0.0))):
        for cluster in clusters:
            head = cluster[0]
            same_place = (
                math.hypot(cand["x"] - head["x"], cand["y"] - head["y"]) < pos_tol
                and angle_diff(cand["yaw"], head["yaw"]) < yaw_tol
                and cand.get("map") == head.get("map"))
            if same_place:
                cluster.append(cand)
                break
        else:
            clusters.append([cand])
    clusters.sort(key=lambda c: -max(float(m.get("fitness", 0.0)) for m in c))
    return clusters


def uniqueness_decision(clusters, unique_margin=0.05):
    """(best_candidate, ambiguous: bool, margin: float|None)."""
    if not clusters:
        return None, True, None
    best_cluster = clusters[0]
    best = max(best_cluster, key=lambda c: float(c.get("fitness", 0.0)))
    if len(clusters) < 2:
        return best, False, None
    second = max(clusters[1], key=lambda c: float(c.get("fitness", 0.0)))
    margin = float(best.get("fitness", 0.0)) - float(second.get("fitness", 0.0))
    return best, margin < unique_margin, margin


def verify_consistent(previous, t_new, pos_tol=0.15, yaw_tol=math.radians(5.0)):
    """All previous VERIFYING frames must lie within tolerance of t_new."""
    for prev in previous:
        if np_hypot(prev, t_new) > pos_tol:
            return False
        if angle_diff(yaw_of(prev), yaw_of(t_new)) > yaw_tol:
            return False
    return True


def np_hypot(t_a, t_b):
    dx = float(t_a[0, 3]) - float(t_b[0, 3])
    dy = float(t_a[1, 3]) - float(t_b[1, 3])
    return math.hypot(dx, dy)


def yaw_of(t):
    return math.atan2(float(t[1, 0]), float(t[0, 0]))


def track_update(degraded_count, result_ok, jump, jump_yaw,
                 correct_max=0.30, correct_yaw_max=math.radians(10.0),
                 lost_correct_max=1.0, lost_jump_yaw=math.radians(20.0)):
    """Returns (new_count, lost: bool). LOST on abnormal correction jumps or
    after enough consecutive bad frames."""
    if not result_ok:
        degraded_count += 3
    elif jump > lost_correct_max or jump_yaw > lost_jump_yaw:
        return 0, True
    elif jump > correct_max or jump_yaw > correct_yaw_max:
        degraded_count += 1
    else:
        degraded_count = 0
    if degraded_count >= 6:
        return 0, True
    return degraded_count, False


def convert_goal(matrix, x, y, z):
    """map_level goal -> camera_init via the inverse of the given TF."""
    import numpy as np
    inv = np.linalg.inv(np.asarray(matrix, dtype=np.float64))
    c = inv @ np.array([x, y, z, 1.0])
    return float(c[0]), float(c[1]), float(c[2])
