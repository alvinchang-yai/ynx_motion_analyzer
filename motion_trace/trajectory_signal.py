"""Planned trajectories (e.g. MoveIt's /display_planned_path) as a command signal.

A planned trajectory carries only relative times (time_from_start) and is published
at planning time - for plan-only requests too - so on its own it says neither when
nor whether it ran. This module pairs every executed action goal with the plan it
came from and anchors that plan to the goal's acceptance stamp from the action
server's status topic. That stamp is on the controller's clock, the same clock as
the controller's feedback, so no PC <-> controller clock sync is needed. The
measured delay therefore starts when the controller accepted the goal: it includes
the controller's start-up and servo lag but not the PC -> controller network time.

Pairing uses bag receive times (all on this PC's clock): a goal executes the next
unused segment of the most recent plan received before the goal first appears.
Multi-segment plans (one goal per segment) are consumed in order. Each pairing is
checked against where the arm actually was when the goal started.
"""
from dataclasses import dataclass

import numpy as np

# action_msgs/GoalStatus: goals that reached the controller's execution stage.
_EXECUTED_STATUSES = {2, 3, 4, 5, 6}  # EXECUTING, CANCELING, SUCCEEDED, CANCELED, ABORTED
START_MISMATCH_WARN_RAD = np.radians(0.5)


@dataclass
class AnchoredSegment:
    t0: float              # goal acceptance, controller clock [s]
    joint_names: list
    times: np.ndarray      # time_from_start of each point [s]
    positions: np.ndarray  # (n_points, n_joints)
    velocities: np.ndarray = None
    accelerations: np.ndarray = None
    goal_id: str = ''
    anchor: str = 'status'


def _get(obj, dotted):
    for part in dotted.split('.'):
        obj = getattr(obj, part)
    return obj


def _goal_key(goal_id):
    return bytes(goal_id.uuid).hex()


def _stamp(t):
    return t.sec + t.nanosec / 1e9


def _segments_of(msg, field):
    # moveit_msgs/DisplayTrajectory.trajectory is a list of RobotTrajectory; also accept
    # a single RobotTrajectory or a bare trajectory_msgs/JointTrajectory.
    value = _get(msg, field)
    items = value if isinstance(value, (list, tuple)) else [value]
    segments = []
    for item in items:
        jt = getattr(item, 'joint_trajectory', item)
        if not jt.points:
            continue
        n = len(jt.joint_names)

        def column(attr):
            rows = [getattr(p, attr) for p in jt.points]
            return np.array(rows, dtype=float) if all(len(r) == n for r in rows) else None

        segments.append(dict(
            joint_names=list(jt.joint_names),
            times=np.array([_stamp(p.time_from_start) for p in jt.points]),
            positions=column('positions'),
            velocities=column('velocities'),
            accelerations=column('accelerations'),
        ))
    return segments


def _goals(data, signal):
    # {goal_key: dict(first_seen_receive_s, accept_stamp_s, executed, first_feedback)}
    goals = {}
    for receive_s, msg in data.get(signal.anchor_topic, []) if signal.anchor_topic else []:
        for st in msg.status_list:
            g = goals.setdefault(_goal_key(st.goal_info.goal_id), dict(
                first_seen=receive_s, accept=None, executed=False, first_feedback=None,
                first_status=st.status))
            if receive_s < g['first_seen']:
                g['first_seen'], g['first_status'] = receive_s, st.status
            stamp = _stamp(st.goal_info.stamp)
            if stamp > 0:
                g['accept'] = stamp
            g['executed'] |= st.status in _EXECUTED_STATUSES
    # Feedback (also carries goal_id) proves execution and gives a fallback anchor:
    # its first stamp, accurate to one feedback period.
    for receive_s, msg in data.get(signal.anchor_feedback_topic, []) if signal.anchor_feedback_topic else []:
        g = goals.setdefault(_goal_key(msg.goal_id), dict(
            first_seen=receive_s, accept=None, executed=False, first_feedback=None, first_status=None))
        g['first_seen'] = min(g['first_seen'], receive_s)
        g['executed'] = True
        fb = msg.feedback
        stamp = _stamp(fb.header.stamp)
        if g['first_feedback'] is None or (stamp > 0 and stamp < g['first_feedback'][0]):
            g['first_feedback'] = (stamp if stamp > 0 else receive_s, list(fb.joint_names),
                                   list(fb.actual.positions))
    # A goal whose first appearance is already terminal (succeeded/canceled/aborted)
    # with no feedback in the bag finished before the recording started.
    finished_before = {4, 5, 6}
    return {k: g for k, g in goals.items() if g['executed']
            and not (g['first_status'] in finished_before and g['first_feedback'] is None)}


# Every axis of a recording needs the same segments, so the last result is kept. Keyed on
# the data object itself (identity), not id(data): Python reuses a freed object's id, so an
# id-keyed cache could hand one recording's plans to the next recording loaded.
_last = (None, None, None)  # (data, signal key, segments)


def anchored_segments(data, signal):
    global _last
    if _last[0] is data and _last[1] == signal.key:
        return _last[2]
    events = [(r, 'plan', m) for r, m in data.get(signal.topic, [])]
    events += [(g['first_seen'], 'goal', (k, g)) for k, g in _goals(data, signal).items()]
    events.sort(key=lambda e: (e[0], e[1] == 'goal'))  # a plan and its goal: plan first

    pending, out = [], []
    for _, kind, payload in events:
        if kind == 'plan':
            pending = _segments_of(payload, signal.field)  # a newer plan supersedes unused ones
            continue
        goal_key, g = payload
        if not pending:
            print(f'  Note: executed goal {goal_key[:8]} has no matching plan on {signal.topic} '
                  '(started before the recording, or not sent by MoveIt) - skipped.')
            continue
        seg = pending.pop(0)
        if g['accept'] is not None:
            t0, anchor = g['accept'], 'status'
        elif g['first_feedback'] is not None:
            t0, anchor = g['first_feedback'][0], 'first feedback'
        else:
            continue
        if seg['positions'] is None:
            continue
        if g['first_feedback'] is not None:
            _, names, actual = g['first_feedback']
            idx = {n: i for i, n in enumerate(names)}
            diffs = [abs(seg['positions'][0][j] - actual[idx[n]])
                     for j, n in enumerate(seg['joint_names']) if n in idx]
            if diffs and max(diffs) > START_MISMATCH_WARN_RAD:
                print(f'  Warning: plan start differs from the arm\'s position at goal {goal_key[:8]} by '
                      f'{np.degrees(max(diffs)):.2f} deg - the plan/goal pairing may be wrong.')
        out.append(AnchoredSegment(t0=t0, goal_id=goal_key, anchor=anchor, **seg))
    out.sort(key=lambda s: s.t0)
    _last = (data, signal.key, out)
    return out


def quintic_hermite(t, p, v, a, grid):
    """Piecewise quintic Hermite through points (t, p, v, a), evaluated at grid.

    Exact for the constant-acceleration phases planners like Pilz produce. Returns
    (positions, velocities, accelerations) at grid. v/a may be None: a missing
    acceleration is taken as 0 (cubic Hermite), missing velocities give linear
    interpolation (and None derivatives)."""
    t, p, grid = np.asarray(t, float), np.asarray(p, float), np.asarray(grid, float)
    if v is None:
        return np.interp(grid, t, p), None, None
    v = np.asarray(v, float)
    a = np.zeros_like(p) if a is None else np.asarray(a, float)
    i = np.clip(np.searchsorted(t, grid, side='right') - 1, 0, len(t) - 2)
    h = t[i + 1] - t[i]
    safe_h = np.where(h > 0, h, 1.0)
    s = np.where(h > 0, (grid - t[i]) / safe_h, 0.0)
    p0, p1 = p[i], p[i + 1]
    v0, v1 = v[i] * h, v[i + 1] * h
    a0, a1 = a[i] * h * h, a[i + 1] * h * h
    s2, s3, s4, s5 = s**2, s**3, s**4, s**5
    # Basis functions and their first/second derivatives with respect to s.
    basis = (
        (1 - 10 * s3 + 15 * s4 - 6 * s5, -30 * s2 + 60 * s3 - 30 * s4, -60 * s + 180 * s2 - 120 * s3),
        (s - 6 * s3 + 8 * s4 - 3 * s5, 1 - 18 * s2 + 32 * s3 - 15 * s4, -36 * s + 96 * s2 - 60 * s3),
        (0.5 * (s2 - 3 * s3 + 3 * s4 - s5), 0.5 * (2 * s - 9 * s2 + 12 * s3 - 5 * s4),
         0.5 * (2 - 18 * s + 36 * s2 - 20 * s3)),
        (0.5 * (s3 - 2 * s4 + s5), 0.5 * (3 * s2 - 8 * s3 + 5 * s4), 0.5 * (6 * s - 24 * s2 + 20 * s3)),
        (-4 * s3 + 7 * s4 - 3 * s5, -12 * s2 + 28 * s3 - 15 * s4, -24 * s + 84 * s2 - 60 * s3),
        (10 * s3 - 15 * s4 + 6 * s5, 30 * s2 - 60 * s3 + 30 * s4, 60 * s - 180 * s2 + 120 * s3),
    )
    coeffs = (p0, v0, a0, a1, v1, p1)
    pos = sum(c * b[0] for c, b in zip(coeffs, basis))
    vel = sum(c * b[1] for c, b in zip(coeffs, basis)) / safe_h
    acc = sum(c * b[2] for c, b in zip(coeffs, basis)) / safe_h**2
    return pos, vel, acc


def resample_grid(t_start, t_end, rate_hz):
    # Evenly spaced at 1/rate_hz, always ending exactly on t_end.
    grid = np.arange(t_start, t_end, 1.0 / rate_hz)
    if len(grid) and t_end - grid[-1] < 0.5 / rate_hz:
        grid = grid[:-1]  # avoid a sliver of a step right before the end
    return np.append(grid, t_end)


def _sample_segment(seg, j, rate_hz):
    grid = resample_grid(seg.times[0], seg.times[-1], rate_hz)
    v = seg.velocities[:, j] if seg.velocities is not None else None
    a = seg.accelerations[:, j] if seg.accelerations is not None else None
    return grid, quintic_hermite(seg.times, seg.positions[:, j], v, a, grid)[0]


def trajectory_series(data, signal, axis):
    # One joint's commanded (times, positions) across every executed plan, on the
    # controller's clock, resampled at signal.sample_rate_hz.
    times, positions = [], []
    for seg in anchored_segments(data, signal):
        if axis.joint and axis.joint in seg.joint_names:
            j = seg.joint_names.index(axis.joint)
        elif not axis.joint and axis.index < len(seg.joint_names):
            j = axis.index
        else:
            continue
        t, p = _sample_segment(seg, j, signal.sample_rate_hz)
        times.append(seg.t0 + t)
        positions.append(p)
    if not times:
        return np.array([]), np.array([])
    return np.concatenate(times), np.concatenate(positions)
