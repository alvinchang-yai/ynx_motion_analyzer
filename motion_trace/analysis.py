"""Analysis as plain data, for the dashboard (and anything else that isn't a CLI).

Profile loading comes from hardware_profile, the delay measurements from delay.
Everything returned is JSON-serializable.
"""
import os
import types
from datetime import datetime

import numpy as np

from motion_trace.delay import crossing_direction, find_signal_threshold_time, sample_delays, segment_ramps
from motion_trace.hardware_profile import (
    bag_topics, detect_profile, extract_series, load_profile, read_bag, require_topics, resolve_axes,
)

# Each move gets two delays:
#   start  - command vs. feedback leaving rest: the time both have moved START_THRESHOLD_DEG
#            from where they stood when the command's move began. The encoder reads exactly
#            constant at rest, so this is just above quantization; below ~0.05 deg the reading
#            drops 10-15 ms because the arm creeps off more gently than the plan.
#   motion - mean of n_samples delays at random positions through the move (0.3 deg to 95% of
#            its travel).
START_THRESHOLD_DEG = 0.05
DEFAULTS = dict(n_samples=10, seed=42, velocity_threshold_deg_s=1.0, min_ramp_peak_deg=1.0)
# A move whose delay differs from the axis' typical (median) move delay by more than
# this is flagged in the summary.
OUTLIER_FRACTION = 0.2
MAX_POINTS_PER_TRACE = 20000


def _round(values, ndigits):
    return [round(float(v), ndigits) for v in values]


def _decimate(t, p, limit=MAX_POINTS_PER_TRACE):
    # Min/max per bucket keeps peaks and steps visible when a long recording has
    # more points than a browser chart handles comfortably.
    if len(t) <= limit:
        return t, p, False
    buckets = limit // 2
    edges = np.linspace(0, len(t), buckets + 1, dtype=int)
    keep = []
    for a, b in zip(edges[:-1], edges[1:]):
        if b > a:
            seg = p[a:b]
            keep += sorted({a + int(np.argmin(seg)), a + int(np.argmax(seg))})
    keep = np.array(keep)
    return t[keep], p[keep], True


def list_bags(experiment_dir):
    """Recordings under experiment_dir, newest first, with the profile they match."""
    bags = []
    if not os.path.isdir(experiment_dir):
        return bags
    no_overrides = types.SimpleNamespace(set=[])
    for name in os.listdir(experiment_dir):
        path = os.path.join(experiment_dir, name)
        if not os.path.isfile(os.path.join(path, 'metadata.yaml')):
            continue
        entry = dict(name=name, modified=datetime.fromtimestamp(os.path.getmtime(path)).isoformat(timespec='seconds'),
                     size_mb=round(sum(os.path.getsize(os.path.join(path, f)) for f in os.listdir(path)
                                       if os.path.isfile(os.path.join(path, f))) / 1e6, 2))
        try:
            topics = bag_topics(path)
            profile = detect_profile(no_overrides, topics)
            entry.update(topics=sorted(topics), profile=profile.name if profile else None, readable=True)
        except Exception as e:  # e.g. a bag recorded by a newer ROS distro
            entry.update(topics=[], profile=None, readable=False, error=str(e).splitlines()[0][:200])
        bags.append(entry)
    bags.sort(key=lambda b: b['modified'], reverse=True)
    return bags


def _rms_after_shift(cmd_t, cmd_p, fb_t, fb_p, delay_s):
    shifted = fb_t - delay_s
    lo, hi = max(fb_t.min(), shifted.min()), min(fb_t.max(), shifted.max())
    m = (cmd_t >= lo) & (cmd_t <= hi)
    if not m.any():
        return None, None
    raw = np.degrees(np.interp(cmd_t[m], fb_t, fb_p) - cmd_p[m])
    sh = np.degrees(np.interp(cmd_t[m], shifted, fb_p) - cmd_p[m])
    return float(np.sqrt(np.mean(raw ** 2))), float(np.sqrt(np.mean(sh ** 2)))


def _start_delay_ms(cmd_t, cmd_p, fb_t, fb_p, ramp):
    c = find_signal_threshold_time(cmd_t, cmd_p, ramp['start_s'], ramp['baseline'], START_THRESHOLD_DEG)
    if c is None:
        return None
    f = find_signal_threshold_time(fb_t, fb_p, ramp['start_s'], np.interp(ramp['start_s'], fb_t, fb_p),
                                   START_THRESHOLD_DEG, direction=crossing_direction(cmd_t, cmd_p, c, ramp['baseline']))
    return None if f is None else float((f - c) * 1000)


def analyze_axis(profile, data, axis, t0, opts):
    cmd_sig, fb_sig = profile.command_signal, profile.feedback_signal
    series = {}
    for sig in profile.signals:
        t, p = extract_series(data, sig, axis)
        series[sig.key] = (t - t0, p)
    cmd_t, cmd_p = series[cmd_sig.key]
    fb_t, fb_p = series[fb_sig.key]

    result = dict(label=axis.label, joint=axis.joint, moved=False, signals=[], moves=[],
                  overall=None, shift=None, flags=[])
    decimated = False
    for idx, sig in enumerate(profile.signals):
        t, p = series[sig.key]
        td, pd, d = _decimate(t, p)
        decimated |= d
        result['signals'].append(dict(key=sig.key, label=sig.label, slot=idx + 1,
                                      role='command' if sig.key == cmd_sig.key else
                                           'feedback' if sig.key == fb_sig.key else 'context',
                                      t=_round(td, 4), p=_round(np.degrees(pd), 4)))
    result['decimated'] = decimated
    if len(cmd_t) < 2 or len(fb_t) < 2:
        result['flags'].append(dict(level='info', text='Not enough samples on the command or feedback signal.'))
        return result

    ramps = segment_ramps(cmd_t, cmd_p, opts['velocity_threshold_deg_s'], opts['min_ramp_peak_deg'])
    result['moved'] = bool(ramps)
    for i, r in enumerate(ramps):
        d = list(sample_delays(cmd_t, cmd_p, fb_t, fb_p, [r], opts['n_samples'], opts['seed'] + i)[0])
        end_val = np.interp(r['end_s'], cmd_t, cmd_p)
        result['moves'].append(dict(
            index=i + 1, start_s=float(r['start_s']), end_s=float(r['end_s']),
            travel_deg=float(np.degrees(end_val - r['baseline'])), peak_deg=r['peak_deg'],
            start_ms=_start_delay_ms(cmd_t, cmd_p, fb_t, fb_p, r), n=len(d),
            mean_ms=float(np.mean(d)) if d else None, min_ms=float(min(d)) if d else None,
            max_ms=float(max(d)) if d else None))
    motion = [m['mean_ms'] for m in result['moves'] if m['mean_ms'] is not None]
    starts = [m['start_ms'] for m in result['moves'] if m['start_ms'] is not None]
    if motion:
        typical = float(np.median(motion))
        result['overall'] = dict(typical_motion_ms=typical,
                                 typical_start_ms=float(np.median(starts)) if starts else None)
        for m in result['moves']:
            if m['mean_ms'] is not None and abs(m['mean_ms'] - typical) > OUTLIER_FRACTION * typical:
                result['flags'].append(dict(level='warning', move=m['index'], text=(
                    f"Move {m['index']}: {m['mean_ms']:.0f} ms vs typical {typical:.0f} ms")))
        # Shift by the typical motion delay (not just the first move's, which may be an
        # outlier) and report how well feedback then matches the command.
        rms_raw, rms_shifted = _rms_after_shift(cmd_t, cmd_p, fb_t, fb_p, typical / 1000)
        result['shift'] = dict(delay_ms=typical, rms_unshifted_deg=rms_raw, rms_shifted_deg=rms_shifted)
    return result


def analyze_bag(bag_path, profile_name=None, **options):
    """Full analysis of one recording: per axis series, delays, moves, shift check."""
    opts = dict(DEFAULTS, **{k: v for k, v in options.items() if v is not None})
    topics = bag_topics(bag_path)
    no_overrides = types.SimpleNamespace(set=[])
    profile = load_profile(profile_name) if profile_name else (
        detect_profile(no_overrides, topics) or load_profile('motoros2'))
    data = read_bag(bag_path, profile.topics())
    require_topics(profile, data, [profile.command, profile.feedback], topics)

    axes = resolve_axes(profile, data)
    starts = [extract_series(data, s, a)[0] for s in (profile.command_signal, profile.feedback_signal) for a in axes]
    t0 = min(float(t[0]) for t in starts if len(t))
    t_end = max(float(t[-1]) for t in starts if len(t))
    results = [analyze_axis(profile, data, a, t0, opts) for a in axes]

    moved = [r for r in results if r['overall']]
    motion = [r['overall']['typical_motion_ms'] for r in moved]
    start = [r['overall']['typical_start_ms'] for r in moved if r['overall']['typical_start_ms'] is not None]
    return dict(
        bag=os.path.basename(os.path.normpath(bag_path)),
        profile=dict(name=profile.name, description=profile.description,
                     command=profile.command_signal.label, feedback=profile.feedback_signal.label),
        start_stamp=t0, duration_s=t_end - t0, options=opts,
        summary=dict(typical_motion_ms=float(np.median(motion)) if motion else None,
                     typical_start_ms=float(np.median(start)) if start else None,
                     moved_axes=[r['label'] for r in moved]),
        axes=results,
    )
