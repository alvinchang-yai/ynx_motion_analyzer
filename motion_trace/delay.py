"""Delay measurement building blocks used by the dashboard's analysis (analysis.py).

Signals are (t [s], p [rad]) numpy arrays. A "ramp" is one move of the command signal:
idle -> moving -> idle.
"""
import numpy as np


def _interpolated_crossing(t0, t1, p0, p1, target):
    if p1 == p0:
        return t1
    return t0 + (target - p0) / (p1 - p0) * (t1 - t0)


def find_rest_point(t, p, before_time, tol_rad=1e-6):
    # The last moment at or before `before_time` when `p` was still at rest, and its
    # value there: walk back from `before_time` while consecutive samples still
    # differ. Delays are measured from this point, for command and feedback alike -
    # baselining the command at an "it has started moving" detection point instead
    # (already a little way into the move, while the lagging feedback is still at
    # rest) makes the command look a few ms later than it is.
    if len(t) == 0:
        return None, None
    i = int(np.searchsorted(t, before_time, side='right')) - 1
    i = min(max(i, 0), len(p) - 1)
    while i > 0 and abs(p[i] - p[i - 1]) > tol_rad:
        i -= 1
    return t[i], p[i]


def crossing_direction(t, p, cross_t, baseline):
    # +1 / -1: which side of `baseline` the signal was on when it crossed.
    return 1.0 if np.interp(cross_t, t, p) >= baseline else -1.0


def find_signal_threshold_time(t, p, after_time, baseline, threshold_deg=1.0, direction=None):
    # From `after_time` onward, scan `p` to find when it has moved more than
    # `threshold_deg` away from `baseline` - a reference value shared by both
    # the sent and feedback scans (each signal's own value at `after_time`, the
    # detected command-start time), not each signal's own array-start sample.
    # That's what makes "command reaches 1 deg" and "feedback reaches 1 deg"
    # directly comparable: both are measured from the same moment in time.
    # `direction` (+1/-1) only accepts crossings on that side of the baseline: pass
    # the command's crossing direction when scanning feedback, so a small overshoot
    # the other way (or a later move) can't be paired with the command's crossing.
    if len(p) < 2:
        return None
    threshold_rad = np.radians(threshold_deg)
    mask = t >= after_time
    t_seg = np.concatenate(([after_time], t[mask]))
    p_seg = np.concatenate(([np.interp(after_time, t, p)], p[mask]))
    for i in range(1, len(p_seg)):
        delta = p_seg[i] - baseline
        if abs(delta) > threshold_rad and (direction is None or np.sign(delta) == direction):
            target = baseline + np.sign(delta) * threshold_rad
            return _interpolated_crossing(t_seg[i - 1], t_seg[i], p_seg[i - 1], p_seg[i], target)
    return None


def segment_ramps(sent_t, sent_p, velocity_threshold_deg_s=1.0, min_ramp_peak_deg=1.0):
    # Find every idle->moving->idle interval in the commanded signal via a
    # velocity threshold. Only run against the command signal, not feedback -
    # the commanded signal is clean/noise-free, so a simple threshold cleanly
    # separates idle from ramp; feedback has real jitter that would make this
    # unreliable. Returns a list of dicts: start_s (last rest sample before the
    # ramp), end_s, baseline (sent_p at start_s), peak_deg (max |deviation| from
    # baseline reached within the ramp).
    vel = np.zeros_like(sent_p)
    dt = np.diff(sent_t)
    dp_deg = np.diff(np.degrees(sent_p))
    with np.errstate(divide='ignore', invalid='ignore'):
        vel[1:] = np.where(dt > 0, dp_deg / dt, 0.0)

    moving = np.abs(vel) > velocity_threshold_deg_s
    ramps = []
    i = 0
    n = len(moving)
    while i < n:
        if moving[i]:
            start_idx = i
            while i < n and moving[i]:
                i += 1
            end_idx = i - 1
            # One sample before/after the moving stretch, so the ramp's own
            # window includes its true start/end (the transition sample
            # itself), not just the interior "definitely moving" samples.
            start_i = max(start_idx - 1, 0)
            end_i = min(end_idx + 1, n - 1)
            # Measure from where the command was still at rest, not from the sample
            # where it first exceeded the velocity threshold (already partway in).
            start_s, baseline = find_rest_point(sent_t, sent_p, sent_t[start_i])
            end_s = sent_t[end_i]
            seg_mask = (sent_t >= start_s) & (sent_t <= end_s)
            peak_deg = float(np.degrees(np.max(np.abs(sent_p[seg_mask] - baseline))))
            if peak_deg >= min_ramp_peak_deg:
                ramps.append(dict(start_s=start_s, end_s=end_s, baseline=baseline, peak_deg=peak_deg))
        else:
            i += 1
    return ramps


def sample_delays(sent_t, sent_p, fb_t, fb_p, ramps, n_samples=10, seed=42):
    # Draw n_samples random (ramp, threshold) pairs spread across every
    # detected ramp and measure the command->feedback delay at each - a more
    # robust estimate than a single fixed threshold on just the first ramp,
    # since it covers every direction/segment of a repeating move. Each ramp
    # has its own local baseline/direction (a descending ramp's baseline is
    # wherever the previous ramp ended, not always 0), so thresholds are
    # scaled to that ramp's own reachable range, not a single global one.
    rng = np.random.default_rng(seed)
    ramp_choices = rng.integers(0, len(ramps), size=n_samples)
    threshold_fracs = rng.uniform(0.0, 1.0, size=n_samples)

    per_ramp_delays = {i: [] for i in range(len(ramps))}
    all_delays = []
    for ramp_idx, frac in zip(ramp_choices, threshold_fracs):
        r = ramps[ramp_idx]
        threshold_deg = 0.3 + frac * (r['peak_deg'] * 0.95 - 0.3)
        fb_baseline_local = np.interp(r['start_s'], fb_t, fb_p)
        sent_th = find_signal_threshold_time(sent_t, sent_p, r['start_s'], r['baseline'], threshold_deg)
        fb_th = None if sent_th is None else find_signal_threshold_time(
            fb_t, fb_p, r['start_s'], fb_baseline_local, threshold_deg,
            direction=crossing_direction(sent_t, sent_p, sent_th, r['baseline']))
        if sent_th is None or fb_th is None:
            continue
        delay_ms = (fb_th - sent_th) * 1000
        per_ramp_delays[ramp_idx].append(delay_ms)
        all_delays.append(delay_ms)
    return np.array(all_delays), per_ramp_delays
