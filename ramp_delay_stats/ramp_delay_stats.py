import argparse

import numpy as np

from motion_trace.plot_motion import AXIS_NAMES, read_bag, extract_series_by_index, find_signal_threshold_time


def segment_ramps(sent_t, sent_p, velocity_threshold_deg_s=1.0, min_ramp_peak_deg=1.0):
    # Find every idle->moving->idle interval in the commanded signal via a
    # velocity threshold. Only run against joint_command_sent, not feedback -
    # the commanded signal is clean/noise-free, so a simple threshold cleanly
    # separates idle from ramp; feedback has real jitter that would make this
    # unreliable. Returns a list of dicts: start_s, end_s, baseline (sent_p at
    # start), peak_deg (max |deviation| from baseline reached within the ramp).
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
            start_s = sent_t[start_i]
            end_s = sent_t[end_i]
            baseline = sent_p[start_i]
            seg_mask = (sent_t >= start_s) & (sent_t <= end_s)
            peak_deg = float(np.degrees(np.max(np.abs(sent_p[seg_mask] - baseline))))
            if peak_deg >= min_ramp_peak_deg:
                ramps.append(dict(start_s=start_s, end_s=end_s, baseline=baseline, peak_deg=peak_deg))
        else:
            i += 1
    return ramps


def sample_delays(sent_t, sent_p, fb_t, fb_p, ramps, n_samples=20, seed=42):
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
        fb_th = find_signal_threshold_time(fb_t, fb_p, r['start_s'], fb_baseline_local, threshold_deg)
        if sent_th is None or fb_th is None:
            continue
        delay_ms = (fb_th - sent_th) * 1000
        per_ramp_delays[ramp_idx].append(delay_ms)
        all_delays.append(delay_ms)
    return np.array(all_delays), per_ramp_delays


def report_axis(sent_samples, feedback_samples, index, axis_label, n_samples, seed,
                 velocity_threshold_deg_s, min_ramp_peak_deg):
    sent_t, sent_p = extract_series_by_index(sent_samples, index)
    fb_t, fb_p = extract_series_by_index(feedback_samples, index)
    if len(sent_t) < 2 or len(fb_t) < 2:
        print(f'Axis {axis_label}: not enough samples on joint_command_sent or joint_feedback - skipped.')
        return
    t0 = min(sent_t[0], fb_t[0])
    sent_t, fb_t = sent_t - t0, fb_t - t0

    ramps = segment_ramps(sent_t, sent_p, velocity_threshold_deg_s, min_ramp_peak_deg)
    if not ramps:
        print(f'Axis {axis_label}: no ramps detected (command never moved) - skipped.')
        return

    all_delays, per_ramp = sample_delays(sent_t, sent_p, fb_t, fb_p, ramps, n_samples, seed)
    if len(all_delays) == 0:
        print(f'Axis {axis_label}: {len(ramps)} ramps detected, but no valid samples - skipped.')
        return

    print(f'Axis {axis_label}: {len(ramps)} ramps detected, {len(all_delays)}/{n_samples} samples valid')
    for ramp_idx, delays in per_ramp.items():
        if delays:
            print(f'  ramp {ramp_idx}: n={len(delays)}  mean={np.mean(delays):.2f} ms  '
                  f'[{min(delays):.2f}, {max(delays):.2f}] ms')
    print(f'  OVERALL: mean={np.mean(all_delays):.2f} ms  std={np.std(all_delays):.2f} ms  '
          f'min={np.min(all_delays):.2f} ms  max={np.max(all_delays):.2f} ms')


def main():
    parser = argparse.ArgumentParser(
        description="More robust command->feedback delay estimate than plot_motion's single "
                    "--threshold-deg measurement: segments the whole recording into every distinct "
                    'idle->moving->idle ramp (a repeating loop move produces several), then draws '
                    'randomized-threshold samples spread across all of them and reports mean/std/min/max, '
                    'both overall and per ramp.')
    parser.add_argument('bag_path', help='Path to the rosbag2 directory (the -o used with record_motion)')
    parser.add_argument(
        '--ns', default='nex10',
        help="The bringup launch's 'ns' argument used when the bag was recorded (default: 'nex10'). "
             "Pass '' if the bag was recorded with no namespace.")
    parser.add_argument(
        '--hw-node', default='nex10',
        help="The hardware component's own node name (hardcoded 'nex10' in the xacro, independent of --ns).")
    parser.add_argument(
        '--axis', action='append', choices=AXIS_NAMES,
        help='Axis to check (repeatable, e.g. --axis S --axis T). Default: all six axes.')
    parser.add_argument(
        '--n-samples', type=int, default=20,
        help='Total number of randomized-threshold samples to draw, spread across all detected ramps '
             '(default: 20).')
    parser.add_argument(
        '--seed', type=int, default=42,
        help='Random seed, for reproducible sampling across runs (default: 42).')
    parser.add_argument(
        '--velocity-threshold-deg-s', type=float, default=1.0,
        help='Velocity (on the commanded signal) above which the axis is considered "moving" rather than '
             '"idle", used to segment ramps (default: 1.0 deg/s).')
    parser.add_argument(
        '--min-ramp-peak-deg', type=float, default=1.0,
        help='Ignore detected ramps that never move at least this many degrees from their baseline - '
             'filters out noise blips rather than real moves (default: 1.0 deg).')
    args = parser.parse_args()

    ns_prefix = f'/{args.ns}' if args.ns else ''
    base = f'{ns_prefix}/{args.hw_node}'
    sent_topic = f'{base}/joint_command_sent'
    feedback_topic = f'{base}/joint_feedback'

    print(f'Reading bag: {args.bag_path}')
    data = read_bag(args.bag_path)

    sent_samples = data.get(sent_topic, [])
    feedback_samples = data.get(feedback_topic, [])

    if not sent_samples or not feedback_samples:
        available = ', '.join(sorted(data.keys())) or '(none)'
        raise SystemExit(
            f"No messages found on '{sent_topic}' or '{feedback_topic}'.\n"
            f'Topics present in this bag: {available}\n'
            "Check the hardware component's node name/namespace with `ros2 topic list` "
            'and pass --ns/--hw-node if they differ.')

    axes = args.axis or AXIS_NAMES
    for axis_label in axes:
        index = AXIS_NAMES.index(axis_label)
        report_axis(sent_samples, feedback_samples, index, axis_label, args.n_samples, args.seed,
                    args.velocity_threshold_deg_s, args.min_ramp_peak_deg)


if __name__ == '__main__':
    main()
