import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

from motion_trace.plot_motion import (
    AXIS_NAMES, SENT_COLOR, FEEDBACK_COLOR, read_bag, extract_series_by_index,
    find_command_start_time, find_signal_threshold_time,
)


def plot_shift_check(sent_samples, feedback_samples, index, axis_label, save_dir,
                      threshold_deg=1.0, show=False):
    # Sanity check for "is feedback just a delayed copy of the commanded
    # trajectory, or does it actually diverge in shape": measure the
    # command->feedback delay the same way plot_motion does (at the
    # threshold-deg crossing), then shift feedback back in time by that
    # single delay value and see whether it lines up with the commanded
    # trajectory across the *whole* move, not just at the one point the
    # delay was measured from. A close match confirms the delay is close to
    # constant and feedback tracks command shape faithfully; a mismatch would
    # mean something beyond a pure time lag is going on (damping, overshoot,
    # a delay that varies with speed/direction, etc).
    sent_t, sent_p = extract_series_by_index(sent_samples, index)
    fb_t, fb_p = extract_series_by_index(feedback_samples, index)

    if len(sent_t) == 0 or len(fb_t) == 0:
        print(f"  Skipping axis '{axis_label}': no samples on joint_command_sent or joint_feedback.")
        return

    t0 = min(sent_t[0], fb_t[0])
    sent_t = sent_t - t0
    fb_t = fb_t - t0

    command_start_s = find_command_start_time(sent_t, sent_p)
    if command_start_s is None:
        print(f"  Skipping axis '{axis_label}': command never starts moving in this recording.")
        return
    sent_baseline = np.interp(command_start_s, sent_t, sent_p)
    fb_baseline = np.interp(command_start_s, fb_t, fb_p)
    sent_threshold_s = find_signal_threshold_time(sent_t, sent_p, command_start_s, sent_baseline, threshold_deg)
    feedback_threshold_s = find_signal_threshold_time(fb_t, fb_p, command_start_s, fb_baseline, threshold_deg)
    if sent_threshold_s is None or feedback_threshold_s is None:
        print(f"  Skipping axis '{axis_label}': command or feedback never reached {threshold_deg:g} deg.")
        return
    delay_s = feedback_threshold_s - sent_threshold_s

    fb_t_shifted = fb_t - delay_s

    # Quantify the match, not just eyeball it: interpolate feedback onto
    # sent's time grid (raw and shifted) within their overlapping time range,
    # and compare RMS error against the commanded position.
    mask = (sent_t >= max(fb_t.min(), fb_t_shifted.min())) & (sent_t <= min(fb_t.max(), fb_t_shifted.max()))
    fb_on_sent_grid_raw = np.interp(sent_t[mask], fb_t, fb_p)
    fb_on_sent_grid_shifted = np.interp(sent_t[mask], fb_t_shifted, fb_p)
    err_raw_deg = np.degrees(fb_on_sent_grid_raw) - np.degrees(sent_p[mask])
    err_shifted_deg = np.degrees(fb_on_sent_grid_shifted) - np.degrees(sent_p[mask])
    rms_raw = float(np.sqrt(np.mean(err_raw_deg ** 2))) if len(err_raw_deg) else float('nan')
    rms_shifted = float(np.sqrt(np.mean(err_shifted_deg ** 2))) if len(err_shifted_deg) else float('nan')

    fig, (ax_raw, ax_shifted) = plt.subplots(2, 1, figsize=(14, 9), sharex=True)

    ax_raw.plot(sent_t, np.degrees(sent_p), label='commanded (sent)', color=SENT_COLOR, linewidth=1.3)
    ax_raw.plot(fb_t, np.degrees(fb_p), label='feedback (real), unshifted', color=FEEDBACK_COLOR,
                linewidth=1.3, linestyle='--')
    ax_raw.set_ylabel('Position [deg]')
    ax_raw.set_title(f'Axis {axis_label}: current latency (unshifted) - RMS error {rms_raw:.3f} deg')
    ax_raw.legend(loc='best')
    ax_raw.grid(True, alpha=0.3)

    ax_shifted.plot(sent_t, np.degrees(sent_p), label='commanded (sent)', color=SENT_COLOR, linewidth=1.3)
    ax_shifted.plot(fb_t_shifted, np.degrees(fb_p), label=f'feedback (real), shifted back {delay_s * 1000:.1f} ms',
                    color=FEEDBACK_COLOR, linewidth=1.3, linestyle='--')
    ax_shifted.set_ylabel('Position [deg]')
    ax_shifted.set_xlabel('Time [s]')
    ax_shifted.set_title(f'Axis {axis_label}: feedback shifted back by the measured '
                          f'{delay_s * 1000:.1f} ms delay - RMS error {rms_shifted:.3f} deg')
    ax_shifted.legend(loc='best')
    ax_shifted.grid(True, alpha=0.3)

    fig.tight_layout()
    save_path = os.path.join(save_dir, f'{axis_label}_shift_check.png')
    fig.savefig(save_path)
    print(f'  Axis {axis_label}: delay={delay_s * 1000:.2f} ms  RMS unshifted={rms_raw:.3f} deg  '
          f'RMS shifted={rms_shifted:.3f} deg  -> Saved {save_path}')
    if not show:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description='Sanity-check whether joint_feedback is essentially a time-delayed copy of '
                    "joint_command_sent: measures the command->feedback delay the same way plot_motion "
                    'does, then shifts feedback back by that delay and overlays it against the commanded '
                    'trajectory across the whole move. Top panel: current (unshifted) latency. Bottom '
                    'panel: feedback shifted back by the measured delay, plus an RMS-error read for both.')
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
        '--threshold-deg', type=float, default=1.0,
        help='Degree threshold the command->feedback delay is measured at, same meaning as in '
             'plot_motion (default: 1.0).')
    parser.add_argument(
        '--show', action='store_true',
        help='Also open live, interactive matplotlib windows for every plot. Requires a working '
             'GUI backend/display.')
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

    save_dir = os.path.join(args.bag_path, 'plot')
    os.makedirs(save_dir, exist_ok=True)

    for axis_label in axes:
        index = AXIS_NAMES.index(axis_label)
        plot_shift_check(sent_samples, feedback_samples, index, axis_label, save_dir,
                          threshold_deg=args.threshold_deg, show=args.show)

    if args.show:
        print('Opening interactive window(s) - close them (or Ctrl+C) to exit.')
        plt.show()


if __name__ == '__main__':
    main()
