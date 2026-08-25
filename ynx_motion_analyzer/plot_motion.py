import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Standard Yaskawa MOTOMAN 6-axis naming, in the order the joints are defined
# in nex10.ros2_control_macro.xacro (joint_1..joint_6).
AXIS_NAMES = ['S', 'L', 'U', 'R', 'B', 'T']


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = {}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in type_map:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_type)
        data.setdefault(topic, []).append((t_ns / 1e9, msg))
    return data


def extract_series_by_index(samples, index):
    times, positions = [], []
    for t, msg in samples:
        if index >= len(msg.position):
            continue
        times.append(t)
        positions.append(msg.position[index])
    return np.array(times), np.array(positions)


def _interpolated_crossing(t0, t1, p0, p1, target):
    if p1 == p0:
        return t1
    return t0 + (target - p0) / (p1 - p0) * (t1 - t0)


def find_command_start_time(sent_t, sent_p, epsilon_deg=0.01):
    # Scan the commanded position from the start of the recording, tracking
    # whether it has moved away from its initial (idle) value yet. Returns the
    # (sub-sample interpolated) time it first does, or None if it never moves.
    if len(sent_p) < 2:
        return None
    epsilon_rad = np.radians(epsilon_deg)
    baseline = sent_p[0]
    for i in range(1, len(sent_p)):
        delta = sent_p[i] - baseline
        if abs(delta) > epsilon_rad:
            target = baseline + np.sign(delta) * epsilon_rad
            return _interpolated_crossing(sent_t[i - 1], sent_t[i], sent_p[i - 1], sent_p[i], target)
    return None


def find_signal_threshold_time(t, p, after_time, baseline, threshold_deg=1.0):
    # From `after_time` onward, scan `p` to find when it has moved more than
    # `threshold_deg` away from `baseline` - a reference value shared by both
    # the sent and feedback scans (each signal's own value at `after_time`, the
    # detected command-start time), not each signal's own array-start sample.
    # That's what makes "command reaches 1 deg" and "feedback reaches 1 deg"
    # directly comparable: both are measured from the same moment in time.
    if len(p) < 2:
        return None
    threshold_rad = np.radians(threshold_deg)
    mask = t >= after_time
    t_seg = np.concatenate(([after_time], t[mask]))
    p_seg = np.concatenate(([baseline], p[mask]))
    for i in range(1, len(p_seg)):
        delta = p_seg[i] - baseline
        if abs(delta) > threshold_rad:
            target = baseline + np.sign(delta) * threshold_rad
            return _interpolated_crossing(t_seg[i - 1], t_seg[i], p_seg[i - 1], p_seg[i], target)
    return None


def plot_axis(sent_samples, command_samples, acu_samples, feedback_samples, index, axis_label, save_dir,
              threshold_deg=1.0, show=False):
    sent_t, sent_p = extract_series_by_index(sent_samples, index)
    cmd_t, cmd_p = extract_series_by_index(command_samples, index)
    acu_t, acu_p = extract_series_by_index(acu_samples, index)
    fb_t, fb_p = extract_series_by_index(feedback_samples, index)

    if len(sent_t) == 0 and len(cmd_t) == 0 and len(acu_t) == 0 and len(fb_t) == 0:
        print(f"  Skipping axis '{axis_label}': no samples found on any topic.")
        return

    t0 = min(list(sent_t[:1]) + list(cmd_t[:1]) + list(acu_t[:1]) + list(fb_t[:1]))
    sent_t = sent_t - t0
    cmd_t = cmd_t - t0
    acu_t = acu_t - t0
    fb_t = fb_t - t0

    fig, (ax_pos, ax_zoom) = plt.subplots(2, 1, figsize=(10, 8))

    # Fixed colors, used consistently in both panels (and for the transition
    # markers below) regardless of which lines are actually plotted where -
    # matplotlib's automatic color cycle would otherwise assign a line a
    # different color in each panel if the two panels don't plot the exact
    # same set of lines.
    SENT_COLOR = 'tab:blue'
    ACK_COLOR = 'tab:orange'
    ACU_SETPOINT_COLOR = 'tab:purple'
    FEEDBACK_COLOR = 'tab:green'

    def plot_four_phases(ax):
        # The four checkpoints of the pipeline, in the order they happen:
        # ros2_control sends the setpoint -> the ACU acknowledges the gRPC call
        # -> the ACU's own internal interpolator/setpoint (GetAxesPos) tracks
        # toward it -> the arm's encoder (GetFeedbackAxesPos) shows it physically.
        ax.plot(sent_t, np.degrees(sent_p), label='commanded (sent)', linewidth=1.5, color=SENT_COLOR)
        ax.plot(cmd_t, np.degrees(cmd_p), label='commanded (ACU ack)', linewidth=1.5, linestyle=':', color=ACK_COLOR)
        ax.plot(acu_t, np.degrees(acu_p), label='ACU internal setpoint', linewidth=1.5, linestyle='-.',
                color=ACU_SETPOINT_COLOR)
        ax.plot(fb_t, np.degrees(fb_p), label='feedback (real)', linewidth=1.5, linestyle='--', color=FEEDBACK_COLOR)
        ax.set_ylabel('Position [deg]')
        ax.legend(loc='best')
        ax.grid(True)

    plot_four_phases(ax_pos)
    ax_pos.set_title(f'Axis: {axis_label} - full move')
    ax_pos.set_xlabel('Time [s]')

    plot_four_phases(ax_zoom)

    # Scan from the start of the recording to find when the command actually
    # starts sending a movement - this only establishes a clean, shared baseline
    # (both signals' own value at that moment), it isn't itself one of the two
    # measured points. From that baseline, find when `sent` reaches
    # `threshold_deg` and, separately, when `feedback` reaches the same
    # `threshold_deg` - the delay between those two "reached 1 deg" moments is
    # the number this view exists to show.
    command_start_s = find_command_start_time(sent_t, sent_p)
    sent_threshold_s = None
    feedback_threshold_s = None
    if command_start_s is not None:
        sent_baseline = np.interp(command_start_s, sent_t, sent_p)
        fb_baseline = np.interp(command_start_s, fb_t, fb_p)
        sent_threshold_s = find_signal_threshold_time(sent_t, sent_p, command_start_s, sent_baseline, threshold_deg)
        feedback_threshold_s = find_signal_threshold_time(fb_t, fb_p, command_start_s, fb_baseline, threshold_deg)

    if sent_threshold_s is not None and feedback_threshold_s is not None:
        margin_s = max(0.01, (feedback_threshold_s - sent_threshold_s) * 0.3)
        window_s = (sent_threshold_s - margin_s, feedback_threshold_s + margin_s)
    else:
        window_s = None

    if window_s is not None:
        ax_zoom.set_xlim(window_s)
        # Re-tick in milliseconds relative to the window start, so the axis reads
        # like a latency measurement instead of a fraction of a second.
        ticks_s = np.linspace(window_s[0], window_s[1], 6)
        ax_zoom.set_xticks(ticks_s)
        ax_zoom.set_xticklabels([f'{(t - window_s[0]) * 1000:.0f}' for t in ticks_s])
        ax_zoom.set_xlabel(f'Time [ms] (relative to {window_s[0]:.3f} s)')
        ax_zoom.set_title('Zoomed transition')

        # Mark the two "reached threshold_deg" events with a color-matched
        # vertical line each.
        for cross_t, color in ((sent_threshold_s, SENT_COLOR), (feedback_threshold_s, FEEDBACK_COLOR)):
            if cross_t is not None and window_s[0] <= cross_t <= window_s[1]:
                ax_zoom.axvline(cross_t, color=color, linestyle='-', linewidth=1, alpha=0.6)

        if sent_threshold_s is not None and feedback_threshold_s is not None:
            # Same position (both signals `threshold_deg` past the shared
            # baseline), two different times: a horizontal red dotted line
            # connects the two crossing points at that shared level, so "same
            # position, this many ms apart" is explicit instead of something
            # you have to infer from two separate vertical marks.
            direction = 1.0 if np.interp(sent_threshold_s, sent_t, sent_p) >= sent_baseline else -1.0
            level_deg = np.degrees(sent_baseline) + direction * threshold_deg
            delay_ms = (feedback_threshold_s - sent_threshold_s) * 1000
            ax_zoom.plot([sent_threshold_s, feedback_threshold_s], [level_deg, level_deg],
                         color='red', linestyle=':', linewidth=1.8, marker='o', markersize=5, zorder=5)
            ax_zoom.annotate(f'{delay_ms:+.2f} ms', xy=((sent_threshold_s + feedback_threshold_s) / 2, level_deg),
                              xytext=(0, 8), textcoords='offset points', ha='center',
                              color='red', fontsize=9, fontweight='bold')

            ax_zoom.text(
                0.02, 0.95, f'command @ {threshold_deg:g} deg -> feedback @ {threshold_deg:g} deg: {delay_ms:+.2f} ms',
                transform=ax_zoom.transAxes, ha='left', va='top', fontsize=9, family='monospace',
                bbox=dict(boxstyle='round,pad=0.4', fc='white', ec='gray', alpha=0.9))
    else:
        ax_zoom.text(0.5, 0.5, f'Command or feedback never reached {threshold_deg:g} deg on this axis',
                     ha='center', va='center', transform=ax_zoom.transAxes, wrap=True)
        ax_zoom.set_xlabel('Time [s]')
        ax_zoom.set_title('Zoomed transition (unavailable)')

    fig.tight_layout()
    save_path = os.path.join(save_dir, f'{axis_label}.png')
    fig.savefig(save_path)
    print(f'  Saved {save_path}')
    if not show:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Plot all four checkpoints - commanded-sent, commanded-ACU-ack, the ACU's own "
                    "internal setpoint, and feedback ('real') - of joint trajectories from a bag "
                    "recorded from ynx_hardware_interface's ~/joint_command_sent, ~/joint_command, "
                    '~/joint_command_acu, and ~/joint_feedback topics. Each axis\'s PNG has a full-move '
                    'overview on top and a millisecond-scale zoomed transition on the bottom. '
                    "Saves one PNG per axis (S/L/U/R/B/T) into '<bag_path>/plot/'.")
    parser.add_argument('bag_path', help='Path to the rosbag2 directory (the -o used with record_motion)')
    parser.add_argument(
        '--ns', default='',
        help="The bringup launch's 'ns' argument used when the bag was recorded, if any (e.g. 'nex10'). "
             'Ignored if the --*-topic flags are given explicitly.')
    parser.add_argument(
        '--hw-node', default='nex10',
        help="The hardware component's own node name (hardcoded 'nex10' in the xacro, independent of --ns).")
    parser.add_argument(
        '--sent-topic', default=None,
        help='Full topic name override. Default: built from --ns/--hw-node, e.g. /<ns>/<hw-node>/joint_command_sent.')
    parser.add_argument(
        '--command-topic', default=None,
        help='Full topic name override. Default: built from --ns/--hw-node, e.g. /<ns>/<hw-node>/joint_command.')
    parser.add_argument(
        '--acu-topic', default=None,
        help='Full topic name override. Default: built from --ns/--hw-node, e.g. /<ns>/<hw-node>/joint_command_acu.')
    parser.add_argument(
        '--feedback-topic', default=None,
        help='Full topic name override. Default: built from --ns/--hw-node, e.g. /<ns>/<hw-node>/joint_feedback.')
    parser.add_argument(
        '--axis', action='append', choices=AXIS_NAMES,
        help='Axis to plot (repeatable, e.g. --axis S --axis T). Default: all six axes.')
    parser.add_argument(
        '--threshold-deg', type=float, default=1.0,
        help='The command-start -> feedback delay is measured from when the commanded position '
             'starts moving to when feedback first moves this many degrees away from its own value '
             "at that moment (default: 1.0) - e.g. 'from 0 deg to 1 deg, what's the latency'.")
    parser.add_argument(
        '--show', action='store_true',
        help='Also open live, interactive matplotlib windows for every plot (in addition to still '
             "saving the PNGs). Use the toolbar's zoom/pan tool to inspect any time range down to "
             'individual samples - e.g. millisecond-scale detail on the latency gaps. Requires a '
             'working GUI backend/display (X11 forwarding, WSLg, etc.). Closes when you close the windows.')
    args = parser.parse_args()

    ns_prefix = f'/{args.ns}' if args.ns else ''
    base = f'{ns_prefix}/{args.hw_node}'
    sent_topic = args.sent_topic or f'{base}/joint_command_sent'
    command_topic = args.command_topic or f'{base}/joint_command'
    acu_topic = args.acu_topic or f'{base}/joint_command_acu'
    feedback_topic = args.feedback_topic or f'{base}/joint_feedback'

    print(f'Reading bag: {args.bag_path}')
    data = read_bag(args.bag_path)

    sent_samples = data.get(sent_topic, [])
    command_samples = data.get(command_topic, [])
    acu_samples = data.get(acu_topic, [])
    feedback_samples = data.get(feedback_topic, [])

    if not sent_samples and not command_samples and not acu_samples and not feedback_samples:
        available = ', '.join(sorted(data.keys())) or '(none)'
        raise SystemExit(
            f"No messages found on '{sent_topic}', '{command_topic}', '{acu_topic}', or '{feedback_topic}'.\n"
            f'Topics present in this bag: {available}\n'
            "Check the hardware component's node name/namespace with `ros2 topic list` "
            'and pass --ns/--hw-node or the --*-topic flags if they differ.')

    axes = args.axis or AXIS_NAMES

    save_dir = os.path.join(args.bag_path, 'plot')
    os.makedirs(save_dir, exist_ok=True)

    for axis_label in axes:
        index = AXIS_NAMES.index(axis_label)
        plot_axis(sent_samples, command_samples, acu_samples, feedback_samples, index, axis_label, save_dir,
                  threshold_deg=args.threshold_deg, show=args.show)

    if args.show:
        print('Opening interactive window(s) - close them (or Ctrl+C) to exit.')
        plt.show()


if __name__ == '__main__':
    main()
