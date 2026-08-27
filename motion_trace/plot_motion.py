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

# Fixed colors, shared by every plot function so a given signal is always the
# same color regardless of which panel/plot it appears in.
SENT_COLOR = 'tab:blue'
ACK_COLOR = 'tab:orange'
ACU_SETPOINT_COLOR = 'tab:purple'
FEEDBACK_COLOR = 'tab:green'


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    data = {}
    while reader.has_next():
        topic, raw, _t_ns = reader.read_next()
        if topic not in type_map:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_type)
        # header.stamp (not bag arrival time) - it reflects the moment the
        # underlying value was actually captured/sent, not when the message
        # happened to be published/delivered, which matters once a topic's
        # value can come from a background cache (joint_feedback/joint_command_acu).
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        data.setdefault(topic, []).append((stamp_s, msg))
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
              threshold_degs=(1.0,), show=False):
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

    # One full-move panel plus one zoomed-transition panel per threshold -
    # e.g. 3 thresholds = 4 panels total, so each threshold's crossing gets
    # its own clean, uncluttered view instead of sharing one panel.
    n_panels = 1 + len(threshold_degs)
    fig, axes = plt.subplots(n_panels, 1, figsize=(16, 4 * n_panels))
    ax_pos = axes[0]
    ax_zooms = axes[1:]

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
    ax_pos.set_xlabel('Time [s]')

    # Scan from the start of the recording to find when the command actually
    # starts sending a movement - this only establishes a clean, shared baseline
    # (both signals' own value at that moment), it isn't itself one of the two
    # measured points, and it's the same for every threshold since it's just
    # "when did the move begin." From that baseline, find when `sent` reaches
    # each threshold and, separately, when `feedback` reaches the same
    # threshold - the delay between those two "reached N deg" moments is the
    # number each zoomed panel exists to show.
    command_start_s = find_command_start_time(sent_t, sent_p)
    sent_baseline = np.interp(command_start_s, sent_t, sent_p) if command_start_s is not None else None
    fb_baseline = np.interp(command_start_s, fb_t, fb_p) if command_start_s is not None else None

    delays_ms = []
    for threshold_deg, ax_zoom in zip(threshold_degs, ax_zooms):
        plot_four_phases(ax_zoom)

        sent_threshold_s = None
        feedback_threshold_s = None
        if command_start_s is not None:
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
            ax_zoom.set_title(f'Zoomed transition @ {threshold_deg:g} deg')

            # Mark the two "reached threshold_deg" events with a color-matched
            # vertical line each.
            for cross_t, color in ((sent_threshold_s, SENT_COLOR), (feedback_threshold_s, FEEDBACK_COLOR)):
                if cross_t is not None and window_s[0] <= cross_t <= window_s[1]:
                    ax_zoom.axvline(cross_t, color=color, linestyle='-', linewidth=1, alpha=0.6)

            # Same position (both signals `threshold_deg` past the shared
            # baseline), two different times: a horizontal red dotted line
            # connects the two crossing points at that shared level, so "same
            # position, this many ms apart" is explicit instead of something
            # you have to infer from two separate vertical marks.
            direction = 1.0 if np.interp(sent_threshold_s, sent_t, sent_p) >= sent_baseline else -1.0
            level_deg = np.degrees(sent_baseline) + direction * threshold_deg
            delay_ms = (feedback_threshold_s - sent_threshold_s) * 1000
            delays_ms.append(delay_ms)
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
            ax_zoom.set_title(f'Zoomed transition @ {threshold_deg:g} deg (unavailable)')

    if delays_ms:
        avg_delay_ms = float(np.mean(delays_ms))
        thresholds_str = ', '.join(f'{t:g}' for t in threshold_degs)
        ax_pos.set_title(f'Axis: {axis_label} - full move  |  avg delay across '
                          f'{len(delays_ms)}/{len(threshold_degs)} thresholds ({thresholds_str} deg): '
                          f'{avg_delay_ms:+.2f} ms')
        print(f'  Axis {axis_label}: delays at {thresholds_str} deg = '
              f'{[f"{d:+.2f}" for d in delays_ms]} ms  ->  average {avg_delay_ms:+.2f} ms')
    else:
        ax_pos.set_title(f'Axis: {axis_label} - full move')

    fig.tight_layout()
    save_path = os.path.join(save_dir, f'{axis_label}.png')
    fig.savefig(save_path)
    print(f'  Saved {save_path}')
    if not show:
        plt.close(fig)


def _stats_text(label, values):
    if len(values) == 0:
        return f'{label}: no samples'
    return (f'{label}: n={len(values)}  mean={np.mean(values):.2f}  '
            f'std={np.std(values):.2f}  max={np.max(values):.2f}')


def plot_jitter(sent_samples, feedback_samples, index, axis_label, save_dir, show=False):
    # Jitter isn't visible on a position-vs-time curve at the timescale
    # plot_axis operates at (a multi-second move, or a millisecond zoom
    # confined to a single transition). It shows up as irregularity in the
    # *size* of each per-sample step - so plot that directly, signed and
    # across the whole recording, instead of position itself.
    #
    # Uses joint_command_sent (the RT loop's own intended setpoint, published
    # every write() cycle) rather than joint_command (the ACU-ack signal,
    # published only when the background sender thread's SetIncrementMove call
    # happens to complete). The ACU-ack signal's step size is downstream of
    # write-side coalescing/RTT variance - it measures how lumpy our sends
    # were, not what was actually intended - so it's the wrong signal for
    # asking "does feedback track the commanded trajectory smoothly."
    sent_t, sent_p = extract_series_by_index(sent_samples, index)
    fb_t, fb_p = extract_series_by_index(feedback_samples, index)

    if len(sent_t) < 2 and len(fb_t) < 2:
        print(f"  Skipping axis '{axis_label}' jitter: not enough samples on joint_command_sent or joint_feedback.")
        return

    t0 = min(list(sent_t[:1]) + list(fb_t[:1]))
    sent_t = sent_t - t0
    fb_t = fb_t - t0

    # Signed, not abs - so the trace follows the actual move's wave shape
    # (rises on the outbound leg, dips negative on the way back) instead of
    # folding everything into positive-only magnitude, making it easy to see
    # which leg of the move a given jitter spike happened on.
    sent_dt_s = np.diff(sent_t) if len(sent_t) > 1 else np.array([])
    fb_dt_s = np.diff(fb_t) if len(fb_t) > 1 else np.array([])
    sent_step_deg = np.diff(np.degrees(sent_p)) if len(sent_p) > 1 else np.array([])
    fb_step_deg = np.diff(np.degrees(fb_p)) if len(fb_p) > 1 else np.array([])

    # Velocity (deg/s), not raw step size - a raw per-sample step is a
    # function of how often you happened to sample, not how fast the arm
    # moved, so it isn't comparable across recordings made at different
    # control-loop rates (e.g. the old ~158Hz synchronous loop vs the current
    # ~500Hz async loop - the same true speed produces a ~3x smaller raw step
    # on the faster loop for no reason other than sampling more often).
    # Dividing by dt cancels that out.
    with np.errstate(divide='ignore', invalid='ignore'):
        sent_vel_deg_s = np.where(sent_dt_s > 0, sent_step_deg / sent_dt_s, 0.0) if len(sent_dt_s) else np.array([])
        fb_vel_deg_s = np.where(fb_dt_s > 0, fb_step_deg / fb_dt_s, 0.0) if len(fb_dt_s) else np.array([])
    sent_t_step = sent_t[1:]
    fb_t_step = fb_t[1:]

    # An exact-zero step isn't "no motion" - it's often a duplicate frame:
    # joint_feedback's cache only refreshes at the stream's ~250Hz sample rate
    # while read() re-publishes it every ~2ms/500Hz, so roughly every other
    # feedback sample just re-reports the previous cycle's value. Drop those
    # so the plot shows only points where the position actually changed.
    if len(sent_step_deg):
        sent_mask = sent_step_deg != 0.0
        sent_t_step, sent_vel_deg_s = sent_t_step[sent_mask], sent_vel_deg_s[sent_mask]
    if len(fb_step_deg):
        fb_mask = fb_step_deg != 0.0
        fb_t_step, fb_vel_deg_s = fb_t_step[fb_mask], fb_vel_deg_s[fb_mask]

    fig, ax_step = plt.subplots(1, 1, figsize=(16, 6))

    if len(sent_vel_deg_s):
        ax_step.plot(sent_t_step, sent_vel_deg_s, '.-', color=SENT_COLOR, label='joint_command_sent',
                     markersize=3, linewidth=0.8)
    if len(fb_vel_deg_s):
        ax_step.plot(fb_t_step, fb_vel_deg_s, '.-', color=FEEDBACK_COLOR, label='joint_feedback',
                     markersize=3, linewidth=0.8)
    ax_step.axhline(0, color='black', linewidth=0.6, alpha=0.4)
    ax_step.set_ylabel('Velocity [deg/s] (signed)')
    ax_step.set_xlabel('Time [s]')
    ax_step.set_title(f'Axis: {axis_label} - per-sample velocity (step/dt, comparable across control-loop rates)')
    ax_step.legend(loc='upper right')
    ax_step.grid(True, alpha=0.3)

    # Clip the y-axis to the typical range - a rare, large, already-understood
    # outlier (e.g. a joint_trajectory_controller interpolation artifact at a
    # zero-velocity segment boundary, amplified by dividing by a small dt)
    # would otherwise stretch the axis and drown out the much smaller "normal"
    # jitter this plot exists to show. The point itself is still in the data
    # and still counted in the stats box below - only the view is clipped.
    combined = np.concatenate([a for a in (sent_vel_deg_s, fb_vel_deg_s) if len(a)]) \
        if (len(sent_vel_deg_s) or len(fb_vel_deg_s)) else np.array([0.0])
    lo, hi = np.percentile(combined, [1, 99])
    pad = max((hi - lo) * 0.2, 1.0)
    ax_step.set_ylim(lo - pad, hi + pad)
    # Stats reported on magnitude even though the plotted line is signed -
    # "typical speed" reads more naturally as a magnitude than a signed value
    # that trends toward ~0 on a back-and-forth move.
    ax_step.text(0.02, 0.95,
                 _stats_text('sent |vel|', np.abs(sent_vel_deg_s)) + '\n' + _stats_text('feedback |vel|', np.abs(fb_vel_deg_s)),
                 transform=ax_step.transAxes, ha='left', va='top', fontsize=8, family='monospace',
                 bbox=dict(boxstyle='round,pad=0.4', fc='white', ec='gray', alpha=0.9))

    fig.tight_layout()
    save_path = os.path.join(save_dir, f'{axis_label}_jitter.png')
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
        '--threshold-deg', type=float, action='append',
        help='The command-start -> feedback delay is measured from when the commanded position '
             'starts moving to when feedback first moves this many degrees away from its own value '
             "at that moment - e.g. 'from 0 deg to 1 deg, what's the latency'. Repeatable "
             '(e.g. --threshold-deg 1 --threshold-deg 5 --threshold-deg 10) to measure the delay at '
             'several points along the same move and see one zoomed-transition subplot per threshold '
             'plus their average, instead of trusting a single point. Default: 1.0.')
    parser.add_argument(
        '--jitter', action='store_true',
        help='Also save <axis>_jitter.png per axis: signed per-sample velocity (derived from '
             'de-duplicated position + real elapsed dt) for joint_command_sent vs joint_feedback, to check '
             'whether feedback tracks the intended trajectory smoothly - detail a position-vs-time plot is '
             'too coarse to show.')
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
    threshold_degs = args.threshold_deg or [1.0]

    save_dir = os.path.join(args.bag_path, 'plot')
    os.makedirs(save_dir, exist_ok=True)

    for axis_label in axes:
        index = AXIS_NAMES.index(axis_label)
        plot_axis(sent_samples, command_samples, acu_samples, feedback_samples, index, axis_label, save_dir,
                  threshold_degs=threshold_degs, show=args.show)
        if args.jitter:
            plot_jitter(sent_samples, feedback_samples, index, axis_label, save_dir, show=args.show)

    if args.show:
        print('Opening interactive window(s) - close them (or Ctrl+C) to exit.')
        plt.show()


if __name__ == '__main__':
    main()
