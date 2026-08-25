import argparse
import subprocess
import sys
from datetime import datetime


def main():
    parser = argparse.ArgumentParser(
        description="Record ynx_hardware_interface's per-cycle joint_command_sent/joint_command/"
                    'joint_command_acu/joint_feedback topics to a rosbag2 for later analysis with '
                    'plot_motion.')
    parser.add_argument(
        '--ns', default='',
        help="The bringup launch's 'ns' argument, if any. Leave empty if launched without a namespace.")
    parser.add_argument(
        '--hw-node', default='nex10',
        help="The hardware component's own node name - hardcoded as 'nex10' in "
             'nex10.ros2_control_macro.xacro regardless of --ns. Change only if that xacro changes.')
    parser.add_argument(
        '-o', '--output', default=None,
        help='Bag output directory (default: motion_bag_<timestamp>)')
    parser.add_argument(
        '--extra-topic', action='append', default=[],
        help='Additional topic to record (repeatable).')
    args = parser.parse_args()

    output = args.output or f"motion_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # The hardware component's node (hardcoded name from the xacro) sits under
    # whatever outer namespace bringup was launched with, e.g. --ns nex10 gives
    # /nex10/nex10/joint_command - the two 'nex10's are two different things.
    ns_prefix = f'/{args.ns}' if args.ns else ''
    base = f'{ns_prefix}/{args.hw_node}'
    topics = [
        f'{base}/joint_command_sent',
        f'{base}/joint_command',
        f'{base}/joint_command_acu',
        f'{base}/joint_feedback',
    ]
    topics.extend(args.extra_topic)

    cmd = ['ros2', 'bag', 'record', '-o', output] + topics
    print('Running:', ' '.join(cmd))
    print(f"Recording to '{output}'. Run your move script now; press Ctrl+C here when the move is done.")

    process = subprocess.Popen(cmd)
    try:
        returncode = process.wait()
    except KeyboardInterrupt:
        # Ctrl+C also delivers SIGINT directly to `ros2 bag record` (same process
        # group), which shuts it down gracefully and flushes metadata.yaml itself -
        # just wait for that instead of killing it, or the bag can be left corrupt/empty.
        returncode = process.wait()
    sys.exit(returncode)


if __name__ == '__main__':
    main()
