import argparse
import os
import subprocess
import sys
from datetime import datetime

from motion_trace.hardware_profile import add_profile_args, profile_from_args

# All experiment recordings live here by default, not wherever the tool
# happens to be run from - a fixed, known location (not derived from
# __file__) since `ros2 run` always executes the installed copy, not the
# source tree, so relative-to-this-file paths would resolve under install/.
EXPERIMENT_DIR = os.path.expanduser('~/ros2_ws/src/ynx_motion_analyzer/experiment')


def main():
    parser = argparse.ArgumentParser(
        description="Record a hardware profile's topics (by default ynx_hardware_interface's per-cycle "
                    'joint_command_sent/joint_command/joint_command_acu/joint_feedback) to a rosbag2 for '
                    'later analysis with plot_motion.')
    add_profile_args(parser)
    parser.add_argument(
        '-o', '--output', default=None,
        help=f'Bag output directory or name (default: motion_bag_<timestamp>). A bare name (not an '
             f'absolute path) is placed under {EXPERIMENT_DIR} - pass an absolute path to override.')
    parser.add_argument(
        '--extra-topic', action='append', default=[],
        help='Additional topic to record (repeatable).')
    args = parser.parse_args()

    output_name = args.output or f"motion_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output = output_name if os.path.isabs(output_name) else os.path.join(EXPERIMENT_DIR, output_name)
    os.makedirs(EXPERIMENT_DIR, exist_ok=True)

    profile = profile_from_args(args)
    topics = profile.topics()
    topics.extend(args.extra_topic)

    cmd = ['ros2', 'bag', 'record', '-o', output] + topics
    print(f"Profile '{profile.name}': {profile.description}")
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
