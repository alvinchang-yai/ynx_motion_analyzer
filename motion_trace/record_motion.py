import argparse
import os
import subprocess
import sys
from datetime import datetime

from motion_trace.hardware_profile import PACKAGE_NAME, add_profile_args, is_hidden_topic, profile_from_args


def experiment_dir():
    # All recordings land in the package's source-tree experiment/ folder, not
    # wherever the tool happens to be run from. `ros2 run` executes the
    # installed copy, so locate the source tree from the install prefix
    # (<ws>/install/<pkg>/share/<pkg> -> <ws>/src/<pkg>). MOTION_ANALYZER_EXPERIMENT_DIR
    # overrides; ./experiment is the last resort.
    override = os.environ.get('MOTION_ANALYZER_EXPERIMENT_DIR')
    if override:
        return os.path.abspath(os.path.expanduser(override))
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(PACKAGE_NAME)
        ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        src_pkg = os.path.join(ws, 'src', PACKAGE_NAME)
        if os.path.isdir(src_pkg):
            return os.path.join(src_pkg, 'experiment')
    except Exception:
        pass
    return os.path.abspath('experiment')


def main():
    exp_dir = experiment_dir()
    parser = argparse.ArgumentParser(
        description="Record a hardware profile's topics (by default MotoROS2 driven by MoveIt: "
                    '/display_planned_path, the follow_joint_trajectory action status and feedback, '
                    '/joint_states and /robot_status) to a rosbag2 for analysis in the dashboard '
                    '(ros2 run ynx_motion_analyzer dashboard).')
    add_profile_args(parser)
    parser.add_argument(
        '-o', '--output', default=None,
        help=f'Bag output directory or name (default: motion_bag_<timestamp>). A relative path is '
             f'placed under {exp_dir} - pass an absolute path to override.')
    parser.add_argument(
        '--extra-topic', action='append', default=[],
        help='Additional topic to record (repeatable).')
    args = parser.parse_args()

    output_name = args.output or f"motion_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output = output_name if os.path.isabs(output_name) else os.path.join(exp_dir, output_name)
    os.makedirs(os.path.dirname(output), exist_ok=True)

    profile = profile_from_args(args)
    topics = list(dict.fromkeys(profile.record_topics() + args.extra_topic))

    cmd = ['ros2', 'bag', 'record', '-o', output]
    if any(is_hidden_topic(t) for t in topics):
        cmd.append('--include-hidden-topics')
    cmd += topics
    print(f"Profile '{profile.name}': {profile.description}")
    print('Running:', ' '.join(cmd))
    print(f"Recording to '{output}'. Run your move now; press Ctrl+C here when it's done.")

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
