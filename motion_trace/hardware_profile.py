import os
import re
from dataclasses import dataclass, field

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Where the analyzer's knowledge of a particular hardware lives: which topics
# carry which pipeline checkpoints, which message field holds the positions,
# and what to call each axis. Everything else (delay measurement, jitter,
# shift-check, ramp stats) only ever sees a profile's generic "command" and
# "feedback" signals, so supporting new hardware means writing a YAML file in
# profiles/, not touching the analysis code. See profiles/ynx.yaml for the
# annotated format.

PACKAGE_NAME = 'ynx_motion_analyzer'
DEFAULT_PROFILE = 'ynx'


@dataclass
class Signal:
    key: str
    topic: str
    field: str = 'position'
    label: str = ''
    color: str = None
    linestyle: str = '-'


@dataclass
class HardwareProfile:
    name: str
    signals: list
    command: str
    feedback: str
    params: dict = field(default_factory=dict)
    axis_names: list = None
    description: str = ''

    def signal(self, key):
        return next(s for s in self.signals if s.key == key)

    @property
    def command_signal(self):
        return self.signal(self.command)

    @property
    def feedback_signal(self):
        return self.signal(self.feedback)

    def topics(self):
        # Unique, in signal order - several signals may share one topic (e.g.
        # controller_state's reference and feedback).
        return list(dict.fromkeys(s.topic for s in self.signals))


def _profile_search_dirs():
    dirs = []
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(get_package_share_directory(PACKAGE_NAME), 'profiles'))
    except Exception:
        pass
    # Source tree, for running straight from the checkout (python3 -m ...).
    dirs.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'profiles'))
    return dirs


def available_profiles():
    names = set()
    for d in _profile_search_dirs():
        if os.path.isdir(d):
            names.update(os.path.splitext(f)[0] for f in os.listdir(d) if f.endswith('.yaml'))
    return sorted(names)


def find_profile_path(name_or_path):
    if os.path.isfile(name_or_path):
        return name_or_path
    for d in _profile_search_dirs():
        candidate = os.path.join(d, f'{name_or_path}.yaml')
        if os.path.isfile(candidate):
            return candidate
    raise SystemExit(f"Unknown hardware profile '{name_or_path}'. Built-in profiles: "
                     f"{', '.join(available_profiles()) or '(none found)'} - or pass a path to a profile YAML.")


def _expand_topic(template, params):
    try:
        topic = template.format(**params)
    except KeyError as e:
        raise SystemExit(f"Topic '{template}' uses placeholder {e} with no value - add it under the "
                         "profile's 'params:' or pass --set KEY=VALUE.")
    # An empty placeholder (e.g. no namespace) leaves '//' behind - collapse it.
    topic = re.sub('/+', '/', topic)
    return topic if topic.startswith('/') else '/' + topic


def load_profile(name_or_path, overrides=None):
    path = find_profile_path(name_or_path)
    with open(path) as f:
        raw = yaml.safe_load(f)

    params = {k: '' if v is None else str(v) for k, v in (raw.get('params') or {}).items()}
    params.update({k: v for k, v in (overrides or {}).items() if v is not None})

    signals = [Signal(**s) for s in raw['signals']]
    for s in signals:
        s.topic = _expand_topic(s.topic, params)
        s.label = s.label or s.key

    profile = HardwareProfile(
        name=raw.get('name', os.path.splitext(os.path.basename(path))[0]),
        signals=signals,
        command=raw['command'],
        feedback=raw['feedback'],
        params=params,
        axis_names=raw.get('axis_names'),
        description=raw.get('description', ''),
    )
    keys = {s.key for s in signals}
    for role in ('command', 'feedback'):
        if getattr(profile, role) not in keys:
            raise SystemExit(f"Profile '{path}': {role} '{getattr(profile, role)}' is not one of its signals "
                             f"({', '.join(sorted(keys))}).")
    return profile


def add_profile_args(parser):
    group = parser.add_argument_group('hardware profile')
    group.add_argument(
        '--profile', default=DEFAULT_PROFILE,
        help=f"Hardware profile: a built-in name ({', '.join(available_profiles())}) or a path to a "
             f"profile YAML. Defines which topics are recorded/analyzed and how axes are named "
             f"(default: {DEFAULT_PROFILE}).")
    group.add_argument(
        '--ns', default=None,
        help="Namespace the hardware was launched under (the profile's {ns} placeholder, e.g. bringup's "
             "ns:= argument). Pass '' for no namespace. Default: the profile's own default.")
    group.add_argument(
        '--hw-node', default=None,
        help="Hardware component's node name (the profile's {hw_node} placeholder, if it uses one). "
             "Default: the profile's own default.")
    group.add_argument(
        '--set', action='append', default=[], metavar='KEY=VALUE',
        help='Override any other topic placeholder the profile defines, e.g. '
             '--set controller=my_trajectory_controller (repeatable).')


def profile_from_args(args):
    overrides = {'ns': args.ns, 'hw_node': args.hw_node}
    for item in args.set:
        key, sep, value = item.partition('=')
        if not sep:
            raise SystemExit(f"--set expects KEY=VALUE, got '{item}'.")
        overrides[key] = value
    return load_profile(args.profile, overrides)


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
        # header.stamp (not bag arrival time) - it reflects the moment the
        # underlying value was actually captured/sent, not when the message
        # happened to be published/delivered, which matters once a topic's
        # value can come from a background cache (e.g. ynx's joint_feedback).
        # Falls back to arrival time only for messages with no usable stamp.
        header = getattr(msg, 'header', None)
        if header is not None and (header.stamp.sec or header.stamp.nanosec):
            stamp_s = header.stamp.sec + header.stamp.nanosec / 1e9
        else:
            stamp_s = t_ns / 1e9
        data.setdefault(topic, []).append((stamp_s, msg))
    return data


def _joint_names(msg):
    # sensor_msgs/JointState uses `name`, control_msgs' controller states use `joint_names`.
    for attr in ('name', 'joint_names'):
        names = getattr(msg, attr, None)
        if names:
            return list(names)
    return []


def _get_field(msg, dotted):
    for part in dotted.split('.'):
        msg = getattr(msg, part)
    return msg


@dataclass
class Axis:
    label: str
    joint: str   # joint name as it appears in the messages ('' if they carry none)
    index: int   # fallback for messages that carry no joint names


def resolve_axes(profile, data):
    # Axes come from whatever joints the command signal actually carries, not
    # a hardcoded count - so a 7-DOF arm or a 2-axis gantry just works. Labels
    # use the profile's axis_names when it lists exactly one per joint.
    samples = data.get(profile.command_signal.topic) or data.get(profile.feedback_signal.topic) or []
    if not samples:
        return []
    first = samples[0][1]
    joints = _joint_names(first)
    n = len(joints) or len(_get_field(first, profile.command_signal.field))
    labels = profile.axis_names if profile.axis_names and len(profile.axis_names) == n else None
    if labels is None:
        labels = joints if joints else [f'joint_{i + 1}' for i in range(n)]
    return [Axis(label=labels[i], joint=joints[i] if joints else '', index=i) for i in range(n)]


def select_axes(all_axes, requested):
    if not requested:
        return all_axes
    by_name = {}
    for a in all_axes:
        by_name[a.label] = a
        if a.joint:
            by_name[a.joint] = a
    missing = [r for r in requested if r not in by_name]
    if missing:
        raise SystemExit(f"Unknown axis {', '.join(missing)}. Available: "
                         f"{', '.join(a.label for a in all_axes)}.")
    return [by_name[r] for r in requested]


def extract_series(data, signal, axis):
    # One joint's (times, positions) from one signal, looked up by joint name
    # when the messages carry names (so topics that order joints differently
    # still line up), by index otherwise.
    times, positions = [], []
    cached_names, idx = None, None
    for t, msg in data.get(signal.topic, []):
        values = _get_field(msg, signal.field)
        names = _joint_names(msg)
        if names != cached_names:
            cached_names = names
            if axis.joint and names:
                idx = names.index(axis.joint) if axis.joint in names else None
            else:
                idx = axis.index
        if idx is None or idx >= len(values):
            continue
        times.append(t)
        positions.append(values[idx])
    return np.array(times), np.array(positions)


def require_topics(profile, data, keys):
    # Every tool needs at least its core signals present; fail with a message
    # that points at the actual cause (wrong profile / namespace) instead of
    # silently producing empty plots.
    signals = [profile.signal(k) for k in keys]
    if all(data.get(s.topic) for s in signals):
        return
    missing = [s.topic for s in signals if not data.get(s.topic)]
    available = ', '.join(sorted(data.keys())) or '(none)'
    raise SystemExit(
        f"Profile '{profile.name}': no messages on {', '.join(dict.fromkeys(missing))}.\n"
        f'Topics present in this bag: {available}\n'
        'Pick the profile matching the hardware this bag was recorded from (--profile), and '
        '--ns/--hw-node/--set if its topics live somewhere else.')
