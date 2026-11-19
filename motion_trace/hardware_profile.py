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
# and what to call each axis. Everything else (the delay analysis behind the
# dashboard) only ever sees a profile's generic "command" and
# "feedback" signals, so supporting new hardware means writing a YAML file in
# profiles/, not touching the analysis code. See profiles/motoros2.yaml for
# the annotated format.

PACKAGE_NAME = 'ynx_motion_analyzer'
# Used by record_motion, and by the dashboard when a bag matches no profile
# (otherwise it picks the profile matching the bag's topics).
DEFAULT_PROFILE = 'motoros2'


@dataclass
class Signal:
    key: str
    topic: str
    field: str = 'position'
    label: str = ''
    # Optional dotted paths. Left unset, the joint-name list and the header
    # are looked up next to `field`, walking up towards the message root, so
    # nested layouts like an action's `feedback.desired.positions` (names and
    # header under `feedback`) need no extra configuration.
    names_field: str = None
    stamp_field: str = None
    # 'array' (default): `field` is an array of positions in every message.
    # 'trajectory': `field` holds planned trajectories (see trajectory_signal.py),
    # anchored in time by the action server's status (`anchor_topic`) and/or
    # feedback (`anchor_feedback_topic`) topics, and resampled at sample_rate_hz.
    type: str = 'array'
    anchor_topic: str = None
    anchor_feedback_topic: str = None
    sample_rate_hz: float = 250.0


@dataclass
class HardwareProfile:
    name: str
    signals: list
    command: str
    feedback: str
    params: dict = field(default_factory=dict)
    axis_names: list = None
    description: str = ''
    # Recorded for context (e.g. robot status) but not plotted or analyzed.
    extra_topics: list = field(default_factory=list)

    def signal(self, key):
        return next(s for s in self.signals if s.key == key)

    @property
    def command_signal(self):
        return self.signal(self.command)

    @property
    def feedback_signal(self):
        return self.signal(self.feedback)

    def topics(self):
        # Unique, in signal order, including the topics a trajectory signal is
        # anchored by. Several signals may share one topic (e.g. an action
        # feedback's actual and a trajectory's fallback anchor).
        topics = []
        for s in self.signals:
            topics += [s.topic, s.anchor_topic, s.anchor_feedback_topic]
        return list(dict.fromkeys(t for t in topics if t))

    def record_topics(self):
        return list(dict.fromkeys(self.topics() + self.extra_topics))


def is_hidden_topic(topic):
    # ROS hides topics with any name segment starting with '_' (e.g. an
    # action's /<name>/_action/feedback); ros2 bag only records them with
    # --include-hidden-topics.
    return any(part.startswith('_') for part in topic.split('/') if part)


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
    # An empty placeholder value leaves '//' behind - collapse it.
    topic = re.sub('/+', '/', topic)
    return topic if topic.startswith('/') else '/' + topic


def load_profile(name_or_path, overrides=None):
    path = find_profile_path(name_or_path)
    with open(path) as f:
        raw = yaml.safe_load(f)

    params = {k: '' if v is None else str(v) for k, v in (raw.get('params') or {}).items()}
    params.update({k: v for k, v in (overrides or {}).items() if v is not None})

    # color/linestyle styled the old matplotlib plots; ignored so older profiles still load.
    signals = [Signal(**{k: v for k, v in s.items() if k not in ('color', 'linestyle')})
               for s in raw['signals']]
    for s in signals:
        if s.type not in ('array', 'trajectory'):
            raise SystemExit(f"Profile '{path}': signal '{s.key}' has unknown type '{s.type}'.")
        s.topic = _expand_topic(s.topic, params)
        for attr in ('anchor_topic', 'anchor_feedback_topic'):
            if getattr(s, attr):
                setattr(s, attr, _expand_topic(getattr(s, attr), params))
        if s.type == 'trajectory' and not (s.anchor_topic or s.anchor_feedback_topic):
            raise SystemExit(f"Profile '{path}': trajectory signal '{s.key}' needs an anchor_topic "
                             'and/or anchor_feedback_topic to place it in time.')
        s.label = s.label or s.key

    profile = HardwareProfile(
        name=raw.get('name', os.path.splitext(os.path.basename(path))[0]),
        signals=signals,
        command=raw['command'],
        feedback=raw['feedback'],
        params=params,
        axis_names=raw.get('axis_names'),
        description=raw.get('description', ''),
        extra_topics=[_expand_topic(t, params) for t in raw.get('extra_topics') or []],
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
             f'profile YAML. Defines which topics are recorded and how axes are named '
             f'(default: {DEFAULT_PROFILE}).')
    group.add_argument(
        '--set', action='append', default=[], metavar='KEY=VALUE',
        help='Override a topic placeholder the profile defines, e.g. '
             '--set action=my_arm/follow_joint_trajectory (repeatable).')


def _overrides_from_args(args):
    overrides = {}
    for item in args.set:
        key, sep, value = item.partition('=')
        if not sep:
            raise SystemExit(f"--set expects KEY=VALUE, got '{item}'.")
        overrides[key] = value
    return overrides


def profile_from_args(args):
    return load_profile(args.profile or DEFAULT_PROFILE, _overrides_from_args(args))


def _open_reader(bag_path):
    if not os.path.exists(bag_path):
        raise SystemExit(f"Bag '{bag_path}' does not exist.")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path), rosbag2_py.ConverterOptions('', ''))
    return reader


def bag_topics(bag_path):
    # Topic names recorded in the bag - no messages are read. (Not via
    # rosbag2_py.Info: Humble's metadata parser rejects bags from newer distros.)
    return {t.name for t in _open_reader(bag_path).get_all_topics_and_types()}


def detect_profile(args, bag_topics):
    # The first built-in profile (DEFAULT_PROFILE tried first) whose command
    # and feedback topics are both in the bag, with any --set overrides
    # applied. Returns None if none match.
    overrides = _overrides_from_args(args)
    names = available_profiles()
    if DEFAULT_PROFILE in names:
        names.remove(DEFAULT_PROFILE)
        names.insert(0, DEFAULT_PROFILE)
    for name in names:
        profile = load_profile(name, overrides)
        if {profile.command_signal.topic, profile.feedback_signal.topic} <= bag_topics:
            return profile
    return None


def read_bag(bag_path, topics=None):
    # {topic: [(receive_time_s, msg), ...]} - only `topics` are deserialized
    # when given, so unrelated recorded topics never need their message
    # packages installed. Signal timestamps are taken from each message's
    # header later (see extract_series), not from the receive time here.
    reader = _open_reader(bag_path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics is not None:
        topics = [t for t in topics if t in type_map]
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    msg_types = {}
    data = {}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in type_map or (topics is not None and topic not in topics):
            continue
        if topic not in msg_types:
            msg_types[topic] = get_message(type_map[topic])
        data.setdefault(topic, []).append((t_ns / 1e9, deserialize_message(raw, msg_types[topic])))
    return data


def _get_field(msg, dotted):
    for part in dotted.split('.'):
        msg = getattr(msg, part)
    return msg


def _parents(msg, dotted):
    # The objects along `dotted`, deepest parent first, ending at the root.
    # e.g. 'feedback.desired.positions' -> [msg.feedback.desired, msg.feedback, msg]
    chain = [msg]
    for part in dotted.split('.')[:-1]:
        chain.append(getattr(chain[-1], part))
    return chain[::-1]


def _joint_names(msg, signal):
    if signal.names_field:
        return list(_get_field(msg, signal.names_field))
    # sensor_msgs/JointState uses `name`, trajectory/controller messages use `joint_names`.
    for obj in _parents(msg, signal.field):
        for attr in ('name', 'joint_names'):
            names = getattr(obj, attr, None)
            if names:
                return list(names)
    return []


def _stamp_s(msg, signal, receive_s):
    # header.stamp (not bag arrival time) - it reflects the moment the value
    # was captured on the controller, not when it happened to be delivered,
    # which matters when a value comes from a cache or a congested link.
    # Falls back to arrival time only when there's no usable stamp.
    if signal.stamp_field:
        candidates = [_get_field(msg, signal.stamp_field)]
    else:
        candidates = [getattr(obj, 'header', None) for obj in _parents(msg, signal.field)]
    for c in candidates:
        stamp = getattr(c, 'stamp', c)
        if stamp is not None and hasattr(stamp, 'sec') and (stamp.sec or stamp.nanosec):
            return stamp.sec + stamp.nanosec / 1e9
    return receive_s


@dataclass
class Axis:
    label: str
    joint: str   # joint name as it appears in the messages ('' if they carry none)
    index: int   # fallback for messages that carry no joint names


def resolve_axes(profile, data):
    # Axes come from whatever joints the command signal actually carries, not
    # a hardcoded count - so a 7-DOF arm or a 2-axis gantry just works. Labels
    # use the profile's axis_names when it lists exactly one per joint.
    for sig in (profile.command_signal, profile.feedback_signal):
        samples = data.get(sig.topic)
        if samples and sig.type == 'array':
            break
    else:
        return []
    first = samples[0][1]
    joints = _joint_names(first, sig)
    n = len(joints) or len(_get_field(first, sig.field))
    labels = profile.axis_names if profile.axis_names and len(profile.axis_names) == n else None
    if labels is None:
        labels = joints if joints else [f'joint_{i + 1}' for i in range(n)]
    return [Axis(label=labels[i], joint=joints[i] if joints else '', index=i) for i in range(n)]


def extract_series(data, signal, axis):
    # One joint's (times, positions) from one signal, sorted by time, looked
    # up by joint name when the messages carry names (so topics that order
    # joints differently still line up), by index otherwise.
    if signal.type == 'trajectory':
        from motion_trace.trajectory_signal import trajectory_series
        times, positions = trajectory_series(data, signal, axis)
        order = np.argsort(times, kind='stable')
        return times[order], positions[order]
    times, positions = [], []
    cached_names, idx = None, None
    for receive_s, msg in data.get(signal.topic, []):
        values = _get_field(msg, signal.field)
        names = _joint_names(msg, signal)
        if names != cached_names:
            cached_names = names
            if axis.joint and names:
                idx = names.index(axis.joint) if axis.joint in names else None
            else:
                idx = axis.index
        if idx is None or idx >= len(values):
            continue
        times.append(_stamp_s(msg, signal, receive_s))
        positions.append(values[idx])
    times, positions = np.array(times), np.array(positions)
    order = np.argsort(times, kind='stable')
    return times[order], positions[order]


def require_topics(profile, data, keys, topics_in_bag=None):
    # Every tool needs at least its core signals present; fail with a message
    # that points at the actual cause (wrong profile / topic names) instead of
    # silently producing empty results.
    signals = [profile.signal(k) for k in keys]
    if all(data.get(s.topic) for s in signals):
        return
    missing = [s.topic for s in signals if not data.get(s.topic)]
    present = sorted(topics_in_bag if topics_in_bag is not None else data.keys())
    raise SystemExit(
        f"Profile '{profile.name}': no messages on {', '.join(dict.fromkeys(missing))}.\n"
        f"Topics present in this bag: {', '.join(present) or '(none)'}\n"
        'Pick the profile matching the hardware this bag was recorded from (--profile), and '
        '--set if its topics live somewhere else.')
