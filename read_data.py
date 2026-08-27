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

if __name__ == "__main__":
    bag_path = "/home/changal/ros2_ws/stop_loop_bag"
    data = read_bag(bag_path)
    joint_feedback_data = data["/nex10/nex10/joint_feedback"]
    print(joint_feedback_data[0])
    # exit()
    joint1_feedback = [(stamp, msg.position[0], msg.velocity[0]) for stamp, msg in joint_feedback_data]
    joint2_feedback = [(stamp, msg.position[1], msg.velocity[1]) for stamp, msg in joint_feedback_data]
    joint3_feedback = [(stamp, msg.position[2], msg.velocity[2]) for stamp, msg in joint_feedback_data]
    joint4_feedback = [(stamp, msg.position[3], msg.velocity[3]) for stamp, msg in joint_feedback_data]
    joint5_feedback = [(stamp, msg.position[4], msg.velocity[4]) for stamp, msg in joint_feedback_data]
    joint6_feedback = [(stamp, msg.position[5], msg.velocity[5]) for stamp, msg in joint_feedback_data]

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))

    for i, ax1 in enumerate(axes.flat):
        times = [stamp for stamp, _ in joint_feedback_data]
        positions = [msg.position[i] for _, msg in joint_feedback_data]
        velocities = [msg.velocity[i] for _, msg in joint_feedback_data]

        # Position
        pos_line = ax1.plot(
            times,
            positions,
            color="tab:blue",
            label="Position"
        )[0]

        ax1.set_title(f"Joint {i + 1}")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Position (rad)", color="tab:blue")
        ax1.tick_params(axis="y", labelcolor="tab:blue")

        # Velocity
        ax2 = ax1.twinx()

        vel_line = ax2.plot(
            times,
            velocities,
            color="tab:red",
            label="Velocity"
        )[0]

        ax2.set_ylabel("Velocity (rad/s)", color="tab:red")
        ax2.tick_params(axis="y", labelcolor="tab:red")

        # Combined legend
        ax1.legend([pos_line, vel_line], ["Position", "Velocity"], loc="upper right")

    plt.tight_layout()
    plt.savefig("joint_feedback.png", dpi=200)
    plt.show()