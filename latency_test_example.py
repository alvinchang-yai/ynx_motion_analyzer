import argparse
import time

import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import MoveGroup, MoveGroupSequence
from moveit_msgs.msg import (
    MotionPlanRequest, MotionSequenceRequest, MotionSequenceItem,
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint,
)
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from shape_msgs.msg import SolidPrimitive
import math


class Move():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('latency_test_example')
        self.node.declare_parameter('ns', '')
        self.ns = self.node.get_parameter("ns").value
        self.prefix = ""
        move_topic = "move_action"
        sequence_topic = "sequence_move_group"
        if self.ns != "":
            self.prefix = str(self.ns) + "_"
            move_topic = "/" + str(self.ns) + "/move_action"
            sequence_topic = "/" + str(self.ns) + "/sequence_move_group"

        self.action_client = ActionClient(self.node, MoveGroup, move_topic)
        self.action_client.wait_for_server()

        # Pilz's blending capability (pilz_industrial_motion_planner/MoveGroupSequenceAction,
        # enabled in pilz_industrial_motion_planner_planning.yaml) - a single goal carrying
        # multiple waypoints, each with a `blend_radius`. blend_radius = 0 stops fully at that
        # waypoint before continuing; blend_radius > 0 blends through it without stopping.
        # This is what makes the stop-vs-non-stop comparison possible.
        self.sequence_client = ActionClient(self.node, MoveGroupSequence, sequence_topic)
        self.sequence_client.wait_for_server()

    # --- Request builders (shared by single-goal and sequence-item paths) ---

    def create_joint_goal(self, joint_positions):
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        joint_constraints = []
        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = self.prefix + name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)
        return joint_constraints

    def create_joint_request(self, joint_constraints, planner_id="PTP", velocity=0.1, acceleration=0.1):
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = planner_id
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration

        constraint = Constraints()
        constraint.joint_constraints = joint_constraints
        request.goal_constraints.append(constraint)
        return request

    def create_joint_msg(self, joint_constraints, planner_id="PTP", velocity=0.1, acceleration=0.1):
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.create_joint_request(joint_constraints, planner_id, velocity, acceleration)
        return goal_msg

    def create_pose_goal(self, frame, x, y, z, ax, ay, az):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        q = quaternion_from_euler(ax, ay, az)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        return msg

    def create_pose_request(self, target_pose: PoseStamped, planner_id: str = "PTP",
                             velocity: float = 0.1, acceleration: float = 0.1) -> MotionPlanRequest:
        request = MotionPlanRequest()
        request.group_name = self.prefix + 'manipulator'
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = planner_id
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration

        # Position Constraint (1mm tolerance sphere)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.prefix + 'tool0'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.0001]

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bounding_volume
        pos_constraint.weight = 1.0

        # Orientation Constraint (approx 0.05 rad tolerance)
        ori_constraint = OrientationConstraint()
        ori_constraint.header = target_pose.header
        ori_constraint.link_name = self.prefix + 'tool0'
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)

        request.goal_constraints.append(constraint)
        return request

    def create_pose_msg(self, target_pose: PoseStamped, planner_id: str = "PTP",
                         velocity: float = 0.1, acceleration: float = 0.1) -> MoveGroup.Goal:
        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.create_pose_request(target_pose, planner_id, velocity, acceleration)
        return goal_msg

    def create_sequence_msg(self, requests_with_blend):
        # requests_with_blend: list of (MotionPlanRequest, blend_radius_m) tuples, in order.
        # The last item's blend_radius should normally be 0.0 so the loop actually comes to
        # a stop at the end, rather than trying to blend past the final waypoint.
        goal_msg = MoveGroupSequence.Goal()
        seq_request = MotionSequenceRequest()
        for request, blend_radius in requests_with_blend:
            item = MotionSequenceItem()
            item.req = request
            item.blend_radius = float(blend_radius)
            seq_request.items.append(item)
        goal_msg.request = seq_request
        return goal_msg

    # --- Action runners ---

    def run_action(self, goal_msg):
        planner_id = goal_msg.request.planner_id
        pipeline_id = goal_msg.request.pipeline_id
        group_name = goal_msg.request.group_name
        vel_scale = goal_msg.request.max_velocity_scaling_factor
        acc_scale = goal_msg.request.max_acceleration_scaling_factor
        self.node.get_logger().info(
            f'Sending goal: group={group_name}, pipeline={pipeline_id}, planner={planner_id}, '
            f'vel_scale={vel_scale}, acc_scale={acc_scale}')

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Goal was rejected by MoveIt! Aborting script.')
            return
        self.node.get_logger().info('Goal accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code.val != 1:
            self.node.get_logger().error(f'Motion failed with error code: {result.error_code.val}. Aborting script.')
            return

        self.node.get_logger().info(f'Step completed [{pipeline_id}/{planner_id}].\n')

    def run_sequence(self, goal_msg):
        n_items = len(goal_msg.request.items)
        blends = [item.blend_radius for item in goal_msg.request.items]
        self.node.get_logger().info(f'Sending sequence goal: {n_items} waypoints, blend_radius={blends}')

        send_goal_future = self.sequence_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Sequence goal was rejected by MoveIt! Aborting script.')
            return
        self.node.get_logger().info('Sequence accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.response.error_code.val != 1:
            self.node.get_logger().error(
                f'Sequence failed with error code: {result.response.error_code.val}. Aborting script.')
            return

        self.node.get_logger().info(f'Sequence completed ({n_items} waypoints).\n')


HOME_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def move_home(move: Move, speed: float):
    move.node.get_logger().info('-- moving to HOME --')
    joint_goal = move.create_joint_goal(HOME_JOINT_POSITIONS)
    joint_msg = move.create_joint_msg(joint_goal, velocity=speed, acceleration=speed)
    move.run_action(joint_msg)


def build_loop_nodes(move: Move):
    """Four Cartesian waypoints forming a small square loop in the Y-Z plane at
    X=0.6m, close to the poses already exercised in move_action_example.py.
    Both run_stop_loop() and run_nonstop_loop() start and end at HOME - these
    are just the four intermediate points in between."""
    frame = move.prefix + "base_link"
    return [
        move.create_pose_goal(frame, 0.6, 0.2, 0.7, 0.0, math.pi, math.pi / 2),
        move.create_pose_goal(frame, 0.6, 0.2, 0.5, 0.0, math.pi, math.pi / 2),
        move.create_pose_goal(frame, 0.6, -0.2, 0.5, 0.0, math.pi, math.pi / 2),
        move.create_pose_goal(frame, 0.6, -0.2, 0.7, 0.0, math.pi, math.pi / 2),
    ]


def run_stop_loop(move: Move, speed: float):
    """Baseline: one MoveGroup goal per leg. Each leg plans to a single target and
    comes to a full stop (zero velocity) before the next leg is even requested -
    by construction, since run_action() blocks until the result is back."""
    move.node.get_logger().info('=== STOP-BETWEEN-POINTS loop: starting ===\n')
    move_home(move, speed)
    nodes = build_loop_nodes(move)
    for i, pose_goal in enumerate(nodes):
        move.node.get_logger().info(f'-- leg {i + 1}/{len(nodes)} (stop) --')
        goal_msg = move.create_pose_msg(pose_goal, planner_id="PTP", velocity=speed, acceleration=speed)
        move.run_action(goal_msg)
    move_home(move, speed)
    move.node.get_logger().info('=== STOP-BETWEEN-POINTS loop: done ===\n')


def run_nonstop_loop(move: Move, speed: float, blend_radius: float):
    """Comparison: the same four waypoints, but as a single MoveGroupSequence goal
    with a non-zero blend_radius on every leg except the last, so the arm keeps
    moving continuously through the intermediate waypoints instead of stopping.
    The HOME transitions on either side always come to a full stop (see
    move_home) - blending a large "go to home" move with the tight square loop
    isn't a meaningful comparison and would need its own, much larger, blend
    radius; only the four loop waypoints themselves are blended."""
    move.node.get_logger().info('=== NON-STOP loop: starting ===\n')
    move_home(move, speed)
    nodes = build_loop_nodes(move)
    requests_with_blend = []
    for i, pose_goal in enumerate(nodes):
        request = move.create_pose_request(pose_goal, planner_id="PTP", velocity=speed, acceleration=speed)
        is_last = (i == len(nodes) - 1)
        requests_with_blend.append((request, 0.0 if is_last else blend_radius))
    goal_msg = move.create_sequence_msg(requests_with_blend)
    move.run_sequence(goal_msg)
    move_home(move, speed)
    move.node.get_logger().info('=== NON-STOP loop: done ===\n')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Drive the arm through the same 4-node loop with the points either fully '
                    'stopped-at or blended-through, to compare latency/tracking behavior between '
                    'the two. Record each run separately with ynx_motion_analyzer\'s record_motion '
                    'and compare the resulting plots.')
    parser.add_argument('--mode', choices=['stop', 'nonstop', 'both'], default='both',
                         help="Which loop(s) to run (default: both, stop first then nonstop).")
    parser.add_argument('--speed', type=float, default=0.1,
                         help='Velocity/acceleration scaling factor, 0-1 (default: 0.1).')
    parser.add_argument('--blend-radius', type=float, default=0.05,
                         help='Blend radius in meters for the non-stop loop (default: 0.05). Must be '
                              'smaller than the distance between adjacent waypoints or Pilz will '
                              'reject the sequence - tune this once you see it running on hardware.')
    parser.add_argument('--pause-between-modes', type=float, default=3.0,
                         help='Seconds to pause between the stop and non-stop runs when --mode both '
                              'is used, so they are easy to tell apart in a recording (default: 3.0).')
    parsed_args, _ = parser.parse_known_args(args=args)

    move = Move()

    if parsed_args.mode in ('stop', 'both'):
        run_stop_loop(move, parsed_args.speed)

    if parsed_args.mode == 'both':
        move.node.get_logger().info(f'Pausing {parsed_args.pause_between_modes:g}s before the non-stop loop...\n')
        time.sleep(parsed_args.pause_between_modes)

    if parsed_args.mode in ('nonstop', 'both'):
        run_nonstop_loop(move, parsed_args.speed, parsed_args.blend_radius)


if __name__ == '__main__':
    main()
