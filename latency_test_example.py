import argparse
import time

import rclpy
from rclpy.action.client import ActionClient
from moveit_msgs.action import ExecuteTrajectory, MoveGroup, MoveGroupSequence
from moveit_msgs.msg import (
    MotionPlanRequest, MotionSequenceRequest, MotionSequenceItem, DisplayTrajectory,
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, JointConstraint,
)
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

from motion_trace.trajectory_signal import quintic_hermite, resample_grid
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import math


def quaternion_from_euler(roll, pitch, yaw):
    """(x, y, z, w) for static-axis roll/pitch/yaw, same convention as tf_transformations' default 'sxyz'."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
# Defaults match yns_moveit_config (NEX10 on the YNS 1.7.0 controller, MotoROS2).
DEFAULT_JOINT_NAMES = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
# Flange facing down. MotoROS2 on YNS reports B 90 deg below CMU's ynx convention, so
# CMU's all-zero HOME is B = -90 deg here.
DEFAULT_HOME = [0.0, 0.0, 0.0, 0.0, -math.pi / 2, 0.0]


class Move():
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('latency_test_example')
        # Every name below is a ROS parameter (--ros-args -p name:=value), defaulting to
        # yns_moveit_config's names.
        p = self.node.declare_parameter
        self.joint_names = list(p('joint_names', DEFAULT_JOINT_NAMES).value)
        self.home = [float(v) for v in p('home', DEFAULT_HOME).value]
        self.group_name = p('planning_group', 'manipulator').value
        self.tip_link = p('tip_link', 'tool0').value
        self.base_frame = p('base_frame', 'base_link').value
        move_topic = p('move_action', 'move_action').value
        sequence_topic = p('sequence_action', 'sequence_move_group').value
        execute_topic = p('execute_action', 'execute_trajectory').value
        display_topic = p('display_topic', 'display_planned_path').value
        server_timeout_s = p('server_timeout', 10.0).value

        self.action_client = ActionClient(self.node, MoveGroup, move_topic)
        # Pilz's blending capability (pilz_industrial_motion_planner/MoveGroupSequenceAction,
        # enabled in yns_moveit_config/launch/moveit.launch.py) - a single goal carrying
        # multiple waypoints, each with a `blend_radius`. blend_radius = 0 stops fully at that
        # waypoint before continuing; blend_radius > 0 blends through it without stopping.
        # This is what makes the stop-vs-non-stop comparison possible.
        self.sequence_client = ActionClient(self.node, MoveGroupSequence, sequence_topic)
        # With command_rate_hz > 0, legs are planned only, resampled, and executed through
        # move_group's ExecuteTrajectory action instead of MoveGroup's built-in execution.
        self.execute_client = ActionClient(self.node, ExecuteTrajectory, execute_topic)
        # The resampled trajectory is republished where move_group publishes its plans, so a
        # recording's command signal is exactly what the controller received.
        self.display_pub = self.node.create_publisher(DisplayTrajectory, display_topic, 10)
        for client, name in ((self.action_client, move_topic), (self.sequence_client, sequence_topic),
                             (self.execute_client, execute_topic)):
            if not client.wait_for_server(timeout_sec=server_timeout_s):
                raise SystemExit(f"No action server on '{name}' after {server_timeout_s:g}s - is move_group "
                                 'running? Set -p move_action:=... / -p sequence_action:=... / '
                                 '-p execute_action:=... if it lives elsewhere.')

    # --- Request builders (shared by single-goal and sequence-item paths) ---

    def create_joint_goal(self, joint_positions):
        if len(joint_positions) != len(self.joint_names):
            raise SystemExit(f'{len(joint_positions)} joint positions given for {len(self.joint_names)} '
                             f'joints ({self.joint_names}).')
        joint_constraints = []
        for name, pos in zip(self.joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            joint_constraints.append(jc)
        return joint_constraints

    def create_joint_request(self, joint_constraints, planner_id="PTP", velocity=0.1, acceleration=0.1):
        request = MotionPlanRequest()
        request.group_name = self.group_name
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
        request.group_name = self.group_name
        request.pipeline_id = 'pilz_industrial_motion_planner'
        request.planner_id = planner_id
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration

        # Position Constraint (1mm tolerance sphere)
        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = self.tip_link

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
        ori_constraint.link_name = self.tip_link
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
    # plan_only: plan every leg without moving. Each leg then starts from where the
    # previous plan ended (not from the robot's actual, unchanged state), so the
    # printout is the joint path the real run would take.
    plan_only = False
    _virtual_state = None
    # Rate [Hz] the trajectory sent to the controller is resampled to. 0 = send the
    # planner's own points (Pilz: every 0.1 s, not configurable in Humble).
    command_rate_hz = 0.0
    # Acceleration scaling factor (0-1) for every plan; None = same as the velocity scaling (--speed).
    accel_scale = None
    # Seconds of standing still, at the trajectory's start pose, prepended to every executed
    # trajectory: the controller is already running the goal when the move begins.
    hold_s = 0.0
    _waited_for_recorder = False

    @property
    def _resampling(self):
        return self.command_rate_hz > 0

    def resample(self, robot_trajectory):
        # Same Pilz profile, sampled every 1/command_rate_hz: positions, velocities and
        # accelerations from the quintic Hermite through the planner's points.
        jt = robot_trajectory.joint_trajectory
        if len(jt.points) < 2:
            return robot_trajectory
        t = np.array([pt.time_from_start.sec + pt.time_from_start.nanosec / 1e9 for pt in jt.points])
        n = len(jt.joint_names)
        col = lambda attr: (np.array([getattr(pt, attr) for pt in jt.points], float)
                            if all(len(getattr(pt, attr)) == n for pt in jt.points) else None)
        P, V, A = col('positions'), col('velocities'), col('accelerations')
        grid = resample_grid(t[0], t[-1], self.command_rate_hz)
        cols = [quintic_hermite(t, P[:, j], None if V is None else V[:, j],
                                None if A is None else A[:, j], grid) for j in range(n)]
        points = []
        for k, tk in enumerate(grid):
            pt = JointTrajectoryPoint()
            pt.positions = [float(c[0][k]) for c in cols]
            if cols[0][1] is not None:
                pt.velocities = [float(c[1][k]) for c in cols]
                pt.accelerations = [float(c[2][k]) for c in cols]
            tk += self.hold_s
            sec = int(tk)
            pt.time_from_start = Duration(sec=sec, nanosec=int(round((tk - sec) * 1e9)))
            if pt.time_from_start.nanosec >= 1_000_000_000:
                pt.time_from_start = Duration(sec=sec + 1, nanosec=0)
            points.append(pt)
        # Keep the planner's exact end state (zero velocity at a stop).
        points[-1].positions = list(jt.points[-1].positions)
        if jt.points[-1].velocities:
            points[-1].velocities = list(jt.points[-1].velocities)
        if jt.points[-1].accelerations:
            points[-1].accelerations = list(jt.points[-1].accelerations)
        if self.hold_s > 0:
            points = self._hold_points(points[0], self.hold_s) + points
        jt.points = points
        return robot_trajectory

    def _hold_points(self, first, hold_s):
        # Stationary points every 1/command_rate_hz from 0 to hold_s at the start pose; every
        # later point is shifted back by hold_s.
        hold = []
        for k in range(int(round(hold_s * self.command_rate_hz))):
            pt = JointTrajectoryPoint()
            pt.positions = list(first.positions)
            pt.velocities = [0.0] * len(first.positions)
            pt.accelerations = [0.0] * len(first.positions)
            tk = k / self.command_rate_hz
            pt.time_from_start = Duration(sec=int(tk), nanosec=int(round((tk - int(tk)) * 1e9)))
            hold.append(pt)
        return hold

    def _wait_for_recorder(self, timeout_s=5.0):
        # A recorder that hasn't discovered our publisher yet would miss the trajectory
        # (it isn't stored), so give DDS a moment the first time.
        if self._waited_for_recorder:
            return
        self._waited_for_recorder = True
        end = time.time() + timeout_s
        while self.display_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self.display_pub.get_subscription_count() == 0:
            self.node.get_logger().info('  (nothing subscribed to the planned path - not recording?)')

    def execute(self, trajectories):
        # Resample, publish for the recording, and execute each trajectory in order.
        for traj in trajectories:
            traj = self.resample(traj)
            n = len(traj.joint_trajectory.points)
            self._wait_for_recorder()
            display = DisplayTrajectory()
            display.trajectory = [traj]
            self.display_pub.publish(display)
            self.node.get_logger().info(f'  executing {n} points at {self.command_rate_hz:g} Hz')
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = traj
            send_future = self.execute_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send_future)
            handle = send_future.result()
            if not handle.accepted:
                self._fail('Trajectory execution was rejected by move_group')
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            code = result_future.result().result.error_code.val
            if code != 1:
                self._fail(f'Execution of the {n}-point trajectory failed with MoveIt error code {code} '
                           '(if the controller rejected it, check its log; a lower --command-rate-hz '
                           'sends fewer points)')

    def _prepare(self, goal_msg, start_state):
        if self.plan_only or self._resampling:
            goal_msg.planning_options.plan_only = True
        if self.plan_only:
            if self._virtual_state is not None:
                start_state.joint_state = self._virtual_state
                start_state.is_diff = False

    def _report_plan(self, trajectories):
        points = [p for t in trajectories for p in t.joint_trajectory.points]
        if not points:
            return
        jt = trajectories[-1].joint_trajectory
        end = points[-1].positions
        start = trajectories[0].joint_trajectory.points[0].positions
        n_points = sum(len(t.joint_trajectory.points) for t in trajectories)
        dur = sum(t.joint_trajectory.points[-1].time_from_start.sec + t.joint_trajectory.points[-1].time_from_start.nanosec / 1e9
                  for t in trajectories if t.joint_trajectory.points)
        moves = [math.degrees(e - b) for b, e in zip(start, end)]
        self.node.get_logger().info(
            f'  plan: {dur:.2f} s, {n_points} points, end [deg] ' + ' '.join(f'{n}={math.degrees(v):7.2f}' for n, v in zip(jt.joint_names, end))
            + f' | largest joint change {max(moves, key=abs):+.1f} deg')
        if self.plan_only and self._resampling:
            n = sum(len(self.resample(t).joint_trajectory.points) for t in trajectories)
            self.node.get_logger().info(f'  would send {n} points at {self.command_rate_hz:g} Hz')
        if self.plan_only:
            from sensor_msgs.msg import JointState
            self._virtual_state = JointState(name=list(jt.joint_names), position=list(end))

    def current_positions(self, timeout_s=5.0):
        # {joint: position}: where the previous plan ended in plan-only mode, else the
        # robot's latest /joint_states (None if nothing arrives).
        if self.plan_only and self._virtual_state is not None:
            return dict(zip(self._virtual_state.name, self._virtual_state.position))
        from sensor_msgs.msg import JointState
        got = []
        sub = self.node.create_subscription(JointState, 'joint_states', got.append, 10)
        end = time.time() + timeout_s
        while not got and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return dict(zip(got[-1].name, got[-1].position)) if got else None

    def _fail(self, what):
        # Stop the whole script: on hardware, carrying on to the next leg after a
        # failed one would start it from an unexpected pose.
        self.node.get_logger().error(f'{what}. Aborting script.')
        raise SystemExit(1)

    def run_action(self, goal_msg):
        planner_id = goal_msg.request.planner_id
        pipeline_id = goal_msg.request.pipeline_id
        group_name = goal_msg.request.group_name
        vel_scale = goal_msg.request.max_velocity_scaling_factor
        acc_scale = goal_msg.request.max_acceleration_scaling_factor
        self.node.get_logger().info(
            f'Sending goal: group={group_name}, pipeline={pipeline_id}, planner={planner_id}, '
            f'vel_scale={vel_scale}, acc_scale={acc_scale}' + (' [plan only]' if self.plan_only else ''))
        self._prepare(goal_msg, goal_msg.request.start_state)

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self._fail('Goal was rejected by MoveIt')
        self.node.get_logger().info('Goal accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.error_code.val != 1:
            self._fail(f'Motion failed with MoveIt error code {result.error_code.val}')
        self._report_plan([result.planned_trajectory])
        if self._resampling and not self.plan_only:
            self.execute([result.planned_trajectory])
        self.node.get_logger().info(f'Step completed [{pipeline_id}/{planner_id}].\n')

    def run_sequence(self, goal_msg):
        n_items = len(goal_msg.request.items)
        blends = [item.blend_radius for item in goal_msg.request.items]
        self.node.get_logger().info(f'Sending sequence goal: {n_items} waypoints, blend_radius={blends}'
                                    + (' [plan only]' if self.plan_only else ''))
        self._prepare(goal_msg, goal_msg.request.items[0].req.start_state)

        send_goal_future = self.sequence_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self._fail('Sequence goal was rejected by MoveIt')
        self.node.get_logger().info('Sequence accepted. Waiting for completion...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        if result.response.error_code.val != 1:
            self._fail(f'Sequence failed with MoveIt error code {result.response.error_code.val}')
        self._report_plan(list(result.response.planned_trajectories))
        if self._resampling and not self.plan_only:
            self.execute(list(result.response.planned_trajectories))
        self.node.get_logger().info(f'Sequence completed ({n_items} waypoints).\n')


HOME_TOLERANCE_RAD = math.radians(0.05)


def move_home(move: Move, speed: float):
    # Already there: skip. A zero-length plan is a single-point trajectory, which
    # MotoROS2 rejects ("Trajectory must contain at least two points").
    current = move.current_positions()
    if current is not None and all(abs(current[n] - h) < HOME_TOLERANCE_RAD
                                   for n, h in zip(move.joint_names, move.home) if n in current):
        move.node.get_logger().info('-- already at HOME, skipping --')
        return
    move.node.get_logger().info('-- moving to HOME --')
    joint_goal = move.create_joint_goal(move.home)
    joint_msg = move.create_joint_msg(joint_goal, velocity=speed, acceleration=move.accel_scale or speed)
    move.run_action(joint_msg)


def build_loop_nodes(move: Move, wrist_tilt_deg: float = 0.0):
    """Four Cartesian waypoints forming a small square loop in the Y-Z plane at
    X=0.6m, close to the poses already exercised in move_action_example.py.
    Both run_stop_loop() and run_nonstop_loop() start and end at HOME - these
    are just the four intermediate points in between.

    wrist_tilt_deg tilts the tool about Y, towards +Y on the +Y side of the square
    and the other way on the -Y side. With the tool pointing straight down (0) the
    wrist roll R never moves; a tilt makes R roll by about the same angle each way
    (planned on the NEX10: 15 deg tilt -> R about +/-15 deg), so every axis is exercised."""
    frame = move.base_frame
    tilt = math.radians(wrist_tilt_deg)
    return [
        move.create_pose_goal(frame, 0.6, y, z, 0.0, math.pi + (tilt if y > 0 else -tilt), math.pi / 2)
        for y, z in ((0.2, 0.7), (0.2, 0.5), (-0.2, 0.5), (-0.2, 0.7))
    ]


def run_stop_loop(move: Move, speed: float, wrist_tilt_deg: float = 0.0):
    """Baseline: one MoveGroup goal per leg. Each leg plans to a single target and
    comes to a full stop (zero velocity) before the next leg is even requested -
    by construction, since run_action() blocks until the result is back."""
    move.node.get_logger().info('=== STOP-BETWEEN-POINTS loop: starting ===\n')
    move_home(move, speed)
    nodes = build_loop_nodes(move, wrist_tilt_deg)
    for i, pose_goal in enumerate(nodes):
        move.node.get_logger().info(f'-- leg {i + 1}/{len(nodes)} (stop) --')
        goal_msg = move.create_pose_msg(pose_goal, planner_id="PTP", velocity=speed, acceleration=move.accel_scale or speed)
        move.run_action(goal_msg)
    move_home(move, speed)
    move.node.get_logger().info('=== STOP-BETWEEN-POINTS loop: done ===\n')


def run_nonstop_loop(move: Move, speed: float, blend_radius: float, wrist_tilt_deg: float = 0.0):
    """Comparison: the same four waypoints, but as a single MoveGroupSequence goal
    with a non-zero blend_radius on every leg except the last, so the arm keeps
    moving continuously through the intermediate waypoints instead of stopping.
    The HOME transitions on either side always come to a full stop (see
    move_home) - blending a large "go to home" move with the tight square loop
    isn't a meaningful comparison and would need its own, much larger, blend
    radius; only the four loop waypoints themselves are blended."""
    move.node.get_logger().info('=== NON-STOP loop: starting ===\n')
    move_home(move, speed)
    nodes = build_loop_nodes(move, wrist_tilt_deg)
    requests_with_blend = []
    for i, pose_goal in enumerate(nodes):
        request = move.create_pose_request(pose_goal, planner_id="PTP", velocity=speed, acceleration=move.accel_scale or speed)
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
                    'the two. Record each run separately with record_motion '
                    'and compare them in the dashboard.')
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
    parser.add_argument('--wrist-tilt-deg', type=float, default=15.0,
                         help='Tilt the tool this far about Y at the waypoints (mirrored between the '
                              'two sides of the square) so the wrist roll R moves too (default: 15; '
                              '0 = tool straight down, R never moves).')
    parser.add_argument('--plan-only', action='store_true',
                         help='Plan every leg without moving the robot, chaining each leg from the '
                              "previous plan's end, and print the joint targets - check them before "
                              'the first real run.')
    parser.add_argument('--command-rate-hz', type=float, default=100.0,
                         help="Resample each planned trajectory to this rate before sending it to the "
                              "controller (default: 100, every 10 ms). Pilz plans every 0.1 s and Humble "
                              "doesn't let that be changed; the resampled points follow the same profile. "
                              "0 = send the planner's points as they are. YNS 1.7.0 can't keep up with "
                              "250 Hz: it plays such a trajectory ~1.45x slow and MoveIt times out.")
    parser.add_argument('--accel', type=float, default=None,
                         help='Acceleration scaling factor, 0-1, applied to joint_limits.yaml\'s '
                              'max_acceleration (default: same as --speed). Pilz uses it for deceleration too.')
    parser.add_argument('--hold-s', type=float, default=0.0,
                         help='Prepend this many seconds of standing still (at the start pose) to every '
                              'executed trajectory, so the controller is already running the goal when the '
                              'move starts (default: 0). Needs --command-rate-hz > 0.')
    parsed_args, _ = parser.parse_known_args(args=args)

    move = Move()
    move.plan_only = parsed_args.plan_only
    move.command_rate_hz = max(0.0, parsed_args.command_rate_hz)
    move.accel_scale = parsed_args.accel
    move.hold_s = max(0.0, parsed_args.hold_s) if move.command_rate_hz > 0 else 0.0

    if parsed_args.mode in ('stop', 'both'):
        run_stop_loop(move, parsed_args.speed, parsed_args.wrist_tilt_deg)

    if parsed_args.mode == 'both':
        move.node.get_logger().info(f'Pausing {parsed_args.pause_between_modes:g}s before the non-stop loop...\n')
        time.sleep(parsed_args.pause_between_modes)

    if parsed_args.mode in ('nonstop', 'both'):
        run_nonstop_loop(move, parsed_args.speed, parsed_args.blend_radius, parsed_args.wrist_tilt_deg)


if __name__ == '__main__':
    main()
