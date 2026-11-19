# ynx_motion_analyzer

Records the commanded ("ideal") and feedback ("real") joint motion of an arm and shows how far
the feedback lags the command in a local web dashboard, per axis and per move. Three commands:

- `record_motion` records a move to a rosbag in `experiment/`.
- `dashboard` analyzes every recording there in the browser.
- `latency_test_example` is a MoveIt client that drives a repeatable test loop.

Nothing is tied to one hardware interface: which topics to record, which message field holds
the positions, and what to call each axis all come from a **hardware profile** (`--profile`,
see below). The default `motoros2` profile covers Yaskawa's MotoROS2 interface (including the
YNX1000's ROS 2 plugin), and `ros2_control` works with any ros2_control hardware. The dashboard
picks the matching profile from each recording's topics.

## Quick start: MotoROS2

MotoROS2 publishes its state directly, with no ros2_control in between. The profile records:

| Topic | Type | What it is |
|---|---|---|
| `/display_planned_path` | `moveit_msgs/DisplayTrajectory` | MoveIt's plan for each move (relative times) - the **command** signal |
| `/follow_joint_trajectory/_action/status` | `action_msgs/GoalStatusArray` | When the controller accepted each goal, on its own clock. Anchors each plan in time |
| `/follow_joint_trajectory/_action/feedback` | FJT feedback | Fallback anchor only. On YNS 1.7.0 its `desired` field is in degrees (not radians) and is the last point loaded into the controller's ~25-30-point motion queue, not the current setpoint - so it runs ahead of the plan, and with short trajectories it is the final goal all move |
| `/joint_states` | `sensor_msgs/JointState` | Encoder position, ~100 Hz - the **feedback** signal |
| `/robot_status` | `industrial_msgs/RobotStatus` | `in_motion`, `e_stopped`, ... (context only) |

The measured delay is "MoveIt's plan, started when the controller accepted the goal" to the
encoder: everything the controller adds between accepting a goal and the arm following it.
Plan anchor and encoder are both on the
controller's clock, so the PC/controller clock offset doesn't matter (and PC → controller
network time isn't included). **Moves must come from MoveIt** executing through
`follow_joint_trajectory`; moves without a plan are skipped.
`profiles/motoros2.yaml` documents each topic in detail.

1. Check that the controller is visible and what its topics are really called (MotoROS2's
   topics live at the root):
   ```bash
   ros2 topic list --include-hidden-topics
   ```
2. Record:
   ```bash
   ros2 run ynx_motion_analyzer record_motion -o my_move
   ```
   The action topics are hidden, so `record_motion` adds `--include-hidden-topics` for you.
   Wait a few seconds after it reports "Subscribed" for every topic (DDS needs that long to
   connect, and a plan published before then is lost), execute the move, then `Ctrl+C`.
3. Analyze: start the dashboard (below) and pick the recording.

Axes are labelled S/L/U/R/B/T for a 6-joint recording, otherwise by the joint names MotoROS2
publishes (`joint_0`..`joint_5` on the YNS 1.7.0 controller). If the action server has a
different name, record with `--set action=<name>`.

## Dashboard

Interactive view of every recording in `experiment/`:

```bash
ros2 run ynx_motion_analyzer dashboard        # then open http://localhost:8050
```

- **Two delays per move:**
  - *Start delay*: when command and feedback each leave rest (0.05° from where they stood).
  - *Motion delay*: the mean of 10 delays sampled at random positions through the move
    (0.3° to 95% of its travel).
  On YNS 1.7.0 the arm leaves rest a little sooner than the motion delay suggests (5-8 ms at
  100 Hz trajectories, ~17 ms with Pilz's own 0.1 s points).
- **Summary:** both typical delays, a per-axis table (typical start and motion delay, range of the
  per-move motion delays, RMS error before → after shifting feedback by the typical motion
  delay, and a note on any move whose motion delay differs from its axis' typical by > 20%).
- **Per axis:** position (command vs. feedback, moves shaded, optional shifted feedback), both
  delays for every move, and a table view of the moves.
- The calculations are in `motion_trace/analysis.py` (per-axis results) and
  `motion_trace/delay.py` (move detection, threshold crossings, delay sampling). Moves are found
  on the command: wherever it moves faster than 1°/s and travels at least 1°.
- Analysis only - nothing on the page talks to the robot. Binds to this PC only; pass
  `--host 0.0.0.0` to share on the LAN, `--port` to change the port. Works offline (Plotly is
  bundled in `dashboard/vendor/`).

## Hardware profiles

A profile is a small YAML file in `profiles/` (installed to
`share/ynx_motion_analyzer/profiles/`). `record_motion` takes:

| Flag | Meaning |
|---|---|
| `--profile NAME\|PATH` | Built-in profile name, or a path to your own YAML. Default: `motoros2`. The dashboard detects it from the recording |
| `--set KEY=VALUE` | Fills a placeholder the profile defines, e.g. `--set action=...` (repeatable) |

Built-in profiles:

- **`motoros2`** (default): see [Quick start: MotoROS2](#quick-start-motoros2). Defaults
  `action=follow_joint_trajectory`. Topics are at the root (no node namespace).
- **`ros2_control`** - `joint_trajectory_controller`'s standard `controller_state` topic
  (`reference.positions` vs `feedback.positions`). This works with any hardware
  interface, mock included, because it relies only on the controller. The measured delay
  is the lag from the controller's reference to the state-interface feedback. Both
  signals share one timestamp per control cycle. Default
  `controller=joint_trajectory_controller`. Use e.g. `--set controller=scaled_joint_trajectory_controller`
  for UR-style drivers.

```bash
ros2 run ynx_motion_analyzer record_motion --profile ros2_control -o jtc_bag
```

**Adding new hardware** takes a YAML file and no code changes. Pass its path to
`--profile`, or drop it into `profiles/` and rebuild to use it by name.
`profiles/motoros2.yaml` is the annotated reference:

```yaml
name: my_arm
description: what these topics are
params:                  # placeholder defaults; --set KEY=VALUE overrides them
  driver: my_arm
axis_names: [A1, A2, A3] # optional, in joint order; omit to label axes by joint name
command: cmd             # the two signals every delay measurement compares
feedback: fb
signals:                 # shown in this order; may include extra context signals
  - {key: cmd, topic: '/{driver}/command', field: position, label: commanded}
  - {key: fb,  topic: '/joint_states',     field: position, label: feedback}
extra_topics:            # optional: recorded for context, never analyzed
  - /robot_status
```

- `field` is a dotted path into the message (`position` for `sensor_msgs/JointState`,
  `reference.positions` for a controller state, `feedback.desired.positions` for an action
  feedback topic, and so on). Any message type works if it carries an array of positions
  and a `name` or `joint_names` list.
- The joint-name list and `header` are looked up next to `field`, walking up towards the
  message root, so nested layouts need no extra settings. Set `names_field` / `stamp_field`
  on a signal to point at them explicitly.
- Hidden topics (any name segment starting with `_`, like `_action/feedback`) are
  recorded with `--include-hidden-topics` automatically.
- Several signals can share one topic.
- Joints are matched **by name** across topics, so topics that order joints differently
  still line up. They're matched by index only when a message carries no names.
- The axis count comes from the recording, so 7-DOF arms and 2-axis stages need no
  special handling.
- Timestamps come from the message's `header.stamp`. The bag's receive time is used only
  when a message has no stamp.
- Positions are treated as radians and shown in degrees, so for now the analysis assumes
  revolute joints.

## Experiment recordings live under `experiment/`

`record_motion` saves every bag under this package's source folder,
`<workspace>/src/ynx_motion_analyzer/experiment/`, which it finds from the install prefix.
A relative `-o name` is placed there automatically. Pass an absolute path to put it
somewhere else, or set `MOTION_ANALYZER_EXPERIMENT_DIR` to change the folder. The dashboard
lists every recording in that folder. Recordings are git-ignored; to share one, force-add it
(`git add -f experiment/<name>`).

## Latency test: stop vs. non-stop

`latency_test_example` drives the arm through the same 4-waypoint loop
(bookended by HOME, the flange-down pose) two ways, so you can compare the delay
behavior between them:

- **`--mode stop`** — one `MoveGroup` goal per leg (HOME → A → B → C → D →
  HOME). The arm comes to a full stop at every point.
- **`--mode nonstop`** — the four loop waypoints (A → B → C → D) sent as a
  single `MoveGroupSequence` goal (Pilz's blending capability - already enabled
  via `pilz_industrial_motion_planner/MoveGroupSequenceAction` in
  `yns_moveit_config/launch/moveit.launch.py`, action name
  `sequence_move_group`) with `--blend-radius` on every leg but the last, so the
  arm moves continuously through the intermediate waypoints instead of
  stopping. The HOME transitions on both ends always stop fully - they're kept
  outside the blended sequence on purpose, since blending a large "go to home"
  move with the tight square loop wouldn't be a meaningful comparison.
- **`--mode both`** (default) — runs stop then non-stop in the same process,
  pausing `--pause-between-modes` seconds in between.

Record each mode **separately**, so each recording's summary describes one kind of motion:

```bash
# terminal A
ros2 run ynx_motion_analyzer record_motion -o stop_loop_bag
# terminal B, a few seconds after recording says "Subscribed to topic" for every topic
# (DDS needs that long to connect; a plan published before then is missed - it isn't stored)
ros2 run ynx_motion_analyzer latency_test_example --mode stop --speed 0.1
# Ctrl+C terminal A once it finishes

# terminal A
ros2 run ynx_motion_analyzer record_motion -o nonstop_loop_bag
# terminal B
ros2 run ynx_motion_analyzer latency_test_example --mode nonstop --speed 0.1 --blend-radius 0.05
# Ctrl+C terminal A once it finishes
```

Then compare `stop_loop_bag` and `nonstop_loop_bag` in the dashboard.

`latency_test_example`'s defaults (joint names, planning group, frames, HOME) already
match `yns_moveit_config`. Other MoveIt setups can override them as ROS parameters
after `--ros-args`, alongside the plain `--mode`/`--speed`/`--blend-radius` flags:

```bash
ros2 run ynx_motion_analyzer latency_test_example --mode stop --ros-args \
  -p planning_group:=my_group -p tip_link:=my_tool -p base_frame:=my_base
```

**First real run tip:** start with a small `--blend-radius` (e.g. `0.02`), or
try `--mode stop` only at first. If the radius is larger than the distance
between your actual waypoints, Pilz rejects the sequence goal ("Goal was
rejected" in the log) - shrink it or space the waypoints further apart. The
four waypoints themselves are defined in `build_loop_nodes()` in
`latency_test_example.py`; edit them if the default square (X=0.6m, Y=±0.2m,
Z=0.5-0.7m) doesn't suit your workspace.

## Command reference

`record_motion`:
| Flag | Default | Meaning |
|---|---|---|
| `--profile`/`--set` | `motoros2` profile | Which topics to record - see [Hardware profiles](#hardware-profiles) |
| `-o`, `--output` | `experiment/motion_bag_<timestamp>` | Bag output directory or relative name (placed under `experiment/`); pass an absolute path to override |
| `--extra-topic` | - | Additional topic to record (repeatable) |

`latency_test_example`:
| Flag | Default | Meaning |
|---|---|---|
| `--mode {stop,nonstop,both}` | `both` | Which loop(s) to run |
| `--speed` | `0.1` | Velocity/acceleration scaling factor (0-1) |
| `--accel` | same as `--speed` | Acceleration (and deceleration) scaling factor (0-1), separately from velocity |
| `--hold-s` | `0` | Prepend this many seconds of standing still at the start pose to every executed trajectory, so the controller is already running the goal when the move starts (needs `--command-rate-hz` > 0) |
| `--blend-radius` | `0.05` | Blend radius in meters for the non-stop loop's intermediate legs |
| `--pause-between-modes` | `3.0` | Seconds to pause between the two runs when `--mode both` is used |
| `--wrist-tilt-deg` | `15.0` | Tilt the tool about Y at the waypoints (mirrored per side) so the wrist roll R moves too; `0` = tool straight down, R stays still |
| `--command-rate-hz` | `100` | Resample each plan to this rate before sending it to the controller. YNS 1.7.0 plays 250 Hz (4 ms) trajectories ~1.45x slower than planned, so MoveIt times out; 100 Hz plays on time. Pilz plans every 0.1 s and Humble can't change that; the resampled points follow the same profile and are executed via move_group's `execute_trajectory`, and republished on `/display_planned_path` so the recording's command is exactly what was sent. `0` = send Pilz's points as they are |
| `--plan-only` | off | Plan every leg without moving (each leg chained from the previous plan's end) and print the joint targets. Run this first on new hardware |
| `--ros-args -p joint_names:=[...]` | `joint_0`..`joint_5` | ROS parameter, not an argparse flag - joint names in MoveIt's robot model |
| `-p home:=[...]` | `[0, 0, 0, 0, -1.5708, 0]` | HOME joint positions [rad] (flange down on the YNS NEX10) |
| `-p planning_group:=` / `tip_link:=` / `base_frame:=` | `manipulator`, `tool0`, `base_link` | MoveIt config names |
| `-p move_action:=` / `sequence_action:=` / `execute_action:=` | `move_action`, `sequence_move_group`, `execute_trajectory` | MoveIt action names |
| `-p server_timeout:=` | `10.0` | Seconds to wait for MoveIt's action servers before giving up |
