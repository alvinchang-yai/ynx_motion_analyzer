# ynx_motion_analyzer

Records the commanded ("ideal") and feedback ("real") joint trajectories streamed by
`ynx_hardware_interface` during any arm movement, and plots them per axis so tracking
behavior can be inspected visually. Also includes `latency_test_example`, a MoveIt
client that drives a repeatable stop-vs-non-stop loop for exercising this analysis.

Requires **real hardware** (`use_mock_hardware:=false`). `mock_components/GenericSystem`
never runs `ynx_hardware_interface`'s code, so the topics this tool needs simply don't
exist under mock hardware.

## What gets recorded

`ynx_hardware_interface` publishes four topics every control cycle, for the whole
duration of any movement (not just a specific action call) — four checkpoints of
the same pipeline, in the order they happen:

- `~/joint_command_sent` — the position ros2_control wants this cycle, timestamped
  right before it's handed to the gRPC call ("when ros2_control sent it").
- `~/joint_command` — the same position, timestamped right after the ACU
  acknowledges the gRPC call ("when the ACU received it").
- `~/joint_command_acu` — the ACU's own internal command/setpoint stream
  (`GetAxesPos`) - what the ACU's own interpolator is currently tracking toward,
  reported by the ACU itself. Distinct from `joint_command` (our record of what
  we sent) and from `joint_feedback` (the physical encoder). A failure to read
  this is logged but never fails the control loop - it's recording-only, unlike
  the feedback read.
- `~/joint_feedback` — the actual encoder position/velocity read back from the ACU
  (`GetFeedbackAxesPos`) ("when the arm actually shows it").

The gap between the first two is network/gRPC latency to the ACU; the gap between
the last two is the ACU's own internal + mechanical response latency - and
`joint_command_acu` lets you split that further into "ACU accepted it but hadn't
started tracking yet" vs. "ACU was tracking but the arm hadn't physically caught up."

All four resolve under the hardware component's node, which is nested under
whatever outer namespace bringup was launched with. With `ns:=nex10`, the real
topics are:

```
/nex10/nex10/joint_command_sent
/nex10/nex10/joint_command
/nex10/nex10/joint_command_acu
/nex10/nex10/joint_feedback
```

The first `nex10` is the bringup namespace (`ns:=`); the second is the hardware
component's own node name, hardcoded in `nex10.ros2_control_macro.xacro` regardless
of `ns`. If your `ns` differs, verify the real topic names with `ros2 topic list`.

## Steps to obtain a plot

1. **Build the workspace** (from `ros2_ws/`):
   ```bash
   colcon build
   ```

2. **Launch bringup with real hardware**, in its own terminal:
   ```bash
   source install/setup.bash
   ros2 launch ynx_bringup bringup.launch.py \
     ns:=nex10 ip:=192.168.1.253 port:=50300 \
     use_mock_hardware:=false model:=nex10 \
     launch_rviz:=false launch_servo:=true use_ft_sensor:=false
   ```
   Adjust `ip`/`port`/`ns` for your setup. Wait until it's fully up (servos powered
   on, controllers active) before continuing.

3. **Start recording**, in a new terminal:
   ```bash
   source install/setup.bash
   ros2 run ynx_motion_analyzer record_motion --ns nex10
   ```
   This starts `ros2 bag record` against `joint_command_sent`/`joint_command`/
   `joint_command_acu`/`joint_feedback` and prints the output directory name
   (default `motion_bag_<timestamp>`, override with `-o`). Leave it running.

4. **Trigger the movement**, in a third terminal:
   ```bash
   source install/setup.bash
   ros2 run ynx_examples move_action_example --ros-args -p ns:=nex10
   ```
   Or run whatever node/script actually commands the arm — the recorder captures
   any movement, regardless of what issued it. To specifically exercise a
   stop-vs-non-stop comparison, use `latency_test_example` instead - see below.

5. **Stop the recording** once the move is done: `Ctrl+C` in the `record_motion`
   terminal. It waits for `ros2 bag record` to shut down gracefully so the bag isn't
   left corrupt/empty.

6. **Generate the plots**:
   ```bash
   ros2 run ynx_motion_analyzer plot_motion motion_bag_20260824_134335 --ns nex10
   ```
   This writes into `motion_bag_20260824_134335/plot/`:
   ```
   plot/S.png  plot/L.png  plot/U.png  plot/R.png  plot/B.png  plot/T.png
   ```
   Axis names follow the standard Yaskawa MOTOMAN 6-axis convention, in joint
   order (`joint_1`..`joint_6` → S, L, U, R, B, T).

## Reading the plots

The `sent -> ACU ack` gap is only a few milliseconds - invisible on a plot whose
x-axis spans a multi-second move, and even zoomed in, four overlapping
near-identical step lines are hard to tell apart by eye. So don't try to eyeball
delays off the lines directly; read the number this tool computes for you instead:

- **Each `<axis>.png`** has two panels, both showing all four checkpoints (sent,
  ACU ack, ACU internal setpoint, feedback):
  - **Top** — the full move, seconds, unzoomed, for context.
  - **Bottom** — the same four lines, auto-zoomed to a millisecond-scale window.
    The delay measurement itself still only compares `sent` and `feedback` (the
    two ends of the pipeline) - the ACU-ack and ACU-internal-setpoint lines are
    there for visual context on where the delay comes from, not folded into the
    number. It's: **when command reaches `--threshold-deg` (default 1°), what's
    the latency until feedback also reaches `--threshold-deg`** - both measured
    from the same shared baseline, found by a sequential scan of the recording
    from its very start:
    1. Scan the commanded (`sent`) position forward until it first moves away
       from its initial (idle) value at all - this only establishes a clean
       baseline (both signals' own value at that moment), it isn't one of the
       two points being compared.
    2. From that baseline, find when `sent` reaches `--threshold-deg`, and
       separately when `feedback` reaches the same `--threshold-deg` (each
       measured from its own value at the baseline moment, so a move that
       begins partway through a bag is measured against the right reference,
       not whatever the first recorded sample happened to be).
    The window is those two "reached threshold" timestamps plus a small margin,
    auto-detected fresh for every axis - there's no manual override, since the
    detection anchors correctly regardless of when in the recording the move
    happens. Each timestamp is marked with a thin color-matched vertical line,
    and since both are the *same* position (both `threshold_deg` past the
    shared baseline), a red dotted line connects them at that shared level with
    the measured gap labeled right on it (and repeated in a text box) - "same
    position, this many ms apart," not something you infer from comparing two
    curves.
- **`--show`** — opens live, interactive matplotlib windows for every plot, in
  addition to still saving all the PNGs as usual. Use the toolbar's zoom/pan tool
  (rectangle-select) to freely explore any time range down to individual samples.
  Opens one window per axis; close them (or Ctrl+C) to let the script exit. Needs
  a working GUI backend/display (a local X server, X11 forwarding, or WSLg on
  Windows) - if nothing shows up, that's almost always a missing display, not a
  bug in the tool.

## Latency test: stop vs. non-stop

`latency_test_example` drives the arm through the same 4-waypoint loop
(bookended by HOME - all joints at 0) two ways, so you can compare the delay
behavior between them:

- **`--mode stop`** — one `MoveGroup` goal per leg (HOME → A → B → C → D →
  HOME). The arm comes to a full stop at every point.
- **`--mode nonstop`** — the four loop waypoints (A → B → C → D) sent as a
  single `MoveGroupSequence` goal (Pilz's blending capability - already enabled
  via `pilz_industrial_motion_planner/MoveGroupSequenceAction` in
  `pilz_industrial_motion_planner_planning.yaml`, action name
  `sequence_move_group`) with `--blend-radius` on every leg but the last, so the
  arm moves continuously through the intermediate waypoints instead of
  stopping. The HOME transitions on both ends always stop fully - they're kept
  outside the blended sequence on purpose, since blending a large "go to home"
  move with the tight square loop wouldn't be a meaningful comparison.
- **`--mode both`** (default) — runs stop then non-stop in the same process,
  pausing `--pause-between-modes` seconds in between.

Record each mode **separately** for the cleanest analysis - `plot_motion`'s
auto-detection locks onto the first movement in a bag, so a bag containing both
modes back-to-back will only auto-analyze the first one (still fine for
eyeballing the whole recording, just not for the per-axis delay numbers on the
second move):

```bash
# terminal A
ros2 run ynx_motion_analyzer record_motion --ns nex10 -o stop_loop_bag
# terminal B, once recording says "Listening for topics..."
ros2 run ynx_motion_analyzer latency_test_example --mode stop --speed 0.1 --ros-args -p ns:=nex10
# Ctrl+C terminal A once it finishes

# terminal A
ros2 run ynx_motion_analyzer record_motion --ns nex10 -o nonstop_loop_bag
# terminal B
ros2 run ynx_motion_analyzer latency_test_example --mode nonstop --speed 0.1 --blend-radius 0.05 --ros-args -p ns:=nex10
# Ctrl+C terminal A once it finishes

ros2 run ynx_motion_analyzer plot_motion stop_loop_bag --ns nex10
ros2 run ynx_motion_analyzer plot_motion nonstop_loop_bag --ns nex10
```

Note `--ros-args -p ns:=nex10` (a ROS *parameter*, consumed by the node to
build topic/action names) coexists on the same command line as `--mode`/
`--speed`/`--blend-radius` (plain argparse flags) - both are needed.

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
| `--ns` | `''` | Bringup's `ns:=` argument, if any |
| `--hw-node` | `nex10` | Hardware component's node name (from the xacro) |
| `-o`, `--output` | `motion_bag_<timestamp>` | Bag output directory |
| `--extra-topic` | - | Additional topic to record (repeatable) |

`plot_motion`:
| Flag | Default | Meaning |
|---|---|---|
| `--ns` | `''` | Same as above; used to build default topic names |
| `--hw-node` | `nex10` | Same as above |
| `--sent-topic` / `--command-topic` / `--acu-topic` / `--feedback-topic` | built from `--ns`/`--hw-node` | Override the topic names directly |
| `--axis` | all six | Restrict to specific axes, e.g. `--axis S --axis T` |
| `--threshold-deg` | `1.0` | Fixed degree threshold the command-start -> feedback delay is measured from |
| `--show` | off | Also open live, interactive windows (requires a display) |

`latency_test_example`:
| Flag | Default | Meaning |
|---|---|---|
| `--mode {stop,nonstop,both}` | `both` | Which loop(s) to run |
| `--speed` | `0.1` | Velocity/acceleration scaling factor (0-1) |
| `--blend-radius` | `0.05` | Blend radius in meters for the non-stop loop's intermediate legs |
| `--pause-between-modes` | `3.0` | Seconds to pause between the two runs when `--mode both` is used |
| `--ros-args -p ns:=<ns>` | (none) | ROS parameter, not an argparse flag - the bringup namespace |

## Troubleshooting

- **No messages / empty bag**: almost always `use_mock_hardware:=true`, or a `--ns`
  mismatch between recording and plotting. Confirm with `ros2 topic list` while
  bringup is running.
- **`plot_motion` errors "No messages found on ..."**: it prints every topic that
  actually exists in the bag — compare that list against what `--ns`/`--hw-node`
  produced, and pass `--command-topic`/`--feedback-topic` directly if needed.
