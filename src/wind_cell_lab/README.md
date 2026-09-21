# Wind-adaptive individual air cells

This is the **wind, cell-geometry and calibration stage** for your existing
ROS 2 / PX4 corridor. It resizes specific cells and applies a local disturbance
to aircraft in Gazebo. It does **not** enforce entry reservations or demonstrate
traffic capacity. The supplied sizing coefficients are explicitly **DEMO**.

Read `IMPLEMENTATION_GUIDE.md` for the mathematics, limitations and research plan.

## What changes

- Original `corridor.sdf`, `route.json`, `uas_sim` and PX4 files are preserved.
- `prepare.py` creates `run/corridor_wind.sdf` from your original world.
- Lane-0 cells 7, 8 and 9 get movable orange side frames. Only their full width changes.
- Initial width is 20 m; maximum permitted width is 50 m. Length ownership is 180 m;
  height remains 50 m at the original 120 m altitude.
- Wind defaults to zero everywhere. You command the middle cell separately.
- Both aircraft are preloaded before the single Gazebo GUI starts, as in the
  visibility launch that worked for you.
- Existing installed `uas_sim/corridor_nav` controls flight. This package only
  sends commands for cell visuals and wind, not aircraft trajectories.

## Validation status

The pure Python policy tests, world-generation checks, launch construction checks,
Python syntax, shell syntax and compiled C++ force-kernel tests were run locally.
The full Gazebo plugin could not be compiled or exercised here because Gazebo,
ROS and their development libraries are absent. Its source targets the documented
Gazebo Sim 7/8/9 APIs; the first WSL build and runtime test are still required.

## 1. Put the folder in WSL

Extract the download so these paths exist:

```text
~/uas_ros2_ws/wind_cell_lab/README.md
~/uas_ros2_ws/wind_cell_lab/config.json
~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py
~/uas_ros2_ws/wind_cell_lab/plugin/AirCellWind.cc
```

This folder sits at the workspace root. Your existing world and route stay in
`~/uas_ros2_ws/worlds/`. Your ROS package stays in `~/uas_ros2_ws/src/uas_sim/`.

To extract a ZIP downloaded through Windows, substitute the actual Windows
username and download filename:

```bash
unzip /mnt/c/Users/YOUR_WINDOWS_USERNAME/Downloads/wind_cell_lab.zip -d ~/uas_ros2_ws
```

Alternatively, run `explorer.exe .` from `~/uas_ros2_ws` and copy the extracted
`wind_cell_lab` folder there. Do not put another `wind_cell_lab` folder inside it.

## 2. Identify your Gazebo version and compile

```bash
gz sim --versions
cd ~/uas_ros2_ws/wind_cell_lab
```

Use the major version used by your working simulation:

| Version | Build command | Development package, if missing |
|---|---|---|
| 7.x, Garden | `bash build.sh 7` | `libgz-sim7-dev` |
| 8.x, Harmonic | `bash build.sh 8` | `libgz-sim8-dev` |
| 9.x, Ionic | `bash build.sh 9` | `libgz-sim9-dev` |

For example, **only if your Gazebo is 8.x**:

```bash
sudo apt install build-essential cmake libgz-sim8-dev python3-tk
bash build.sh 8
```

Use the matching version package in the existing Gazebo apt repository. Do not
install a different Gazebo release to satisfy this example. The build requires
matching transport, messages and plugin development dependencies. A missing apt
package or CMake dependency is a setup problem, not a reason to mix major versions.
Gazebo 6/Fortress is not covered by this source and needs an Ignition API port.

## 3. Prepare the world

```bash
source /opt/ros/humble/setup.bash
source ~/uas_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
cd ~/uas_ros2_ws/wind_cell_lab
python3 prepare.py
```

To use other existing locations:

```bash
python3 prepare.py --world /absolute/path/corridor.sdf --route /absolute/path/route.json
```

Run `prepare.py` again after changing `config.json` or moving this folder: the
generated files contain absolute resource paths. Stop the experiment before
regenerating them. There is no `colcon build` step for this standalone folder.

## 4. Check the geometry without automatic takeoff

Stop your previous launch using Ctrl+C. Start one simulation:

```bash
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py \
  2>&1 | tee /tmp/cell_wind.log
```

Navigation defaults to **false**. The new cell monitor should display three
cells, their wind readings, required width and applied width. In Gazebo, look
for `cell_L0_0008_side_left` and `cell_L0_0008_side_right` in the Entity Tree.
Those visual-only side frames move apart and together. They cannot collide
with the drone. Other corridor cells remain the original static reference.

If your working visibility setup required Ogre, append `render_engine:=ogre`.
If Tk is unavailable, append `monitor_gui:=false` and inspect `/aircells/state`.
Close the whole launch, not just the monitor, when stopping this experiment.

In a second terminal, source the same ROS environment and set the same Fast DDS
environment variables as in step 3. Then:

```bash
cd ~/uas_ros2_ws/wind_cell_lab
python3 wind_command.py --index 8 --crosswind 6
```

Leave the command running. It keeps commanding a 6 m/s crosswind in cell 8;
actual wind approaches that value with a 0.5 s time constant. Cells 7 and 9
remain calm. The middle cell's DEMO required width approaches 29.7 m, and its
orange sides move outward at the configured width-change rate. The drone is
still on the ground, so this step verifies geometry, **not** flight response.

Ctrl+C stops the driver. If a calm message cannot be sent during ROS shutdown,
the plugin's 3 simulation-second command timeout requests calm automatically.
After wind memory and contraction dwell expire, width contracts slowly.
Never run two wind drivers on the same cell simultaneously.

## 5. Test physical wind during flight

Stop the launch and restart with automatic flight enabled:

```bash
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py start_navigation:=true \
  2>&1 | tee /tmp/cell_wind.log
```

In your second terminal, run the wind command again, initially with a small
crosswind such as 2 m/s. Follow `x500_0` using Gazebo's Camera Tracking plugin.
Verify both `armed=True` and `offboard=True` in the navigation logs.

The default cell indices are zero-based: the localized test is in lane 0,
indices 7–9. With the existing spawns at indices 0 and 4, aircraft must travel
downstream before they reach it. A stationary drone outside those cells should
feel no wind from this plugin. The monitor only draws drones within the three
study stations, so an empty monitor initially is expected.

You should see `[AirCellWind] force attached to x500_0` and the same for x500_1.
The cell monitor shows actual Gazebo positions, not positions computed from a
scripted trajectory. Position data from the original route viewer can still be
used separately, but do not start another simulator.

Inspect bridge output if observations are absent:

```bash
gz topic -l | grep /aircells
ros2 topic echo /aircells/observations --once --qos-reliability best_effort
tail -n 80 /tmp/cell_wind.log
```

The ROS bridge must support String/StringMsg, Vector3/Vector3d and Float64/Double.
An absent Gazebo topic indicates a plugin/server issue; a present Gazebo topic
with no ROS messages indicates a bridge or ROS discovery issue.

## 6. Read results and fit an empirical envelope

Each launch creates `logs/<UTC-run-id>/config.json`, `cells.csv`, and `vehicles.csv`.

```bash
python3 summarize.py logs/YOUR_RUN_ID
```

Use **independent flights** across multiple wind levels and both signs, including
gust onset and recovery. For the first calibration, use a single drone by setting
`"drones": 1` in config and preparing again. Use the wide fixed policy during
calibration to avoid confusing demo cell resizing with observed tracking error:

```bash
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py \
  start_navigation:=true cell_policy:=fixed_worst
```

The supplied fitter requires at least three distinct run files and enough data
in every wind bin. Do not duplicate one file to satisfy this requirement.

```bash
python3 fit_envelope.py \
  logs/RUN_1/vehicles.csv logs/RUN_2/vehicles.csv logs/RUN_3/vehicles.csv \
  --output envelope.json
```

The example paths are placeholders. A bin with insufficient samples is a request
for additional calibration, not permission to invent an error bound. Force-capped
runs are rejected as a whole, since saturation affects subsequent motion too.

To use the fitted table, edit `config.json` under `controller`:

```json
"model": "table",
"envelope_file": "envelope.json"
```

Preserve the other fields, run `prepare.py`, and restart the launch. Evaluate
on different wind realizations and independent flights. Table validity is limited
to measured crosswind, speed, and the documented near-zero alongwind/vertical-wind
conditions. It is not a bound for arbitrary gust gradients or actuator saturation.

## 7. Compare geometry policies

Run separate experiments with the same initial state, flight route and wind input:

```bash
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py start_navigation:=true cell_policy:=fixed_nominal
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py start_navigation:=true cell_policy:=fixed_worst
ros2 launch ~/uas_ros2_ws/wind_cell_lab/cell_wind.launch.py start_navigation:=true cell_policy:=adaptive
```

These are alternatives, not simultaneous commands. They compare width allocation
and containment. Because the aircraft controller and admissions are unchanged,
this stage cannot establish a traffic throughput improvement. That requires the
reservation manager described in the implementation guide.

## Rollback

Stop this launch and run your existing `multi_drone_visible.launch.py` again.
The original `uas_sim` package, route, world and PX4 parameters were not edited.
