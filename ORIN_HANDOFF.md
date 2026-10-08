# Resume on the Jetson Orin

Snapshot prepared October 7, 2026. The implementation was already committed as
`ff25a77` (updated wind based navigation). This handoff records the current local
dependencies; SETUP_NOTES.md describes an older Orin environment.

## Current dependency state

- ROS 2 Humble and Gazebo Harmonic (Gazebo Sim 8; previously tested with 8.15.0).
- PX4 repository: https://github.com/PX4/PX4-Autopilot.git
- PX4 commit: `6dc89338a8b3947f232525594459a749ef2002e1`.
- PX4 has one local tracked change: `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500`.
  Its complete current contents are preserved in
  `px4_config/airframes/4001_gz_x500_current_snapshot`. This snapshot differs from
  the older `px4_config/airframes/4001_gz_x500`; neither was applied to another machine.
- px4_msgs repository: https://github.com/PX4/px4_msgs.git
- px4_msgs commit: `dfdaf94cb574e4ec344753c1eb83aa1560f069b2`; local tree clean.
- MicroXRCEAgent is installed locally at `/usr/local/bin/MicroXRCEAgent`; its version
  has not been pinned. The Orin also needs the agent, ros_gz_sim, ros_gz_bridge,
  Fast DDS, Gazebo 8 development libraries, and python3-tk for the optional monitor.

## Retrieve the workspace

If the Orin already has the repository, preserve any local edits, then:

```bash
cd ~/uas_ros2_ws
git pull --ff-only origin main
```

For a fresh workspace:

```bash
git clone https://github.com/HexboxRC/uas-air-corridor.git ~/uas_ros2_ws
cd ~/uas_ros2_ws
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
git -C src/px4_msgs checkout dfdaf94cb574e4ec344753c1eb83aa1560f069b2
```

For an existing px4_msgs checkout, inspect its local changes before selecting that
commit. PX4 itself lives outside this workspace, normally at
`~/Desktop/PX4-Autopilot`. Reproduce the pinned commit and submodules in a separate
checkout if the existing Orin installation must remain usable. Compare the saved
airframe with the Orin airframe before choosing to apply it; retain a backup.
Build PX4 SITL on the Orin using that checkout's build instructions. Do not copy
x86-64 executable files or shared libraries from this machine to the ARM64 Orin.

## Build on the Orin

Use a fresh build/install directory on the Orin; do not reuse another machine's
build artifacts. Once the native dependencies and PX4 are ready:

```bash
cd ~/uas_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs uas_interfaces air_cell_wind uas_sim --symlink-install --cmake-args -DGZ_SIM_MAJOR=8
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
```

Initial wind test without automatic takeoff:

```bash
ros2 launch uas_sim wind_experiment.launch.py start_navigation:=false monitor_gui:=true
```

In a second terminal with the same ROS environment:

```bash
cd ~/uas_ros2_ws
ros2 run uas_sim wind_command --index 8 --crosswind 6
```

Enable flight by restarting with `start_navigation:=true`. For no-wind flight:

```bash
ros2 launch uas_sim multi_drone_visible.launch.py start_navigation:=true
```

Only run one simulation launch at a time. Override `px4_dir:=/your/PX4/path` if
needed. See `src/uas_sim/docs/WIND_EXPERIMENT.md` and `VERIFICATION.md` for details.

## What Git carries

The source packages, original worlds and route, configuration, launch files,
reference implementation, existing visual assets/backups, and documentation are
tracked. `build/`, `install/`, `log/`, `wind_runs/`, and the independent px4_msgs
checkout are deliberately ignored. Generated logs are not needed to resume coding
or run the simulation; transfer `wind_runs/` separately if you need experiment
history. PX4, ROS/Gazebo installations, and MicroXRCEAgent are external dependencies.

Known prior issue: PX4 startup reported `COM_DIST_MAX` not found. No parameters
were changed for this handoff. Wind aerodynamics remain approximate; admission
is advisory and flight calibration remains necessary.
