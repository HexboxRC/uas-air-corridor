# Wind-adaptive air cell experiment

This adds localized approximate wind forces, observed-wind width control, movable
visual boundaries, and calibration logs. Admission advice is **unenforced**.
Existing `corridor_nav` does not consume it. This is not a safety validation,
collision avoidance system, or demonstration of increased traffic throughput.

## Inspected integration

`start_corridor.sh` runs `multi_drone.launch.py`. The wind launch instead includes
`multi_drone_visible.launch.py`, retaining its working visibility workaround:
preload both x500 models into a temporary world before starting **one** Gazebo
server and **one** GUI, then bind PX4 instances to `x500_0` and `x500_1`.
Namespaces remain `px4_0` / `px4_1`, starts at lane-0 indices 0 and 4, and the
existing navigation parameters and versioned PX4 topics are retained. No PX4
parameters or arming checks were changed. Original launch defaults are retained.
No applicable AGENTS.md was present. Existing dirty files were preserved.

Locally detected Gazebo **8.15.0** uses sim8, transport13, msgs10 and plugin2.
ROS is Humble. The shell used Cyclone DDS, whereas the working launch requires
Fast DDS, domain 0, ROS_LOCALHOST_ONLY=0 and no CYCLONEDDS_URI.

Actual `worlds/route.json` contains two straight parallel lanes, 45 cells per lane,
180 m center spacing, visual dimensions 200 x 50 x 50 m, 50 m lateral lane spacing,
and 120 m center altitude. JSON `yaw` is compass bearing; `gz_yaw` is ENU yaw.
The wind code derives ENU heading from actual centers (-0.9974473496 rad).
World positions and wind use ENU metres and m/s. Navigation independently converts
to PX4 NED with the existing spawn offsets.

Lane-0 indices **7, 8, 9** are selected. Each owns [-90, 90) metres along-track
and the original altitude band. Occupancy ignores lateral width, so drift cannot
make an aircraft disappear. Irregular, curved, sloping or misoriented routes are
explicitly rejected. Neighboring allocated lanes are checked by lateral and
longitudinal projections; 50 m is the largest permitted full width here.
The original static frames are retained as references; orange visual-only rails
show the applied width. No collision geometry is added to the rails.

## Build and run

From the workspace:

```bash
cd ~/uas_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select air_cell_wind uas_sim --symlink-install --cmake-args -DGZ_SIM_MAJOR=8
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
unset CYCLONEDDS_URI
```

Stop the prior simulation before launching. No process-killing cleanup is used.
Geometry-only means **no automatic takeoff**; PX4 and both aircraft still start:

```bash
ros2 launch uas_sim wind_experiment.launch.py world:="$PWD/worlds/corridor.sdf" route:="$PWD/worlds/route.json" start_navigation:=false monitor_gui:=true
```

In another terminal, source the same environment, then:

```bash
cd ~/uas_ros2_ws
ros2 run uas_sim wind_command --index 8 --crosswind 6
```

Keep the driver running. Only the middle cell's observed wind rises; its demo
required width approaches **29.7 m**, with expansion limited to 4 m/s full width.
Cells 7 and 9 remain 20 m if observations are complete and no occupants require
more space. Stopping the driver triggers calm after the 3 simulation-second
command timeout. After the 2 s recent-wind maximum and 10 s contraction dwell,
width contracts at 0.5 m/s, with 1 m hysteresis. It can settle within 1 m of the
minimum because of that hysteresis. All timing uses simulation time; wall-time
observation staleness pauses commands and closes advisory admission.

Automatic flight uses the existing navigation:

```bash
ros2 launch uas_sim wind_experiment.launch.py world:="$PWD/worlds/corridor.sdf" route:="$PWD/worlds/route.json" start_navigation:=true monitor_gui:=true
```

Choose **one** policy per launch using `cell_policy:=adaptive`,
`cell_policy:=fixed_nominal` (20 m), or `cell_policy:=fixed_worst` (50 m).
Physical wind, spawn geometry and flight control are identical across policies.
The fixed policies deliberately retain their baseline widths even when occupancy
or wind requirements exceed them; their deficits/advice remain visible.
`fixed_worst` is a baseline name, not a claim of demonstrated safety.

Other arguments: `enable_wind:=false`, `monitor_gui:=false`,
`render_engine:=ogre`, `px4_dir:=/path/to/PX4-Autopilot`,
`config:=/path/to/config.json`, `run_root:=/path/to/output`, and
`run_role:=calibration|evaluation`. World and route defaults are relative to the
launch working directory, not installed package resources. For a custom run root,
pass `--runtime /path/to/output/latest_runtime.json` to the wind utility.
The optional Tk monitor is separate from the single Gazebo GUI.

## Model and observations

For local horizontal normal n=(-sin(theta), cos(theta)), crosswind is |wind dot n|.
The DEMONSTRATION error envelope is 0.75 + 0.35 U^2 metres, vehicle radius 0.5 m,
extra margin 1 m, W_min=20 m and maximum available width 50 m. Required full width
is max(W_min, 2*(radius + envelope + margin)); it is never clipped in reporting.
The demo domain is crosswind <=8 m/s. Required, commanded and measured applied
width are separate. `width_deficit=max(0, required-applied)` stays visible during
slew and beyond available space. Missing aircraft inhibit adaptive contraction;
missing panels and stale width commands close advisory admission. Simulation
reset latches the manager stopped; restart the launch to create a new run.

The fixed weather support is 180 x 50 x 50 m with smooth 10 m edge tapers.
Operational resizing never alters it. Each cell has an independent world wind.
A 0.5 s time constant smooths commands. The force increment is:

    drag(v-wind) - drag(v), where drag(u) = -0.5*rho*CdA*|u|*u

Defaults: rho=1.225 kg/m^3, CdA=0.08 m^2, cap=5 N. Parameters are in
`config/wind_demo.json`. Force is applied at the canonical aircraft link's center
of mass using Gazebo's AddWorldForce API. This is an approximate disturbance
model, not validated x500 aerodynamics; rotor inflow and gust torques are omitted.
Calm produces zero additional force. Saturation is logged to the console and
latched across each observation interval so brief caps invalidate calibration.
Positions, xyzw orientation quaternions and velocities are post-step ground truth; force/wind are the most recently
applied pre-step disturbance. Observations are approximately 10 Hz simulation time.
Applied width is measured from actual panel world poses, not simply echoed from
ROS commands. Missing panels invalidate the measurement/advice.

Topics: `/aircells/<id>/wind_cmd` (Vector3), `/aircells/<id>/width_cmd` (Float64),
`/aircells/observations` (JSON String, Gazebo truth), `/aircells/state` (JSON String,
computed policy and explicit `admission_enforced=false`). Width-command freshness
and wind-command freshness are separate; an expired wind command intentionally
returns weather toward calm.

## Logs and calibration

Each launch writes a generated world and runtime snapshot under
`wind_runs/<UTC timestamp>/`. The manager creates a separate run folder under
`wind_runs/logs/<UTC timestamp>/` with `config.json`, `cells.csv`, `vehicles.csv`,
`observations.jsonl`, and stale/reset `events.jsonl`. The raw observations include **all** tracked aircraft,
even outside study stations; the vehicle CSV contains station-specific clearances,
world vectors, local coordinates, force and membership. Cell CSV contains width,
wind, deficit, occupancy, freshness and advisory reasons. These are sampled logs;
absence of sampled violations is not a between-sample containment guarantee.

```bash
ros2 run uas_sim wind_summary wind_runs/logs/RUN_ID
```

Use at least three independent calibration flights, multiple wind levels and
both signs, and wide fixed geometry. Tag them at launch:

```bash
ros2 launch uas_sim wind_experiment.launch.py start_navigation:=true cell_policy:=fixed_worst run_role:=calibration
ros2 run uas_sim wind_fit_envelope wind_runs/logs/CAL_1/vehicles.csv wind_runs/logs/CAL_2/vehicles.csv wind_runs/logs/CAL_3/vehicles.csv --output envelope.json
```

The fitter rejects evaluation runs, duplicate content, missing raw observations,
force-capped runs and inadequately populated bins. It creates a monotone sampled
maximum plus explicit empirical margin, records source files and observed wind /
speed domain, and warns that it is **not a formal statistical safety bound**.
Independence of flights cannot be proven from file identity; the experimenter
must use distinct flights and keep evaluation flights separate. Crosswind-only
limits also constrain alongwind and vertical wind. No unsupported extrapolation
is allowed by the policy: outside table domain, it reports invalidity and targets
maximum available width. To use a fit, copy the demo config, set controller.model
 to `table` and controller.envelope_file to the fit path, then launch with
`config:=...`. Freeze the fit before evaluation on independent flights.

## Verification and rollback

```bash
PYTHONPATH="src/uas_sim:$PYTHONPATH" python3 -m pytest src/uas_sim/test/test_wind_core.py src/uas_sim/test/test_wind_integration.py -q
ctest --test-dir build/air_cell_wind --output-on-failure
# After stopping other Gazebo sessions:
GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/air_cell_wind/lib" python3 src/air_cell_wind/test/physics_smoke.py
python3 src/uas_sim/scripts/smoke_wind.py
```

The smoke script starts one complete launch with navigation disabled, commands
only cell 8, checks both aircraft and panel observations, expansion and slow
contraction, and shuts down its own launch. It writes `wind_runs/smoke_launch.log`,
`smoke_states.json` and `smoke_result.json` (`passed=true` only on success). It does not validate
flight disturbance or visible pixels. See `VERIFICATION.md` for actual local results.

Rollback: stop the experiment with Ctrl+C, then `bash start_corridor.sh` for the
original automatic navigation launch, or
`ros2 launch uas_sim multi_drone_visible.launch.py` for the original visibility
launch without automatic navigation. Original world/route and navigation sources
were not overwritten. `enable_wind:=false` also reuses the visibility launch
without adding the wind plugin/manager.

Next stage: enforced per-cell reservations, upstream admission control, validated
transition handling and reachable occupant protection. Reactive expansion alone
cannot prevent transient deficits under an unannounced gust. Lateral tracking
error does not establish longitudinal following distance. Throughput experiments
require actual admission/scheduling changes, which are outside this implementation.
