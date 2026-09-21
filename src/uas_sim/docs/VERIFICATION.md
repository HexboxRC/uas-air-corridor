# Local verification — September 20, 2026

Environment: ROS 2 Humble; Gazebo Sim 8.15.0, transport13, msgs10, plugin2.

- `air_cell_wind` compiled and linked against the installed Gazebo libraries.
- `uas_sim` built as ament_python with installed wind entry points, launch and config.
- 22 targeted Python tests passed: headings, ownership, wind memory, slew, dwell,
  hysteresis, occupancy, missing vehicles, stale/invalid data, pause and reset
  latch, deficits, baseline initial geometry, route rejection, world preservation,
  calibration separation, monotonic fitting and force-cap rejection.
- CTest `wind_force` passed using the compiled C++ force kernel.
- The installed launch's arguments were enumerated successfully.
- A **real single-GUI / two-PX4** no-navigation smoke test passed. Both aircraft,
  all panels, plugin and Gazebo-to-ROS observations were present. Cell 8 peaked
  at 29.6999999981 m; at simulation 40.036 s it had contracted to 21.7519999981 m.
  Cells 7 and 9 remained 20 m. Raw observations confirm grounded aircraft near
  z=0.227 m, with zero local wind/force outside the weather region.
- A separate **real headless Gazebo physics fixture** passed with two 1 kg inertial
  bodies. Calm adds zero force; an outside body remains stationary; a body inside
  the weather region accelerates while outside the initial operational width;
  resizing preserves the weather; command timeout returns wind to calm.
  This verifies physics integration, **not PX4 flight aerodynamics**.
- SHA-256 checks confirm `worlds/corridor.sdf` and `worlds/route.json` match their
  pre-implementation contents. Existing navigation and PX4 configuration sources
  were not edited. The original launch remains available.

Evidence lives in `wind_runs/smoke_result.json`, `smoke_states.json`,
`smoke_launch.log`, per-run CSV/JSONL folders, and `wind_runs/physics_smoke/`.
The smoke script overwrites its summary files on a new test; per-run observation
folders remain separate. The repeat successful run is `wind_runs/logs/20260921T005210_175961Z/`.
The initial successful log run was
`wind_runs/logs/20260921T004541_726486Z/` (UTC timestamp).

Known limitations / remaining checks:

- Both PX4 instances report **Parameter COM_DIST_MAX not found** from the existing
  airframe startup. This pre-existing PX4 configuration was left unchanged.
- Gazebo required SIGTERM after its five-second shutdown grace period; the launch
  stopped its own processes. This was during cleanup, after the assertions passed.
- No automatic takeoff or PX4 wind-disturbed flight was performed. No empirical
  aircraft envelope has been calibrated, and no safety or throughput result follows.
- The Gazebo GUI initialized and the real panel poses moved; rendered pixels and
  the optional Tk monitor were not visually inspected in this session.
- Brief force saturation is latched over the observation interval; containment
  and position measurements are still sampled, not continuous-time guarantees.
- Only Gazebo 8 was locally built and exercised. Other Gazebo versions are unverified.

Reproduction commands and remaining flight/monitor commands are in
[WIND_EXPERIMENT.md](WIND_EXPERIMENT.md). Gazebo's documented
[Link API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Link.html) defines the
world-frame center-of-mass force and velocity checks used by the plugin.
