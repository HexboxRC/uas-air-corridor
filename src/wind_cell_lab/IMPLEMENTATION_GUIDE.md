# Implementing wind-adaptive individual air cells

Prepared for the UNT corridor simulation, September 18, 2026.

## 1. The exact object that changes

The research object is an **air cell**, a managed volume of airspace with its
own geometry and traffic rules. Each cell observes local conditions, computes
its required protection, changes its operational boundary when feasible, and
eventually coordinates entry reservations with neighboring cells.

For the first implementation, change one geometric dimension: **full lateral
width**. Keep cell centers, along-route ownership intervals and altitude fixed.
This directly connects horizontal crosswind to lateral tracking error while
avoiding simultaneous changes to route topology and altitude control.

An eventual general state is:

`cell_id, revision, center, orientation, length, width, height, wind_estimate,
uncertainty, required_protection, admission_state, reservations, valid_until`.

The current source package implements the local wind field, physical disturbance,
width controller, changing Gazebo side panels, monitor and calibration logs.
**It does not implement admission enforcement or a proof of safe transitions.**
Its entry state is explicitly advisory. Existing `corridor_nav` keeps flying the
existing route. This is deliberate for the physical calibration stage; a paper
claim about traffic capacity requires the next control layer in section 10.

## 2. What the present route actually contains

The supplied route has two lanes, 45 cells per lane, centers 180 m apart,
nominal drawn dimensions 200 × 50 × 50 m, and center altitude 120 m.
The two drone starts are at lane-0 indices 0 and 4, about 720 m apart.
The complete displayed segment is approximately five miles.

Two geometric issues need explicit treatment:

1. A 200 m box every 180 m overlaps its neighbor by 20 m. A vehicle can then be
   visually inside two cells. For unique state ownership, this experiment assigns
   half-open intervals between center midpoints: `[-90, +90)` m around each
   selected center. The selected dynamic side frames are also 180 m long.
2. Lane centers are 50 m apart. Symmetrically increasing a lane-0 cell from
   50 to 80 m would consume neighboring lane-1 space. The first experiment
   constrains selected cells to a fixed available full width of 50 m.

The demonstration operating width starts at 20 m and may grow to 50 m. This is
a selected nominal experiment setting, **not a measured safe width**. Cells
7, 8 and 9 are selected by default. The middle cell receives the gust; the
others retain calm conditions unless separately commanded.

The fixed available envelope is the maximum allocated space. The dynamic cell
is the currently assigned operational volume inside that envelope. A deficit
beyond available space is an unavailable-cell condition; the code does not
pretend that clipping a requirement to the maximum has satisfied it.

## 3. Keep coordinate frames explicit

Gazebo positions and the wind driver use world ENU: east, north, up.
PX4 local position uses NED: north, east, down. The original navigation code
converts these frames and adds each drone's spawn offset. This experiment's
tracking-error logger uses Gazebo ground truth directly, avoiding an additional
EKF-origin assumption in the physical measurements.

Let a cell's ENU yaw be theta. Its along-route and leftward lateral directions are:

\[
\mathbf t=(\cos\theta,\sin\theta,0),\quad
\mathbf n=(-\sin\theta,\cos\theta,0).
\]

For wind vector w and vehicle position p relative to cell center c:

\[
w_\parallel=\mathbf t^\top\mathbf w,\qquad
w_\perp=\mathbf n^\top\mathbf w,
\]

\[
s=\mathbf t^\top(\mathbf p-\mathbf c),\qquad
e_\perp=\mathbf n^\top(\mathbf p-\mathbf c).
\]

For this straight route, distance to the cell centerline is the absolute lateral
projection, not distance to the next waypoint. Waypoint distance includes forward
progress and would give a false lateral-error measurement.

For the provided route, theta is approximately -0.99745 rad. A positive 6 m/s
crosswind is approximately ENU `(5.0405, 3.2547, 0)` m/s. The `wind_command.py`
utility computes this conversion; do not paste a PX4 NED wind vector into it.
The direction here is where air moves **toward**, not meteorological wind-from.

## 4. Make the disturbance local and physically observable

There are three separate quantities:

- **Wind command:** what the experiment asks the weather driver to produce.
- **Realized wind:** the field used by the physics plugin after its temporal
  response and spatial taper.
- **Aircraft response:** resulting position, velocity and tracking error.

Changing a ROS number without applying a force would test display logic, not
weather adaptation. Moving the aircraft pose directly would bypass PX4 control
and dynamics. This prototype instead adds a world-frame force to the aircraft's
canonical link on every physics step.

The built-in Gazebo WindEffects model is documented as a uniform worldwide wind
model with approximate forces and optional spatial force scaling. A small custom
system gives this experiment independently addressable local fields and explicit
logging. The custom system is also approximate; it is not a replacement for
validated aerodynamics or CFD. [Gazebo WindEffects documentation](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html)

### Spatial support

Each weather region has a fixed 180 × 50 × 50 m support. It does not shrink or
grow with the operational cell. Wind tapers smoothly over the last 10 m toward
each support face using `q(a)=a²(3−2a)`, where `a` runs from 0 at the boundary
to 1 inside the field. This prevents a discontinuous force at a cell boundary.
The center reading is the maximum field amplitude for this simple construction.

This local weather support is a controlled experimental assumption, not a claim
that real atmospheric wind stops at managed-cell boundaries. A subsequent study
should use a spatially correlated field crossing several cells, sampled by the
independent cell controllers.

Most importantly, the physical wind field must not change just because the cell
controller changes width. Otherwise a narrower cell could appear safer merely
because the simulator stopped applying wind outside its new boundary.

### Temporal response

The realized center wind approaches the requested vector using:

\[
\mathbf w_{k+1}=\mathbf w_k+
 (1-e^{-\Delta t/\tau})(\mathbf w_{cmd}-\mathbf w_k).
\]

The initial time constant is 0.5 s. Driver commands time out after three
simulation seconds, then the target becomes calm. Timing uses simulator time,
so pausing Gazebo does not advance the wind or contraction dwell.

### Force approximation

For air-relative velocity `v−w`, isotropic quadratic drag is:

\[
\mathbf F_d(\mathbf v-\mathbf w)
=-\tfrac12\rho C_DA\,\|\mathbf v-\mathbf w\|(\mathbf v-\mathbf w).
\]

The plugin adds the wind-induced increment relative to calm air:

\[
\Delta\mathbf F=\mathbf F_d(\mathbf v-\mathbf w)-\mathbf F_d(\mathbf v).
\]

This avoids intentionally adding a second calm-air drag term to the existing
aircraft simulation. It does not prove exact consistency with that simulation's
aerodynamic model. The force acts at the canonical link's center of mass, with
no additional gust torque. Attitude-dependent drag, rotor inflow changes,
turbulence spectra, wake effects and building aerodynamics are omitted.

Defaults: density 1.225 kg/m³, effective drag area 0.08 m², force cap 5 N.
At rest in a 6 m/s wind, the uncapped force magnitude is 1.764 N. These are
implementation parameters to investigate, not measured x500 properties.
The cap is a numerical experiment limit, not a safety guarantee. Saturation is
logged and force-capped runs must not be used for the initial envelope fit.

Gazebo applies the force in PreUpdate and publishes observations in PostUpdate.
The force record refers to the just-completed step; the position record is its
result. [Gazebo system lifecycle](https://gazebosim.org/api/sim/8/createsystemplugins.html),
[Link force API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Link.html)

## 5. Derive cell width from tracking uncertainty

For one centerline in a cell, the proposed full required width is:

\[
W_{req}=\max\{W_{min},\;2[r+E_\perp(U)+B]\}.
\]

Here r is vehicle lateral extent represented by a radius, E is the tracking-error
envelope, and B is an additional per-side margin. Multiplication by two accounts
for both sides of the centerline. It is not the two-vehicle separation calculation.

For adjacent centerlines used simultaneously, a separate non-overlap condition is:

\[
D_{ij}\ge r_i+r_j+E_{\perp,i}+E_{\perp,j}+B_{ij}.
\]

Along-route following distance has a different derivation based on speed,
relative motion, sensing/control delay, braking and disturbance response.
Do not use the crosswind width formula as an automatic longitudinal-spacing law.

### Initial demonstration model

The source starts with:

\[
E_{demo}(U)=0.75+0.35U^2\quad\text{metres},
\]

with r=0.5 m, B=1 m and Wmin=20 m. The coefficient 0.35 has units s²/m.
It is selected to exercise the software, **not derived from flight data**.

| Crosswind | Demo E | Required full width | Interpretation |
|---|---:|---:|---|
| 0 m/s | 0.75 m | 20.0 m | Nominal lower limit |
| 4 m/s | 6.35 m | 20.0 m | Lower limit still dominates |
| 6 m/s | 13.35 m | 29.7 m | Widen middle cell |
| 8 m/s | 23.15 m | 49.3 m | Near available limit |
| 10 m/s | 35.75 m | 74.5 m | Unavailable; 50 m maximum cannot meet it |

The 8 m/s configured domain limit is also a demonstration setting, despite the
configuration field name `max_calibrated_crosswind_m_s`. Nothing is calibrated
until measurements have been collected and independently evaluated.

### Calibration workflow

Use one drone initially. Fly the same straight route under calm conditions,
multiple positive and negative crosswinds, and several gust rise/fall profiles.
Log ground-truth e_perp, actual local wind, speed, force and saturation. Keep
controller tuning, aircraft mass and aerodynamic parameters fixed.

Fit the envelope from independent calibration runs, then evaluate different
runs. The included fitter bins absolute crosswind, takes the maximum absolute
tracking error in each bin plus an explicit extra empirical margin, and applies
a cumulative maximum so the table cannot decrease with wind speed.

This table is easy to inspect but is not a probabilistic guarantee. Pointwise
wind speed may be insufficient: the same speed during gust onset, recovery,
control saturation or different forward flight speed can have different error.
The two-second wind-memory window partly addresses recent exposure but does not
establish a bound on these dynamics. Record the failure cases rather than tuning
them away using the evaluation dataset.

The first fitted table limits use to its measured crosswind range and speed
range, with near-zero alongwind and vertical wind. A stronger model conditions
on forward speed, gust gradient/variance, observation age and vehicle class.
Use block maxima or flight-level splits for statistical validation: adjacent
10 Hz samples are not independent trials. Three flights is only the software's
minimum input check, not a sufficient research sample size.

## 6. Required, commanded and applied width

Always record these separately:

1. **Required:** output of the uncertainty model; never clipped to hide a deficit.
2. **Commanded:** requested geometry after limits, dwell and slew rate.
3. **Applied:** cell width accepted by the simulation plugin. Gazebo GUI rendering
   can lag this logical geometry; the width is not a measurement of a physical wall.

The prototype uses a two-second rolling maximum of measured absolute crosswind,
4 m/s maximum full-width expansion, 0.5 m/s contraction, 10 s contraction dwell,
and a 1 m full-width hysteresis band. These are initial policy settings.

When shrinking, the target must also satisfy:

\[
W_{target}\ge 2\max_{i\;in\;cell}(|e_{\perp,i}|+r_i+B).
\]

This check prevents deliberately moving the new boundary across a currently
observed occupant. It does not predict future motion and therefore does not
prove safe contraction. A full controller must test reachable occupant tubes
over the complete actuation/communication delay and transition horizon.

If the empirical model is out of range, target the largest available width and
report unavailable. If required width exceeds available width, record that
shortfall. Missing aircraft observations block contraction conservatively.
An observation gap stops slew catch-up; a stale monitor pauses commands, leaving
the plugin's last width in place. The aircraft's existing PX4 failsafes remain
independent of these advisory cell states.

### What a sudden gust demonstrates

At 6 m/s the demonstration requirement is 29.7 m. Starting from 20 m, even a
4 m/s expansion rate needs at least 2.425 s to make up 9.7 m after that final
requirement is reached. A `required_minus_applied` deficit is expected during
some transients. The software intentionally reveals it.

Rate limiting is an actuator constraint. It does not turn a physically inadequate
cell into a safe one. Avoiding this deficit requires sufficient pre-existing
reserve, preview, or a defined bound on disturbance growth that the cell can
track. A reactive controller cannot guarantee containment under arbitrary
unannounced gusts simply by publishing a larger cell afterward.

## 7. Software interfaces and ownership

| Component | Owns | Does not own |
|---|---|---|
| `wind_command.py` | Requested local ENU wind | Cell width, aircraft trajectory |
| `AirCellWind.cc` | Realized field, force increment, side-panel poses, ground-truth observations | PX4 control, safety proof |
| `cell_core.py` | Geometry projections, envelope lookup, width policy | ROS, rendering |
| `cell_manager.py` | Width commands, monitoring and CSV records | Enforced admission or flight commands |
| Existing `corridor_nav` | Offboard flight through current route | Dynamic cell geometry |
| `prepare.py` | Experimental world copy and validated cell configuration | Original world/route changes |

For each study cell:

- `/aircells/<cell_id>/wind_cmd`: ROS Vector3 to Gazebo Vector3d.
- `/aircells/<cell_id>/width_cmd`: ROS Float64 to Gazebo Double; full width in metres.
- `/aircells/observations`: Gazebo StringMsg to ROS String; JSON with simulation
  time, realized center wind, logical applied width, and vehicle ground truth.
- `/aircells/state`: ROS String; JSON with required/commanded/applied widths and
  **advisory** entry state.

The manager consumes observations, not the wind command schedule. Weather can
be scripted for repeatability without making adaptation itself scripted.
The current observation source is ideal simulator truth. A later sensor adapter
must explicitly add measurement error, sampling interval, latency and dropouts.
The evaluator should continue using independent ground truth so controller
estimation errors cannot disappear from the reported results.

## 8. How cell resizing appears in Gazebo

Each selected cell has two separate static, visual-only side-panel models. Each
panel contains upper/lower longitudinal rails and two vertical end posts. The
plugin translates them to lateral offsets ±W/2 using Model.SetWorldPoseCmd.
Moving existing panels avoids removing and recreating models at every update,
which is particularly useful given the earlier GUI scene-update trouble.

There are no collision surfaces on the cell frames: these are managed-airspace
boundaries. The drone should pass through a boundary if its dynamics demand it,
and the logger should record that as a violation. Physical walls would create
an entirely different experiment.

The companion monitor shows a top view of the three cells, actual positions,
local wind, required/applied widths and deficit. A negative body clearance is
an observed lateral containment violation. A model margin deficit is different:
it can occur even when the vehicle happens to remain near the centerline.

## 9. What to measure first

The first successful run should establish all of the following:

1. A command to cell 8 produces nonzero realized wind there and zero readings
   in cells 7 and 9.
2. A drone outside the weather support receives zero force from this plugin.
3. Upon entering the middle weather region, force and aircraft response change.
4. Cell 8 widens from observations; cells 7 and 9 remain nominal unless an
   occupant or data-validity condition independently requires expansion.
5. Wind removal causes delayed, slower contraction.
6. `required_minus_applied` exposes transient deficiencies instead of hiding them.

Use the logs, not only visual impressions. A well-tuned aircraft may hold the
centerline closely even under a force; a small deviation is a result, not a
reason to increase artificial drag until the paper looks dramatic.

Log interpretation:

- `body_clearance_m = applied_width/2 − |e_perp| − radius`.
- `model_clearance_m = applied_width/2 − (radius + E + extra_margin)`.
- `required_minus_applied > 0` means the geometric requirement is unmet.
- `force_limited=true` means the simulator has clipped the force; inspect or
  exclude that run before treating it as a wind calibration point.

The logger samples at approximately 10 Hz of simulation time. It cannot rule
out violations between samples. The first monitor is lateral-only; it reports
vertical offset and altitude-band membership but is not a full 3-D safety monitor.
Research runs should add physics-rate extrema/event logging for complete exposure
durations and transient peaks before claiming no separation/containment violations.

## 10. Next layer: enforce cell transitions and admission

The following is a design for the next implementation, **not code already present
in this prototype**. It is what turns the geometry experiment into airspace
management with a measurable traffic-capacity effect.

### Shared reservations

For each cell maintain current occupants and future reservations, each with a
vehicle identifier, expected entry/exit time, reachable spatial tube and geometry
revision. Allow entry only when the downstream reservation has been granted and
the interface between cells is feasible over the transition horizon.

Geometry revisions should use a prepare/validate/commit process:

1. Estimate the required envelope from local observations and uncertainty.
2. Propose a new geometry within the fixed allocated airspace.
3. Check all current and committed future occupant tubes against it.
4. Check adjacent-cell non-overlap and connected entry/exit interfaces.
5. Stop granting incompatible new reservations immediately.
6. Coordinate a feasible trajectory or holding plan for affected aircraft.
7. Commit the geometry revision when those conditions are met, or record a
   deficit/unavailable state if the disturbance has already invalidated them.

Do not delay the *reported physical requirement* until it is convenient to
commit. A revision protocol cannot erase an already-existing wind-induced risk.

### Upstream response

If cell B has reduced admission, cell A must reduce grants into B before aircraft
reach the boundary. Holding locations and deceleration space must exist in A.
Stopping a waypoint index alone does not stop a moving drone at a boundary.
For an initial one-dimensional check, the remaining distance must exceed a
conservative stopping distance such as `v*t_delay + v²/(2*a_brake)` plus uncertainty.
Both delay and attainable deceleration must be measured or conservatively bounded.

The complete approach must account for the leading and following aircraft; a
lead drone's sudden stop can cause a rear-end conflict. If no feasible hold exists,
use an already-defined alternative route or operational restriction and count
that outcome in the experiment. Do not improvise a last-moment stop as a proof.

### Make the capacity mechanism explicit

Wider cells do not automatically lower throughput. For an isolated lane with the
same speed and following distance, width adaptation may have no throughput effect.
Two possible research mechanisms are:

- A fixed total corridor width can hold fewer parallel operational cells/lanes
  when each needs more lateral protection. Lane retirement must be coordinated
  with current occupants and future entries.
- Joint reachable-volume and time reservations reduce allowable concurrent cell
  occupancy under wind. This needs a longitudinal/temporal uncertainty model as
  well as a lateral-width model.

Choose and implement one explicitly. The present two-drone, fixed-centerline
experiment can establish local physical response and dynamic geometry, but
cannot establish a high-density network-capacity improvement.

## 11. Evaluation that supports a paper

Use a separate calibration set, policy-tuning set if needed, and untouched test
set. Freeze the final uncertainty model and controller settings before test runs.
Test the same arrival demand and disturbance realizations across all policies.
Use the same independent ground-truth safety evaluator for every policy.

| Policy | Purpose |
|---|---|
| Fixed nominal dimensions | Efficiency reference under nominal conditions |
| Fixed dimensions for a declared worst-case envelope | Conservative allocation reference |
| Adaptive dimensions without transition coordination | Ablation exposing transition failures |
| Adaptive dimensions with transition coordination | Proposed full method |

In this source, `fixed_worst` means the configured 50 m maximum; it is only
worst-case-safe if calibration/validation establishes adequacy for the declared
operating envelope. A run beyond that envelope is a stress test, not evidence
that the supposedly worst-case baseline was incorrectly designed.

Vary gust direction, rise time, duration and location; observation delay and
dropout; traffic demand; and representative vehicle properties. A useful initial
matrix includes calm, sustained crosswind, a single ramp, repeated gusts, gust
reversal and a spatial patch spanning multiple cells. Select repetition counts
based on variability and desired confidence, not a convenient round number.

Report:

- Measured containment/separation violation count, depth and exposure duration.
- Minimum measured and predicted margins, including transition periods.
- Requested versus allocated cell dimensions and airspace-volume use over time.
- Completed trips per simulation minute, waiting-time distribution and rejected
  admissions **once reservation enforcement exists**.
- Frequency and duration of unavailable-cell states, including cases exceeding
  the declared wind envelope.
- Sensitivity to sizing margin, contraction dwell and expansion capability.
- Runtime and simulation real-time factor separately from traffic performance.

Do not promise that the adaptive curve will dominate both baselines. That is
the hypothesis. Negative results can identify the preview/reserve needed for
adaptation to be useful, or show that width is not the actual capacity constraint.

Suggested focused research question:

> Under bounded wind uncertainty, can locally resized air cells with coordinated
> transitions reduce reserved airspace or improve admitted traffic while meeting
> a specified containment and separation criterion?

Existing wind/separation work means novelty must be assessed against specific
cell geometry and transition-management methods, not the broad use of wind.
[NASA: Separation at Crossing Waypoints Under Wind Uncertainty](https://ntrs.nasa.gov/citations/20210017342)

## 12. Files and implementation status

| File | Role |
|---|---|
| `config.json` | Study cells, physical parameters, demo sizing and slew settings |
| `prepare.py` | Validate geometry and generate experiment world/runtime data |
| `plugin/AirCellWind.cc` | Gazebo field, forces, movable boundaries and truth output |
| `plugin/wind_math.hh` | Independently tested force and spatial-taper calculations |
| `plugin/CMakeLists.txt`, `build.sh` | Match the plugin build to Gazebo's major version |
| `cell_core.py` | Dependency-free width controller and coordinate transforms |
| `cell_manager.py` | ROS bridge consumer, live cell monitor, sizing outputs and CSV |
| `wind_command.py` | Local signed crosswind commands for independent experiments |
| `cell_wind.launch.py` | One Gazebo GUI, preloaded drones, PX4, bridges and manager |
| `fit_envelope.py` | Empirical monotone calibration table |
| `summarize.py` | Sampled containment and deficit summary |
| `tests/` | Executable mathematical regression checks |

The policy, force kernel, generation and launch wiring have been checked locally.
The full Gazebo plugin and flight integration remain to be built and tested on
your WSL machine. ROS Humble does not by itself identify your Gazebo major version;
use `gz sim --versions` and build for the matching server.

This package preserves your original world, route and working navigation. The
first milestone is observable **local wind → aircraft response → individual cell
resizing**, with honest transient deficits. Then add enforced reservations and
network-level capacity experiments using the calibrated model.
