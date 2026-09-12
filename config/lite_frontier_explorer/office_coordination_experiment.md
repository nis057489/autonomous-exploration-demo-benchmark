# Office occupancy-sharing experiment

## Current configuration

Keep independent visible-gain exploration. Each robot selects a reachable
viewpoint using unknown area visible in its own fused occupancy map divided by
travel cost. Only received occupancy updates can tell it that another robot
has explored an area. Known walls stop visibility and constrain route estimates.
Goal reservations, reservation relays and static rank assignment are disabled
in the experiment launch. Legacy reservation code remains available for isolated
regression tests; historical recording/plot support remains intact.

`experiment.conf` selects `SPAWN_PRESET=shared_hall`, `NUM_ROBOTS=3`, and
`EXPLORE_START_STAGGER_S=60`. World selection remains the launch argument:

```sh
./docker.sh office
```

Current transport settings are preserved. Use identical spawn and stagger
settings across none, oracle, baseline, zstd and vxch.

## Geometry and starts

Use [the generated collision layout](../../simulation/worlds/office/office_start_layout.svg),
not the shipped office map image, which does not match the world geometry.
The current collision extraction includes 438 volumes and a connected interior
of approximately 637.4 m² at z=0.10 m. This is not a sensor-observable coverage
denominator: it is a static floor slice. Several apparent rooms are solid
collision blocks.

The western side contains long parallel aisles with cross-corridor connections;
the east includes rooms and branches. This is suitable for studying repeated
traversal and redundant room/aisle observation, but it is not a pure tree of
long dead ends. Loops allow alternative routes and can weaken communication
sensitivity compared with a deliberately branching test map.

| Robot | x (m) | y (m) | yaw | Additional ready-time delay |
| --- | ---: | ---: | ---: | ---: |
| robot1 | -3.50 | 2.00 | pi (west) | 0 s |
| robot2 | -1.00 | 2.00 | pi (west) | 60 s |
| robot3 | 1.50 | 2.00 | pi (west) | 120 s |

All start in the same southern hall, facing the western circulation spine.
They are 2.5 m apart, with the first robot ahead of the waiting robots. This
avoids allocating separate wings through spawn placement. Distant aisles are
outside the configured 3 m gain horizon at startup; actual sensor visibility
and subsequent destination choices must still be checked in a pilot.

Reproduce the static validation and figure with:

```sh
python3 simulation/worlds/office/inspect_experiment_starts.py
```

Checks: all starts connect to four western aisle probes and an eastern probe
in the floor grid with 0.35 m obstacle inflation; starts have no padded-body
collision over sampled heights 0.1–0.6 m. The configured rectangular footprint
has a circumscribed radius of approximately 0.291 m. These checks use the
existing primitive-collision extractor and do not replace a full simulator
rollout, footprint sweep, or validation of unsupported collision shapes.

## Stagger wiring and clock semantics

The setting passes through experiment.conf -> docker.sh environment -> launch.sh
-> multi_robot_vxch_experiment.launch.py -> multi_robot_frontier_explorer.launch.py
-> each explorer's `explore_start_delay_s = index * stagger`.

All robots spawn and continue sensing, SLAM and communication during the hold.
Only frontier goal issuance is delayed. A delayed explorer starts its timer
after its costmap, transform, configured occupancy input and Nav2 action server
are ready. It uses ROS simulation time, so pausing the simulator does not consume
the delay. Readiness and release times are logged. A zero stagger disables the
hold. Release is checked on the 3 s planning tick.

These are per-robot readiness-relative delays, not exact offsets from a shared
mission epoch. Bringup differences therefore affect absolute release times.
Record those times and keep elapsed coverage curves measured from run start;
do not subtract the hold. For strict synchronized deployment comparisons, a
common readiness barrier remains future work.

## Validation and interpretation

First run none and oracle pilots to verify collision-free departure and useful
branch decisions; use a fixed duration that includes the holds. The sweep
runner currently starts its duration after every robot sends its first goal,
so that duration is not a common mission-time horizon. Use a common recorded
simulation-time window for analysis and do not treat an idle but healthy robot
as failed merely because shared knowledge leaves it no useful goal.

Freeze starts and delay before codec comparisons. Include simultaneous-start
trials and rotate start order in repeated trials. Report time to fixed physical
coverage, repeated observations, and travel distance separately from received
map knowledge. Do not require unimpaired sharing to win each realization.

No Gazebo pilot was executed for this configuration change. The static geometry
and release/launch regressions pass; communication-dependent exploration
performance has not yet been validated on these starts.
