# Three-robot office coordination experiment

The hypothesis to test is: earlier delivery of useful peer observations changes
a robot's destination before it repeats a teammate's exploration, improving
team physical coverage per metre travelled. Improved received-map coverage is
an intermediate result, not the physical exploration outcome.

This protocol is a proposed pilot, not a validated office result. No simulator
runs were performed while preparing it. Keep transport performance and mission
performance as separate outcomes; a positive mission effect is not guaranteed.

## Frontier scoring fix

Both robot configurations now use `selection_strategy: visible_gain`. It
samples up to five reachable cells along each frontier and scores each by
visible unknown square metres / (1 + gain_distance_weight * path metres).
Rays stop at known lethal obstacles, including diagonal wall corners, and
stay within `sensor_range_m` (currently 3 m). Unknown space is optimistically
transparent, so this is a visibility-based area estimate, not exact expected
laser returns. Each viewpoint is scored independently, without a region cap
or competition for credit from neighboring candidates.

The selected goal is an actual reachable frontier cell. New peer observations
immediately reduce its unknown-area score. In-flight switching compares the
same score at the current destination and requires a 25% utility improvement,
plus the existing separation and preemption-streak guards. It does not cancel
an active goal solely because all frontiers disappeared; persistent obsolete
goal cancellation remains separate work. Scores and the actual active goal
are logged every nine seconds. Reachability is the existing grid BFS proxy;
Nav2 still validates and executes the path.

For the original-policy control, explicitly set `selection_strategy:
nearest_high_gain`. Region/gain-threshold, turn-penalty, directional-hysteresis,
and distance-improvement tuning below concern that legacy policy. The new
policy uses `sensor_range_m`, `gain_distance_weight`, `gain_max_viewpoints`,
and `goal_preempt_utility_ratio`. Use identical settings in all transport arms.

## Findings in the original policy

- `lite_frontier_explorer` reads each robot's global costmap. Peer data reaches
  it through incoming maps -> team fusion -> local-map compositor -> Nav2.
  Local known cells override team cells; local unknown cells retain peer data.
  This is a real causal path for sharing to affect exploration.
- The default selector is `nearest_high_gain`. Connected unknown regions are
  capped at 2,000 cells: only 5 square metres at 0.05 m resolution. Several
  large frontiers can therefore have identical gains. Unknown connectivity
  also does not identify architectural wings: unknown space can connect
  through walls that have not yet been observed. Increasing the cap alone
  does not solve this.
- The 6 m directional bonus produces up to a 12 m preference for continuing
  forward over reversing. This can dominate the small differences in travel
  cost that sharing creates.
- An active goal is only preempted if the new goal is at least 2 m closer in
  Euclidean distance, and preemption stops after three consecutive switches.
  There is no separate check that the current frontier became explored. A
  disappeared frontier can therefore remain the destination even when the
  next useful frontier is farther away, or no frontier remains.
- The launch allowlist previously discarded selection/gain/hysteresis and
  wedge settings. That forwarding bug is fixed alongside this document.
  Existing default values are unchanged.
- The office `distributed` preset is actually three nearby positions at
  x=14.1 and y=4.5, 1.5, 7.5. Do not infer dispersed wing starts from its name
  or trust the older ring-position comment. The office preset documentation
  also warns that the supplied map image differs from collision geometry.

## First pilot: delayed deployment at a branch

Use one scout and two later departures. Place the waiting robots on clear
floor near a branch where one alternative leads into the scout's wing and
another into still-unexplored space. They must not obstruct the scout or see
most of its wing while stationary. Start with the existing office positions,
but verify these properties in the actual world before treating the layout
as suitable. Do not invent coordinates from the office image.

The scout explores normally. At a fixed, predeclared delay the second robot
begins exploration; the third begins after another equal delay. With no
sharing, a follower should still see a plausible frontier into the scout's
region. With unimpaired sharing, that region should be sufficiently mapped
that another destination wins. A distant peer update with no effect on a
choice is not a useful test event.

Pilot `EXPLORE_START_STAGGER_S` values of 0, 60, and 120 seconds, giving
release offsets (0,0,0), (0,60,120), and (0,120,240). These are starting
values, not calibrated timings. The implementation times each delay from
that robot's first costmap/TF readiness, not a shared mission epoch or Nav2
action-server readiness. Record actual release times. For final experiments,
use a common simulation-time readiness barrier and prescribed release times.
All mapping/communication stacks stay running during the hold.

Choose the delay using only the none/oracle pilot: find a reproducible
decision at which the oracle map removes a redundant wing visit. Freeze the
layout and delay before comparing codecs; rotate robot/scout roles and use
separate evaluation runs. Do not select a delay by which codec wins.

For the first pilot use the existing policy. If it cannot distinguish the
two map states, try `hysteresis_bonus_m: 1.5` in the applicable existing YAML,
holding every other policy parameter identical across transports. Compare
against the original 6.0 setting and watch for oscillation. Do not remove
the closer-goal guard globally: it exists to prevent observed goal thrashing.
The simulation launch chooses `config_visit_once.yaml` for
`turtlebot3_waffle`, and `config.yaml` otherwise. Verify effective parameters
using `ros2 param dump /robot2/lite_frontier_explorer`.

If a valid destination becomes obsolete during travel, a subsequent policy
change should handle *loss of expected observation gain* independently of
ordinary distance-based switching. Require persistence across fresh map
updates, use a sensor-sized observation region, and cancel without
blacklisting the destination. A missing centroid or one inflated costmap
snapshot is insufficient evidence. Test this policy under every transport,
including none. This behavior is a proposed follow-up, not implemented here.

## Transport conditions

Begin with `none` and `oracle`. In `experiment.conf`, set:

```sh
NUM_ROBOTS=3
IMPAIRMENT_MODE=sim
LINK_PROFILE=static
BANDWIDTH_KBPS=0
LOSS_PCT=0
DELAY_MS=0
RECORD_METRICS=true
EXPLORE_START_STAGGER_S=60
```

Then run the existing sweep launcher for a smoke pilot:

```sh
python3 tools/run_experiment_sweep.py --world office --methods none,oracle --min-runs 3 --target-runs 3 --duration 900
```

The launcher edits/restores configuration; check existing settings rather
than relying on exported overrides, since `docker.sh` sources experiment.conf.
Three runs check feasibility, not statistical significance. The runner's
duration is wall time measured after every robot sends its first goal;
900 here is a smoke-test limit, not a common simulation-time mission horizon.
Also, a robot having no useful goal after sharing is not necessarily a
bringup failure. Final validity checks must use stack readiness, not require
that every healthy robot emits a goal.

Only after a reproducible none/oracle separation, compare
`none,oracle,baseline,zstd,vxch` at fixed capacities, initially 15, 30, and
60 kbps. Calibrate these against actual offered load and delivery delay;
the load figures in experiment.conf refer to an earlier implementation/run.
`vxch` is the launch arm name; there is no separate `wavestream` arm in the
current launch validation. Record codec revision and options with each run.

Target the regime where useful-map delivery for one transport precedes the
branch decision and another follows it. Do not achieve this by disabling the
baseline's delivery outright. A sufficient-capacity condition should show
whether transports converge. Add the time-varying link condition only after
the static causal mechanism is established; a static bottleneck can already
produce a coordination effect.

For three robots, simulation creates six independent directed links. At
30 kbps each, that is a nominal 60 kbps incoming budget per robot and
180 kbps across the team, not a shared 30 kbps radio channel. Report that
model explicitly, or implement a shared scheduler before making shared-radio
claims. The software model charges serialized messages and drops whole
messages, not network packets; keep loss zero in the initial experiment.

The proxy token bucket uses steady wall time while exploration uses ROS
simulation time. If real-time factor is r, a C kbps wall-time link supplies
approximately C/r kbps per simulated second. Different simulation speeds
can therefore change both effective communication and distance travelled
under wall-time cutoffs. Log real-time factor and use a common simulation
clock for link service and mission horizons before the final comparison.

Baseline/zstd traffic has no latest-map replacement in the proxy and is
exempt from band queue shedding. VXCH bands have different priority/shedding
behavior. Thus the current comparison measures a complete transport stack,
including queue policy. Include a competent latest-pending-snapshot baseline
if attributing the effect specifically to representation/compression; finish
in-flight transmissions so replacement cannot starve large snapshots.

## Outcomes and evidence

Use a common evaluation grid and fixed reachable/observable interior mask
derived independently from the collision world or a reference survey, never
the best map from a competing arm. Report absolute square metres too.

Primary outcomes: time and total team distance to fixed physical coverage
thresholds (for example 80% and 90%), plus unique physical coverage at a
common simulation-time horizon. Keep failures to reach thresholds censored;
do not silently discard them. Include scout work and deployment delays in
mission totals, and report post-release improvement separately.

Use the union of raw local `/robotN/map` observations for physical coverage,
not the union/sum of `/nav_map`. The comparison script already has local
union and redundant-coverage machinery. Audit alignment, rasterization,
loop-closure effects and exterior cells before interpreting it as area.
Redundancy can be summarized as sum of per-robot observed areas minus their
union, but shared-start sensing and necessary corridor transit are not all
avoidable waste. Also count repeated wing entries and distance spent there
after a teammate had already surveyed the region.

For each branch decision record: robot pose/time, candidate frontier
locations, gains/costs, selected and active goals, local and received maps,
and when the relevant peer observation reached the planning input. Replay
the same pose and policy state with local-only, actual-received, and
unimpaired-peer maps. This offline diagnostic isolates whether information
changes the choice; it does not replace closed-loop mission evaluation.

The desired evidence chain is: earlier useful update -> fewer obsolete
targets/repeated wing visits -> less redundant travel -> earlier physical
coverage. Also report goal failures and recovery time so navigation stalls
cannot hide the mechanism. Received-map PSNR/known area alone cannot show it;
coarse maps must preserve unknown-space boundaries and useful connectivity.

Use paired layouts, release schedules and link seeds, rotate roles, vary
seeds across evaluation blocks, and report paired differences with confidence
intervals. A DDIL RNG seed alone does not make Gazebo/SLAM deterministic.
Treat simultaneous departures as a separate generalization condition.

Perfect sharing is an information reference, not a mathematical performance
upper bound: independent robots can still chase the same fresh frontier.
If that dominates, study a common coordination policy with target allocation
as a separate factor and account for its messages equally across arms.
The established cost/utility coordination approach explicitly addresses
overlapping robot objectives: [Burgard et al., Coordinated Multi-Robot
Exploration](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/burgard05tro.pdf).
Do not assume improved historical map sharing also communicates future intent.
