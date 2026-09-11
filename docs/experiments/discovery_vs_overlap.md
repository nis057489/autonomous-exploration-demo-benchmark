# First discovery versus overlap

Physical coverage means the cumulative area of cells first observed by **any**
robot: `U(t) = cell_area * |union_i observed_i(t)|`. Its rate is the increase
in U over a fixed time window. A received map is never a new physical
observation. The current comparison figure already uses this definition,
approximated by raw local SLAM maps transformed into a common frame. It counts
known free **and occupied** cells, and is subject to SLAM registration error;
it is not a ground-truth floor-coverage percentage.

## Latest maze2 pilot

Bags: `20260911_220359_none_maze2` and
`20260911_215744_oracle_maze2`, three robots, distributed starts. These are
single trials, not evidence of a population correlation. Values below use
unchanged bag extraction and elapsed bag-recording time.

| At 300 seconds | None | Oracle |
|---|---:|---:|
| Team first discovery U (m²) | 487.86 | 500.67 |
| Sum of each robot's own distinct observations S (m²) | 807.90 | 760.08 |
| Cross-robot overlap R = S − U (m²) | 320.05 | 259.41 |
| U / S | 60.4% | 65.9% |

Oracle reduces overlap 18.9% but increases first discovery only 2.6%.
The arithmetic explains why: it has 60.64 m² less overlap **and** 47.83 m²
fewer summed individual observations, leaving 12.81 m² more team discovery.
Lower R only guarantees higher U if S is fixed; equal time or distance does
not fix S. Plotting R against U without considering S is insufficient.

None reaches 400 m² at 242.07 s and 450 m² at 254.38 s; oracle reaches those
thresholds at 244.95 s and 272.40 s. The small oracle advantage at 300 s does
not mean it discovered the first 450 m² faster.

Final recorded path lengths sum to 168.87 m (none) and 167.12 m (oracle).
These are whole-recording totals, not distance clipped at 300 s. Raw map
header clocks advance about 327.7 simulation seconds over 328.6 recording
seconds for none, and 330.15 over 332.83 for oracle (robot1). Simulator
slowdown is small in this pair; it does not explain the missing large gain.

Crucially, R counts a cell at most once per robot beyond its first observer.
It does **not** count repeated travel by the same robot through its own map.
Conversely, scanning a shared corridor while proceeding to distinct work
increases R without necessarily wasting any travel. The current graph is
observation overlap, not a measurement of time lost to revisits. These bags
establish the accounting difference; they do not establish which maneuver
caused it. Check paths and frontier decisions before attributing it to
backtracking, clustering or recovery.

## Diagnostic experiment setup

Use the existing `long_t` branch-exchange world as a targeted next pilot,
with its distributed starts at the ends of three separately surveyed
branches. It makes visiting a teammate's completed branch an avoidable trip
while six unobserved work branches remain. In maze2, the eight cycles give
shared corridors a legitimate transit role, weakening that relationship.

Keep the **same current** `visible_gain` / `local_rank` policy, ownership
bonus, cluster separation, sensor range, robot speed and geometry in both
arms. Do not compare an old strict-rank run to a new local-rank run. Do not
supply a reference map or advance branch assignments to the robots. Static
robot IDs are allowed equally in both arms; they are not exclusive claims.

Use these experiment.conf values, retaining the existing sensor/controller
and frontier YAML settings:

```sh
NUM_ROBOTS=3
SPAWN_PRESET=distributed
EXPLORE_START_STAGGER_S=0
IMPAIRMENT_MODE=sim
LINK_PROFILE=static
BANDWIDTH_KBPS=0
LOSS_PCT=0
DELAY_MS=0
RECORD_METRICS=true
```

Run a balanced pilot with the existing runner:

```sh
python3 tools/run_experiment_sweep.py --world long_t --methods none,oracle --min-runs 3 --target-runs 3 --duration 1200
```

The runner's duration is a **wall-time collection cap after all first goals**,
not the analysis budget. Include the initial surveys in coverage. Evaluate
on a common simulation clock and prespecify 300 s and 600 s checkpoints;
report an endpoint as unavailable if a recording does not reach it. The
current figure/summary utility uses elapsed bag time, so simulation-time
analysis requires map header timestamps (or recording /clock), not relabeling
the current axis. Freeze settings after the pilot and collect at least ten
paired trials, counterbalancing `distributed` and `reversed` starts equally
in both conditions. Keep failed missions and distinguish startup failures.

Primary outcomes:

- New team-observed area at the fixed simulation-time checkpoints.
- First time to fixed discovery targets, declared before the confirmatory
  runs. Use 300 and 450 m² as absolute known-cell pilot targets. For literal
  floor percentages, first build a common collision-derived floor mask; do
  not divide raw known-cell area by nominal floor area.
- New team-observed area per fixed team-distance budget, alongside time.

Mechanism checks, all **before** the discovery target:

- Entry into a teammate's already-surveyed start branch while useful work
  remains, plus distance and elapsed time spent there.
- Travel time without new team cells, including revisits to the robot's own
  territory. Record whether movement, turning, waiting or recovery consumed
  that interval; a short low-gain transit may still be necessary.
- Simultaneous work in distinct branches. A map-only rank preference does not
  guarantee exclusive task ownership. If oracle robots still follow one
  another, address allocation before interpreting a transport comparison.

The pilot passes the mechanism check only if avoiding redundant branch
entries releases time that oracle actually spends discovering other branches.
The common work spine can still bottleneck the team. If this happens, a next
geometry revision should provide three parallel work areas directly off the
transfer hall, with enough work beyond occluding bends to keep all robots
busy. Make completed side trips longer than useful transit, keep corridor
width/sensing identical across arms, and freeze the revised world before
confirmatory trials. Do not tune a map separately for a transport or claim a
strong relationship before it appears across repeated trials.

## Reproduce the accounting

Export the two runs with `tools/export_run_summary.py` in the ROS environment,
using robot1, robot2 and robot3 entries pointing to each run's `bag` directory.
Then run (NumPy only; no ROS required):

```sh
python3 tools/analyze_discovery.py /tmp/discovery_none.npz /tmp/discovery_oracle.npz --budgets 60 120 180 240 300 --targets 200 300 400 450 --window 30
python3 -m unittest tools.tests.test_discovery -v
```

The checked pilot output is in `maze2_discovery_pilot.json`; its /tmp keys name
the source caches, while the actual run IDs are recorded above. The tool
reports windowed new-cell rate, overlap, individual observation totals and
first-passage times. It refuses mismatched resolutions and does not silently
extend a run beyond the last map observation common to all its robots.
