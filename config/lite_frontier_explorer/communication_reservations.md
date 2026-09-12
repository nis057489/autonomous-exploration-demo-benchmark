# Communication-dependent goal reservations

The 20260912_091319_oracle_long_t logs show robot3 sent to (16.78, -11.18)
and robot2 sent to (16.83, -11.68) seconds later. Different frontier ranks
were not different tasks. Robot3 subsequently retained a 32 m return trip
with only 0.295 m² remaining gain because its preemption budget was exhausted.

Simulation configs now use independent travel utility plus goal reservations.
The multi-robot launch publishes each active goal at 1 Hz as a PoseArray in
`map`; an empty array releases it. Each receiver subscribes only to
`/<robot>/incoming/<peer>/reservation`, never the peer's direct publisher.
Baseline, oracle, VXCH and Zstd relay these messages through the existing
per-peer DDIL proxy and shared token budget. The none arm creates no relays.
The source clock stamp defines an 8 s lease, so delayed delivery cannot refresh
an old reservation. Older updates cannot resurrect a released goal.

New goals exclude viewpoints within 6 m of a live reserved destination,
measured through known traversable space rather than straight through walls.
Simultaneous conflicting active claims resolve by robot index: the higher
index yields, even if its ordinary utility-preemption budget is exhausted.
There is no unimpeded coordinator, peer pose feed, or transport-label score.
Once other robots receive a claim, they can select another destination or wait;
when a lease expires, work becomes eligible again.

Candidates must offer more than 0.5 m² observable unknown area. Active goals
at or below that threshold are cancelled.
This deliberately leaves tiny residual map gaps rather than spending long
trips on them. `min_gain_m2`, `reservation_radius_m`, and `reservation_ttl_s`
are configurable and must be held constant across transport arms.

This is local goal-region exclusion, not a guarantee of exclusive ownership
of an entire architectural hallway. Before claims arrive, or after a lease
expires during an outage, duplicate decisions remain possible. Different
maps can also produce different distance estimates. Those limitations must be
measured in closed-loop runs; source and decision tests alone do not establish
end-to-end efficiency improvements.

Simulation recordings now include published and delivered reservations.
Comparison bandwidth totals include their serialized bytes, avoiding a free
coordination channel hidden from the transport comparison. Existing runs have
no reservations and cannot validate this policy retrospectively. Hardware
launches need equivalent impaired relay wiring before claiming the same behavior.


Active-goal validation
----------------------

In 20260912_092924_oracle_long_t, robot2 continued to (-6.18, 1.98) while
northern alternatives were selected and its active score repeatedly became
empty. Previously the utility-comparison guard then prevented any switch.
The scorer now distinguishes outside-map, blocked/unknown, and unreachable
active viewpoints from valid scored ones. Two distinct consecutive costmap
updates confirming invalidity permit cancellation if a reachable, unreserved
alternative exists, regardless of the ordinary switching budget. Missing input
alone does not trigger cancellation, and this is not labelled task completion
or blacklisted as a navigation failure. Logs state the invalidity reason.

Occupancy-dependent paths and replanning
----------------------------------------

The existing visible-gain policy uses the fused received occupancy grid for
frontiers, wall-occluded unknown area, and now the traversable-space mask used
for travel costs and reservation distances. Nav2 safety costs still constrain
movement; Nav2 clearing or update lag cannot turn an occupied or unknown
occupancy cell into an exploration route. Neither input map is modified.

Active and alternative goals are compared on the same current evidence using
the existing gain / (1 + distance_weight * path_distance) utility. The 1.25
utility ratio and 1 m goal-separation hysteresis remain. The lifetime
`max_consecutive_preemptions` cap now applies only to legacy selectors: it
must not force visible-gain exploration to ignore subsequent map evidence.
This also permits leaving a residual frontier above the 0.5 m² completion
threshold when another frontier has materially better utility.

Either a fresh occupancy message or a fresh Nav2 map can confirm active-goal
invalidity; repeated ticks on the same input pair cannot. These are corrections
to the existing algorithm, not a reproduction of a published coordination
algorithm. Closed-loop runs are still needed to measure efficiency and churn.
