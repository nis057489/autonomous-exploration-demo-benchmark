# Frontier allocation by robot rank

Rank allocation is retained as an ablation. The simulation configurations now
use independent utility with DDIL-relayed reservations; see
[communication_reservations.md](communication_reservations.md).

To reproduce rank allocation, enable `frontier_assignment: robot_rank`
with `selection_strategy: visible_gain`. The multi-robot launch supplies
zero-based `robot_index` and `team_size`; robot1 receives index 0, etc. A
single robot retains the independent utility selector. Direct node launches
must supply membership themselves; the default is a one-robot team. Legacy
selection strategies continue to operate independently.

Each robot uses its delivered `/<robot>/nav_map` for frontier detection and
information gain, aligned onto the Nav2 costmap lattice. Occupancy values at or
above `information_occ_threshold` (default 50) block gain rays and cannot be
candidate viewpoints. Nav2 costs separately determine reachability; inflation
costs are not treated as occupancy probabilities. A configured information map
must be available and in the same frame before planning. Standalone launches
without `information_map_topic` retain costmap-only scoring.

Each robot independently performs this procedure on that available information:

1. Detect frontier clusters and sample canonical viewpoints in each cluster.
2. Rank clusters by their maximum visible unknown area. Break equal-gain ties
   by sorted cell coordinates. Do not include the robot's position, private
   blacklist, or minimum goal distance in this shared ranking.
3. Allocate rank 1 to robot1, rank 2 to robot2, rank 3 to robot3. Additional
   ranks cycle through the team: robot1 owns ranks 1,4,7; robot2 owns 2,5,8.
4. Score reachable viewpoints across **all** locally usable owned clusters by
   gain / (1 + distance weight * path length), then choose the best. A low
   rank is not a reason to take a long detour over a better owned destination.
   Only when all owned work is unusable, compare all usable fallback clusters.

This allocates distinct clusters, not several points on the same frontier.
Identical maps and parameters yield identical ranking regardless of the input
cluster order. When at least N usable clusters exist and all robots can use
their own assignments, N robots select different clusters. A cluster is not
necessarily an architectural branch: fragmented boundaries can still produce
nearby assignments in the same wing.

When there are fewer clusters than robots, IDs wrap around the available
ranks. With two clusters and three robots the assignments are 1,2,1; with one
cluster they all use it. This preserves progress, including `none` runs where
each robot may initially see only one local frontier. If every owned cluster
is unreachable, too close, or blacklisted, the robot falls back to another
usable rank. The score log explicitly marks `fallback=True`. Duplicates are
possible during either fallback; this is preferable to a robot waiting forever
for work it cannot observe or reach.

Active goals are rescored on each replan. A different assignment replaces an
active goal only if its utility exceeds the configured improvement ratio
(default 1.25), the goals are sufficiently separated, and the consecutive
preemption budget permits it. Rank changes alone do not trigger a switch.
This applies to rank allocation as well as independent selection: committing
to a long redundant detour while a much better assignment exists defeats the
travel-cost objective. A valid viewpoint with zero remaining observable unknown
area is cancelled even if the preemption budget is exhausted. Missing or
unreachable scores are not evidence that exploration at a viewpoint is complete.

This is not a continuous exclusive reservation: robots replanning at different
times may overlap assignments.
There is no goal claim channel, coordinator, peer pose subscription, or
transport bypass. Slightly different costmaps can yield different rankings,
even under oracle, because local observations and obstacle layers differ.

Use the identical policy and membership in every transport arm. `none` still
has no peer maps; knowing its static team ID does not tell a robot what anyone
has explored. For an allocation ablation set `frontier_assignment: independent`
in the applicable YAML. The same 25% utility-preemption rule applies to both assignment modes.

Scores include one-based `rank`, `fallback`, selected viewpoint and active
goal. Startup logs show the effective assignment mode and robot/team ID.

Validation covers distinct assignments with different robot positions, stable
ties under cluster/cell reordering, private blacklists, fewer frontiers than
robots, unreachable assignments and one-robot compatibility. ROS/Gazebo
closed-loop evaluation is still needed; allocation by rank is a simple
symmetry breaker rather than an auction or an exclusive task allocator.


Communication-coupled evaluation
-------------------------------

The causal input is the map actually delivered to the robot, never a transport
name or configured bandwidth. Fresh peer observations remove completed work
from candidate gain and now terminate active trips to completed viewpoints.
Delayed observations leave that work apparently useful until delivery. The same
logic runs under no sharing, all codecs, and unimpaired sharing.

This is a direct information-to-action mechanism, not a guarantee of monotonic
performance across independent stochastic runs. Rank allocation still permits
conflicting assignments with asynchronous/different maps. Evaluate paired runs
with identical world, starts, policy and duration, repeated across seeds. Report
unique ground-truth observed area per team travel metre and time to a fixed
coverage target; use map overlap as a secondary diagnostic. For a controlled
causal check, deliver the same peer observation at different times while a robot
approaches that region and measure remaining redundant travel. Do not infer
exploration efficiency from the largest individual map or absolute overlap alone.


Recorded false-gain diagnosis (20260912_085742_vxch_long_t)
---------------------------------------------------------

Robot2 was sent to (5.575, 16.325) after a stuck-goal cancellation. The score log
reported 11.92 m² gain and 33.85 m path length. The latest recorded nav_map before
that command gives 12.18 m² with the old visibility calculation. The difference
is expected: the old node scored a different topic, the unrecorded Nav2 costmap.
All of those nav-map credited cells fall outside the corridor in simulator
geometry. The goal cell has occupancy 56: treating only 100 as a wall let rays
pass into unknown exterior space. With the occupied threshold 50 its gain is
zero and it is excluded as a frontier viewpoint. Geometry was used for this
post-run diagnosis only; the policy uses delivered occupancy evidence.

Ranking by travel utility and allowing beneficial goal replacement address
separate detour mechanisms, but neither repairs false information gain. The
occupancy/costmap separation is necessary for this observed backtrack.
