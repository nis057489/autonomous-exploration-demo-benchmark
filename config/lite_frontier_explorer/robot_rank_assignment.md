# Frontier allocation by robot rank

Both simulation configurations now enable `frontier_assignment: robot_rank`
with `selection_strategy: visible_gain`. The multi-robot launch supplies
zero-based `robot_index` and `team_size`; robot1 receives index 0, etc. A
single robot retains the independent utility selector. Direct node launches
must supply membership themselves; the default is a one-robot team. Legacy
selection strategies continue to operate independently.

Each robot independently performs this procedure on its available costmap:

1. Detect frontier clusters and sample canonical viewpoints in each cluster.
2. Rank clusters by their maximum visible unknown area. Break equal-gain ties
   by sorted cell coordinates. Do not include the robot's position, private
   blacklist, or minimum goal distance in this shared ranking.
3. Allocate rank 1 to robot1, rank 2 to robot2, rank 3 to robot3. Additional
   ranks cycle through the team: robot1 owns ranks 1,4,7; robot2 owns 2,5,8.
4. Choose the first locally usable owned cluster. Within that cluster, select
   a reachable viewpoint by gain / (1 + distance weight * path length).

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

Active goals are committed through ordinary gain/rank changes. The robot
chooses another assignment after success, failure, or the existing stuck
handling. This prevents repeated rank swaps from cancelling navigation every
three seconds. It also means this is not a continuous exclusive reservation:
robots replanning at different times may temporarily overlap assignments.
There is no goal claim channel, coordinator, peer pose subscription, or
transport bypass. Slightly different costmaps can yield different rankings,
even under oracle, because local observations and obstacle layers differ.

Use the identical policy and membership in every transport arm. `none` still
has no peer maps; knowing its static team ID does not tell a robot what anyone
has explored. For an allocation ablation set `frontier_assignment: independent`
in the applicable YAML. The earlier 25% utility-preemption rule still applies
to that independent mode; rank allocation suppresses ordinary preemption.

Scores include one-based `rank`, `fallback`, selected viewpoint and active
goal. Startup logs show the effective assignment mode and robot/team ID.

Validation covers distinct assignments with different robot positions, stable
ties under cluster/cell reordering, private blacklists, fewer frontiers than
robots, unreachable assignments and one-robot compatibility. ROS/Gazebo
closed-loop evaluation is still needed; allocation by rank is a simple
symmetry breaker rather than an auction or an exclusive task allocator.
