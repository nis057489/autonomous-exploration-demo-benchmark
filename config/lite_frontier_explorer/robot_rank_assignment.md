# Frontier allocation by robot rank

Both simulation configurations enable `frontier_assignment: local_rank` with
`selection_strategy: visible_gain`. The multi-robot launch supplies zero-based
`robot_index` and `team_size`; robot1 receives index 0, etc. A single robot
retains the independent utility selector. Direct node launches must supply
membership themselves; the default is a one-robot team. Legacy selection
strategies continue to operate independently.

Each robot independently performs this procedure on its available costmap:

1. Detect frontier clusters and sample canonical viewpoints in each cluster.
   Cells are grouped at 8-connectivity (`frontier_connectivity`), so a
   boundary running diagonally is one cluster rather than a row of fragments.
2. Rank clusters by their maximum visible unknown area. Break equal-gain ties
   by sorted cell coordinates. Merge a cluster into a better-ranked task when
   `rank_merge_overlap` of the area it can observe is already claimed by that
   task, so one boundary cannot hold several ranks. Do not include the robot's
   position, private blacklist, or minimum goal distance in this shared
   ranking.
3. Allocate rank 1 to robot1, rank 2 to robot2, rank 3 to robot3. Additional
   ranks cycle through the team: robot1 owns ranks 1,4,7; robot2 owns 2,5,8.
4. Score every task by gain / (1 + distance weight * path length), multiplying
   owned ranks by `rank_ownership_bonus`, and take the best. Under
   `robot_rank` instead, take the first locally usable owned task and choose a
   viewpoint only within it.

`local_rank` is the default because allocation must not be paid for in
distance. A robot takes a teammate's rank only when it is worth more than
`rank_ownership_bonus` times its own best option, so the bonus is the entire
split-up-versus-detour tradeoff: 1.0 is independent selection, and a large
enough value reproduces `robot_rank`'s willingness to cross a mapped building.
`robot_rank` ignores the robot's position completely, which is a property to
choose deliberately, not a default.

Identical maps and parameters yield identical ranking regardless of the input
cluster order. A task is not necessarily an architectural branch: the merge
groups clusters that observe the same space, not clusters in the same room, so
two ways into one wing can still be ranked separately.

When there are fewer tasks than robots, IDs wrap around the available ranks.
With two tasks and three robots the assignments are 1,2,1; with one task they
all use it. This preserves progress, including `none` runs where each robot may
initially see only one local frontier. If every owned task is unreachable, too
close, or blacklisted, the robot falls back to another usable rank. The score
log explicitly marks `fallback=True`. Duplicates are possible during either
fallback; this is preferable to a robot waiting forever for work it cannot
observe or reach.

Active goals are committed through ordinary gain/rank changes. The robot
chooses another assignment after success, failure, the existing stuck handling,
or when the destination stops being worth reaching at all (see below). This
prevents repeated rank swaps from cancelling navigation every three seconds. It
also means this is not a continuous exclusive reservation: robots replanning at
different times may temporarily overlap assignments, and because ranks are
ordered by gain, exploring one task renumbers the rest -- a robot deciding
later can be allocated the task a teammate is already driving to. Nothing in a
map-only scheme prevents that.

There is no goal claim channel, coordinator, peer pose subscription, or
transport bypass. Slightly different costmaps can yield different rankings,
even under oracle, because local observations and obstacle layers differ.

Use the identical policy and membership in every transport arm. `none` still
has no peer maps; knowing its static team ID does not tell a robot what anyone
has explored. For an allocation ablation set `frontier_assignment: independent`
in the applicable YAML, or `rank_ownership_bonus: 1.0`. The earlier 25%
utility-preemption rule still applies to that independent mode; rank allocation
suppresses ordinary preemption.

`obsolete_goal_gain_m2` is separate from preemption and applies in every mode:
an in-flight goal whose remaining observable area falls below it on
`obsolete_goal_confirmations` consecutive ticks is cancelled and re-chosen,
without blacklisting, provided another frontier at least
`goal_preempt_distance_m` away exists. This is the loss-of-expected-gain rule
the office protocol lists as follow-up work; it is what stops a robot
completing a long drive to a frontier a teammate has since mapped, and it is
the one place where a peer's map can change a decision already in flight.

Scores include one-based `rank`, `fallback`, cluster count, selected viewpoint
and active goal. Startup logs show the effective assignment mode and
robot/team ID.

## What this replaced, and the evidence

Measured by replaying the selector on `experiment_runs/20260911_211603_oracle_maze2`
(three robots, oracle, maze2) at the five decisions where robots converged:

- Grouping frontier cells at 4-connectivity turned those maps into ~210
  clusters, nearly all 2-cell pieces of a few diagonal boundaries. The nine
  best-ranked "distinct" clusters were all within a metre of (24, 4), so
  ranks 1/2/3 were three slivers of one frontier and all three robots drove
  to it; the same convergence repeated at (16, 0). Every one of the 15
  replayed picks put a robot within sensor range of a teammate's pick.
- Strict `robot_rank` chose by rank alone, so robot3 selected a frontier
  44.7 m away with usable work 3.6 m in front of it.
- With 8-connectivity, task merging and a 2.0 ownership bonus, the same five
  decisions give 2 colliding pairs instead of 15, 95.5 m of total planned path
  instead of 284.1 m, a worst single trip of 12.3 m instead of 44.7 m, and
  ~0.7 s of selector CPU per tick instead of ~2.8 s.
- The two remaining collisions are robots roughly a metre apart with one rich
  frontier nearby and nothing else within 30 m. Separating those requires
  knowing where the teammate is, which no map-only rule can supply.

Validation covers distinct assignments with different robot positions, stable
ties under cluster/cell reordering, private blacklists, fewer frontiers than
robots, unreachable assignments, one-robot compatibility, diagonal boundary
grouping, merging of clusters that observe one area, separation of co-located
robots, and that ownership does not force a long trip. ROS/Gazebo closed-loop
evaluation is still needed; allocation by rank is a simple symmetry breaker
rather than an auction or an exclusive task allocator.
