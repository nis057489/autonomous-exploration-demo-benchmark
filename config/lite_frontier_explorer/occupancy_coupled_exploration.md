# Selecting an occupancy-coupled exploration policy

Status: retain and correct the existing visible-gain algorithm, per the updated
experiment direction. Literature below supplies context, not a claim that this
implementation reproduces a published algorithm. Current corrections and
behavior are documented in [communication_reservations.md](communication_reservations.md).

## Required dependency

The experimental variable is delivery of the underlying occupancy grid, not
merely delivery of robot positions, bids, reservations, or region assignments.
Each robot must plan from its own observations and the peer occupancy updates
it has actually received. Neither the evaluator's complete map nor another
robot's undelivered map may enter its exploration decisions.

The required causal chain is:

    transport -> received occupancy evidence -> local fused map
              -> frontier candidates and expected observations -> exploration

A robot that receives evidence that a hallway is explored should stop treating
that hallway's observed cells as potential new information. A received wall can
also change visibility and reachability. An update may reveal a useful frontier
as well as remove one. Therefore, delivered bytes or known-cell count alone are
not sufficient measures of useful communication.

## Closest bandwidth-specific reference

[Meier, Stachniss and Burgard (2005), Coordinating Multiple Robots During
Exploration Under Communication With Limited Bandwidth](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/meier05ecmr.pdf)
combines local observations with received approximate maps before extracting
frontiers. Algorithm 1 discounts target utility using received peer goals and
selects utility minus travel cost. Its experiments vary map refinement rate.
The network assumption is reliable, unlimited-range, bandwidth-limited links.
Crucially, initial target utility is constant: this is not an explicit
comparison of expected newly observed area between frontier clusters.

Use this as the reference for map-dependent distributed coordination. Replacing
its polygon representation with decoded occupancy grids is an adaptation and
must be labelled accordingly. It does not by itself settle the observed problem
of selecting small residual frontiers over substantial unexplored corridors.

The underlying
[Burgard et al. (2005), Coordinated Multi-Robot Exploration](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/burgard05tro.pdf)
provides the utility discounting and travel-cost foundation. It includes
coordination under limited communication range. Range experiments should not
be presented as evidence about occupancy codec quality or packet loss.

## Selection decision

Prioritize a distributed policy whose frontier set and observation utility
depend on the delivered occupancy grid. Persistent region allocation alone
does not establish the desired dependence: robots can follow separate region
assignments despite receiving little subsequent map information.

Retain the existing visible unknown area divided by travel-cost utility, with
its ratio hysteresis. Correct inconsistencies that allow occupancy evidence to
affect candidates but fail to affect route estimates or active-goal decisions.
Keep the bandwidth-specific reference as context for the map-sharing dependency,
without labelling the existing scoring formula a faithful reproduction.

## Repository integration requirements

- Frontier detection and observation utility must use the same received-aware
  occupancy evidence. Navigation inflation and collision-clearing artifacts
  must not become evidence of explored space.
- Locally observed cells must not become unknown again merely because a peer
  update is coarse or missing. Define conflicting-evidence handling explicitly.
- Map updates must participate in the published replanning rule, including
  reevaluation of an active goal. Correct candidate generation is insufficient
  if the robot remains committed to a goal whose value has disappeared.
- Preserve the distinction between travelling through known space to reach
  unexplored space and selecting already observed space as an exploration goal.
- Keep the policy and parameters identical across codecs. No scoring term may
  depend on the transport's name or an oracle performance target.
- Log the map revision, candidate rejection reasons, utility components,
  travel costs, and active-goal decision so a screenshot can be traced to a
  reproducible decision.

## Evidence required before adoption

First replay a fixed local map and fixed peer poses/goals while varying only
received occupancy updates. Include a peer-explored hallway, a newly received
wall, an opened northern frontier, and a shrinking active frontier. Decisions
must be explained by the selected algorithm's equations, not expected labels.

Then run closed-loop paired trials with identical starts and environments:

| Map delivery | Coordination delivery | Purpose |
| --- | --- | --- |
| Unimpaired | Unimpaired | Establish competent exploration first |
| Impaired | Unimpaired | Isolate dependence on occupancy delivery |
| Unimpaired | Impaired | Measure dependence on control messages |
| Impaired | Impaired | Evaluate the complete deployed system |
| None | None | Establish independent exploration baseline |

Count coordination traffic in deployment bandwidth. Use the isolated cases as
ablations, not as the headline codec comparison.

Measure physical observed union, distance and time to fixed coverage thresholds,
duplicate observations, per-robot map knowledge, occupancy errors, and age of
useful received updates separately. Use evaluator-only geometry consistently
across methods. Report repeated paired runs and failures to reach thresholds.

Acceptance requires measured sensitivity to useful occupancy delivery and
competent unimpaired behavior. It does not require every individual run to rank
in bandwidth order: exploration decisions are discrete, and communication
benefits can saturate. No policy should be deliberately weakened to manufacture
a desired ordering.
