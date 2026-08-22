# C++ TP-ASD-RRT* planner

The planner keeps the existing RRT* parent selection, rewiring, feasibility,
and terrain-speed/time/radiation-dose edge cost. With
`enable_adaptive_sampling: true` (the default), it adds the mechanisms that
distinguish TP-ASD-RRT* from risk-weighted RRT*:

- an 8 by 8 region distribution weighted by
  `max(0.05, (1 - combined_risk)^low_risk_exponent)` and goal distance;
- a normalized mixture of uniform, direct-goal, and risk-region sampling;
- APF extension direction mixing the sampled direction, goal attraction, and
  the negative central-difference gradient of combined terrain/radiation risk;
- bounded probability adaptation based on search progress, consecutive
  failures, and iterations without improvement in closest goal distance.

Invalid or unavailable risk values omit affected regions. If no region or
gradient is usable, sampling safely falls back to uniform and APF continues
without a risk term. Status strings report `samples_uniform`, `samples_goal`,
`samples_risk`, `guided`, and `adaptations` for mechanism evidence.

Set `enable_adaptive_sampling:=false` in launch, or the corresponding YAML
parameter, for the legacy baseline. That branch does not build regions, call
the risk callback, draw any additional random numbers, or apply APF; its
uniform/direct-goal sample and steer sequence is seed-compatible with the
previous C++ implementation. PID, residual RL, Safety Gate, path, action, and
command topics are unchanged.

The main tunables are documented in `config/tp_asd_rrt_star_online.yaml`.
`goal_sample_rate` and `risk_sample_rate` initialize the mixture; min/max
rates preserve exploration and keep all probabilities legal. Region, gradient,
direction-weight, adaptation interval/stagnation, and adaptation-step
parameters are independently configurable.

## Verification and result labels

The planner-node weighted cost combiner is shared with a pure unit test.
Holding other terms fixed, increasing terrain, radiation, or traversal time
strictly increases total cost under positive configured weights;
`include_time_penalty=false` removes only the traversal-time contribution.

`acceptance_logs/tp_asd_20260822/synthetic_ab_30seeds.json` contains paired
seeds 31--60 on open, risk-wall, and gradient maps. All six groups succeeded
30/30 and TP-ASD produced nonzero risk/guided counters. Relative to the
**legacy risk-weighted RRT* baseline**, TP-ASD mean cost changed by -0.1975366
on open, +0.0973630 on risk-wall, and +0.0189667 on gradient. It used fewer
mean nodes on all three. Because cost was slightly higher on two maps, this is
a mechanism acceptance result, not evidence of uniform performance
superiority. Planner results produced before this adaptive implementation must
be labeled legacy risk-weighted RRT* even if an old node banner said TP-ASD.
