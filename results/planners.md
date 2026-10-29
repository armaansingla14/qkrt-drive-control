100 random 100x100 maps (0.5 m cells, 50x50 m) per density, obstacles inflated by 0.3 m, start/goal >= 20 m apart.
Length ratio is relative to the 8-connected A* path on the same query (A* row: shortcut A* vs raw A*). RRT and RRT* get 3000 iterations, 1 m step, 5 % goal bias; RRT stops at the first solution, RRT* spends the full budget.

| Obstacle density | Solvable queries | Planner | Success (of solvable) | Mean time [ms] | p95 time [ms] | Length / A* | Length / A* after shortcut |
|---|---|---|---|---|---|---|---|
| 0.15 | 99 / 100 | astar | 100.0 % (99) | 0.09 | 0.27 | 1.000 | 0.954 |
| 0.15 | 99 / 100 | rrt | 99.0 % (98) | 0.28 | 1.08 | 1.351 | 1.061 |
| 0.15 | 99 / 100 | rrt_star | 99.0 % (98) | 12.32 | 14.53 | 0.982 | 0.970 |
| 0.25 | 100 / 100 | astar | 100.0 % (100) | 0.14 | 0.46 | 1.000 | 0.957 |
| 0.25 | 100 / 100 | rrt | 96.0 % (96) | 0.49 | 2.69 | 1.361 | 1.075 |
| 0.25 | 100 / 100 | rrt_star | 96.0 % (96) | 10.62 | 14.52 | 1.010 | 0.987 |
| 0.35 | 98 / 100 | astar | 100.0 % (98) | 0.15 | 0.46 | 1.000 | 0.954 |
| 0.35 | 98 / 100 | rrt | 90.8 % (89) | 0.74 | 2.33 | 1.438 | 1.153 |
| 0.35 | 98 / 100 | rrt_star | 90.8 % (89) | 6.78 | 8.95 | 1.066 | 1.045 |
