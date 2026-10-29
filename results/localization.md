Simulated, 30 seeds x 120 s per scenario, GPS-like fixes at 1 Hz with 1.0 m noise per axis.

| Scenario | Time slipping [%] | Estimator | Position RMSE [m] (mean +/- sd) | Max error [m] | Heading RMSE [deg] |
|---|---|---|---|---|---|
| nominal | 0.0 | dead_reckoning | 12.152 +/- 13.603 | 26.07 | 46.00 |
| nominal | 0.0 | ekf_naive | 0.274 +/- 0.084 | 0.71 | 1.81 |
| nominal | 0.0 | ekf_slip_aware | 0.274 +/- 0.084 | 0.71 | 1.81 |
| slip_zones | 28.4 | dead_reckoning | 27.573 +/- 21.162 | 49.21 | 105.04 |
| slip_zones | 28.4 | ekf_naive | 2.452 +/- 1.996 | 5.00 | 11.97 |
| slip_zones | 28.4 | ekf_slip_aware | 0.596 +/- 0.264 | 1.61 | 2.01 |
| slip_bursts | 13.7 | dead_reckoning | 21.699 +/- 12.749 | 40.95 | 88.68 |
| slip_bursts | 13.7 | ekf_naive | 0.797 +/- 0.335 | 1.60 | 4.23 |
| slip_bursts | 13.7 | ekf_slip_aware | 0.294 +/- 0.073 | 0.79 | 2.12 |

Slip detector (per 10 ms step vs simulator ground truth):

| Scenario | Precision | Recall | False-flag rate |
|---|---|---|---|
| nominal | - | - | 0.0000 |
| slip_zones | 0.725 | 0.953 | - |
| slip_bursts | 0.638 | 0.983 | - |
