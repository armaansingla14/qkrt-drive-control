### Periodic loop (gate + wheel PIDs + simulator + slip-aware EKF)

| Rate | Cycles | Compute p50 [us] | Compute p99 [us] | Compute max [us] | Wake-up p50 [us] | Wake-up p99 [us] | Wake-up max [us] | Overruns |
|---|---|---|---|---|---|---|---|---|
| 100 Hz | 1000 | 3.8 | 17.1 | 38.0 | 112.0 | 450.9 | 592.9 | 0 |
| 500 Hz | 5000 | 2.5 | 10.0 | 47.0 | 94.6 | 153.7 | 740.7 | 0 |
| 1000 Hz | 10000 | 2.3 | 48.7 | 109.6 | 93.5 | 162.6 | 650.3 | 0 |

### Component cost (tight loop, 200000 iterations)

| Component | ns/call |
|---|---|
| Wheel PID update | 13 |
| EKF predict + gyro + odometry (+GPS every 500) | 450 |
| Simulator step | 133 |
| Operator JSON decode | 687 |

### Anti-windup on a saturating wheel-speed step (0 -> 18 rad/s)

| Strategy | Overshoot [%] | 2% settling [s] | IAE [rad] |
|---|---|---|---|
| none | 10.83 | 0.630 | 1.680 |
| clamping | -0.00 | 0.268 | 1.209 |
| back_calculation | 0.19 | 0.173 | 1.066 |
