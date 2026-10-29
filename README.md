# qkrt-drive-control

Drive control and real-time robotics software for the Queen's Knights Robotics Team (QKRT) rover:
wheel-velocity control, differential / skid-steer kinematics, slip-aware EKF localization, grid and
sampling-based motion planning, broad-phase collision checking, a JSON/UDP teleoperation link with an
operator console, and the CAN / CAN FD message set for the motor controllers.

The algorithms live in a ROS-independent C++17 library (`qkrt_core`, Eigen + CMake) that is built and
tested on its own. A thin ROS 2 package (`qkrt_drive_ros`) wraps the library in nodes and launch files.
Everything is exercised against a 2D differential-drive simulator with motor lag, wheel-radius error,
wheel slip and noisy sensors, which is also what the benchmarks below run on.

## Features

| Area | What is implemented |
|---|---|
| Motor control | Discrete PID with derivative-on-measurement, optional derivative low-pass, feed-forward, and three anti-windup modes (none / conditional clamping / back-calculation). `DriveController` maps a body twist to wheel setpoints with curvature-preserving desaturation and runs one PID per wheel. |
| Kinematics | Differential drive and skid-steer (ICR expansion factor `chi` on the effective track width), exact arc-integrated wheel odometry. |
| Localization | 6-state EKF `[x, y, theta, v, omega, gyro_bias]` fusing wheel odometry, gyro, accelerometer (propagation) and GPS-like position fixes. Speed-dependent odometry yaw-rate noise plus zero-rate updates at standstill keep the gyro-bias state from absorbing odometry errors. |
| Slip handling | `WheelSlipDetector` compares encoder yaw rate vs. bias-corrected gyro, and encoder speed vs. an IMU-integrated speed that is only re-anchored while traction is trusted; an odometry NIS gate catches the rest. While slip is flagged the odometry covariance is inflated. |
| Planning | 8-connected A* (octile heuristic, no corner cutting), RRT and RRT* (choose-parent + rewire with cost propagation), line-of-sight shortcutting, obstacle inflation, random map generation, pure-pursuit path tracking. |
| Collision | Circles and oriented boxes with exact narrow phase (SAT). Brute force baseline, static BVH (binned SAH, flat array), and a dynamic AABB tree (fattened boxes, perimeter-cost insertion, AVL-style rotations) for moving obstacles. |
| Teleoperation | Strict flat-JSON codec, operator message set (`drive`, `heartbeat`, `estop`, `clear_estop`), `TeleopGate` (stale/out-of-order rejection with sequence wrap, watchdog timeout, latched e-stop, magnitude and rate limits), non-blocking UDP socket, `teleop_server` simulator app, and a Python terminal console (`tools/operator_console.py`). |
| CAN | 11-bit ID plan, classic 8-byte drive command / motor status frames, a 32-byte CAN FD telemetry frame with CRC-16/CCITT, DLC mapping, saturating fixed-point scaling, alive-counter monitoring, and a matching `can/qkrt_motor.dbc`. |
| ROS 2 | `sim_node`, `drive_controller_node` (teleop-over-autonomy twist mux), `localization_node`, `planner_node` (plan + pure pursuit), `teleop_bridge_node`, `rover.launch.py`, `teleop_only.launch.py`, `config/rover.yaml`. |

## Layout

```
core/                     ROS-independent library (colcon "cmake" package qkrt_core)
  include/qkrt/           public headers
  src/                    implementation
  tests/                  unit tests (self-contained harness, run by ctest)
  bench/                  benchmarks writing to results/
  apps/teleop_server.cpp  simulated rover driven over the UDP operator protocol
ros2/qkrt_drive_ros/      ament_cmake package: nodes, launch files, parameters
tools/operator_console.py terminal operator UI (curses) / scripted client, plus pytest tests
can/qkrt_motor.dbc        CAN database for the motor-controller frames
results/                  benchmark outputs (CSV + Markdown)
build.sh                  configure, build and test the core in the devtools container
run_benchmarks.sh         build and run all benchmarks
```

## Building and testing

The core needs CMake >= 3.16, a C++17 compiler and Eigen 3. In this repository it is built inside a
Docker image that provides g++ 13, CMake, Ninja and Eigen:

```bash
./build.sh                          # cmake + ninja + ctest (5 test executables incl. a UDP loopback test)
python -m pytest tools -q           # operator console protocol tests
```

Without Docker:

```bash
cmake -S core -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ctest --test-dir build
```

Teleoperation demo (robot side and operator side in two terminals):

```bash
./build/teleop_server --port 47800
python3 tools/operator_console.py --port 47800            # interactive: w/a/s/d, space, e, c, q
python3 tools/operator_console.py --script "drive 0.8 0 2; drive 0.5 0.6 2; estop 0.5; clear 0.2; idle 1"
```

The scripted run above was used as a smoke test: the server accepted 104 operator packets, went to
`estop` on command, ignored drive packets until `clear_estop`, and reported `timeout` once the
operator went silent.

ROS 2 workspace (requires a ROS 2 installation; see Limitations):

```bash
mkdir -p ~/qkrt_ws/src && ln -s $PWD/core $PWD/ros2/qkrt_drive_ros ~/qkrt_ws/src/
cd ~/qkrt_ws && colcon build && source install/setup.bash
ros2 launch qkrt_drive_ros rover.launch.py
```

## Results

All numbers below come from `./run_benchmarks.sh` (full mode), executed in the Docker container on a
WSL2 Linux host. Everything is simulated; the raw outputs are in `results/`.

### Control loop timing (`bench_control_loop`)

One cycle = teleop gate + both wheel PIDs + simulator step + slip-aware EKF (predict, gyro, odometry,
GPS when available). Loop paced with `std::this_thread::sleep_until` on a stock (non-real-time)
kernel, no `SCHED_FIFO`, no CPU isolation. "Wake-up" is how late the thread woke after its deadline.

| Rate | Cycles | Compute p50 [us] | Compute p99 [us] | Compute max [us] | Wake-up p50 [us] | Wake-up p99 [us] | Wake-up max [us] | Overruns |
|---|---|---|---|---|---|---|---|---|
| 100 Hz | 1000 | 3.8 | 17.1 | 38.0 | 112.0 | 450.9 | 592.9 | 0 |
| 500 Hz | 5000 | 2.5 | 10.0 | 47.0 | 94.6 | 153.7 | 740.7 | 0 |
| 1000 Hz | 10000 | 2.3 | 48.7 | 109.6 | 93.5 | 162.6 | 650.3 | 0 |

The control computation itself is a few microseconds; scheduling latency of the OS dominates, which is
why the onboard computer would need `SCHED_FIFO`/`PREEMPT_RT` for tight 1 kHz deadlines.

Anti-windup on a saturating wheel-speed step (0 to 18 rad/s, motor no-load speed 20 rad/s):

| Strategy | Overshoot [%] | 2 % settling [s] | IAE [rad] |
|---|---|---|---|
| none | 10.83 | 0.630 | 1.680 |
| clamping | 0.00 | 0.268 | 1.209 |
| back-calculation | 0.19 | 0.173 | 1.066 |

### Localization with and without slip handling (`bench_localization`)

30 seeds x 120 s per scenario, closed-loop driving with the wheel PIDs, per-run random wheel-radius
error (0.5 % std), gyro bias, accelerometer bias, and GPS-like fixes at 1 Hz with 1.0 m noise. The same
sensor stream is fed to all three estimators.

| Scenario | Time slipping | Estimator | Position RMSE [m] (mean +/- sd) | Max error [m] | Heading RMSE [deg] |
|---|---|---|---|---|---|
| nominal | 0.0 % | dead reckoning | 12.152 +/- 13.603 | 26.07 | 46.00 |
| nominal | 0.0 % | EKF, no slip handling | 0.274 +/- 0.084 | 0.71 | 1.81 |
| nominal | 0.0 % | EKF, slip-aware | 0.274 +/- 0.084 | 0.71 | 1.81 |
| slip zones | 28.4 % | dead reckoning | 27.573 +/- 21.162 | 49.21 | 105.04 |
| slip zones | 28.4 % | EKF, no slip handling | 2.452 +/- 1.996 | 5.00 | 11.97 |
| slip zones | 28.4 % | EKF, slip-aware | 0.596 +/- 0.264 | 1.61 | 2.01 |
| slip bursts | 13.7 % | dead reckoning | 21.699 +/- 12.749 | 40.95 | 88.68 |
| slip bursts | 13.7 % | EKF, no slip handling | 0.797 +/- 0.335 | 1.60 | 4.23 |
| slip bursts | 13.7 % | EKF, slip-aware | 0.294 +/- 0.073 | 0.79 | 2.12 |

Slip detector against simulator ground truth (per 10 ms step): precision 0.725 / recall 0.953 in the
slip-zone scenario, precision 0.638 / recall 0.983 with random slip bursts, and no false flags in the
nominal scenario (so both EKF variants are identical there). Precision is limited mainly by the
0.3 s hold time after slip ends.

### Collision queries (`bench_collision`)

Random circles and oriented boxes (about one obstacle per 25 m^2), 20,000 oriented-box rover
footprint queries per size; every method returns exactly the same hit sets as brute force.

| Obstacles | Method | Build [ms] | All-hits query [ns] | Any-hit query [ns] | Speed-up vs brute force |
|---|---|---|---|---|---|
| 1,000 | brute force | 0.10 | 2,368 | 2,139 | 1.0x |
| 1,000 | BVH | 0.71 | 244 | 230 | 9.7x |
| 1,000 | dynamic AABB tree | 0.53 | 405 | 370 | 5.9x |
| 10,000 | brute force | 1.07 | 43,143 | 39,787 | 1.0x |
| 10,000 | BVH | 8.23 | 430 | 393 | 100.4x |
| 10,000 | dynamic AABB tree | 7.79 | 1,059 | 931 | 40.7x |
| 100,000 | brute force | 10.26 | 475,156 | 423,217 | 1.0x |
| 100,000 | BVH | 93.84 | 944 | 629 | 503.2x |
| 100,000 | dynamic AABB tree | 109.12 | 2,592 | 2,411 | 183.3x |

At 100 obstacles the trees are only 1.3-1.5x faster. With moving obstacles (10,000 obstacles, 1,000
moving per frame, 1,000 queries per frame) the incremental AABB tree takes 1.36 ms/frame, rebuilding
the BVH every frame 6.35 ms/frame, and brute force 45.5 ms/frame.

### Planners (`bench_planners`)

100 random 50 x 50 m maps (0.5 m cells) per density, obstacles inflated by 0.3 m, start and goal at
least 20 m apart. A* (complete on the grid) decides which queries are solvable. RRT and RRT* get 3,000
iterations, 1 m steps and 5 % goal bias.

| Density | Solvable | Planner | Success (of solvable) | Mean time [ms] | Length / A* | Length / A* after shortcut |
|---|---|---|---|---|---|---|
| 0.15 | 99 | A* | 100.0 % | 0.09 | 1.000 | 0.954 |
| 0.15 | 99 | RRT | 99.0 % | 0.28 | 1.351 | 1.061 |
| 0.15 | 99 | RRT* | 99.0 % | 12.32 | 0.982 | 0.970 |
| 0.25 | 100 | A* | 100.0 % | 0.14 | 1.000 | 0.957 |
| 0.25 | 100 | RRT | 96.0 % | 0.49 | 1.361 | 1.075 |
| 0.25 | 100 | RRT* | 96.0 % | 10.62 | 1.010 | 0.987 |
| 0.35 | 98 | A* | 100.0 % | 0.15 | 1.000 | 0.954 |
| 0.35 | 98 | RRT | 90.8 % | 0.74 | 1.438 | 1.153 |
| 0.35 | 98 | RRT* | 90.8 % | 6.78 | 1.066 | 1.045 |

RRT and RRT* share the sampling sequence for a given seed and RRT* only changes parent edges, so their
success rates are identical by construction; the difference is path quality versus planning time.
Ratios below 1.0 mean the continuous-space path is shorter than the 8-connected grid path.

## Design notes

- **Gyro bias vs odometry.** A first version fused odometry yaw rate with constant noise; with a 0.5 %
  wheel-radius mismatch the bias state slowly absorbed the odometry error and heading drifted. Making
  odometry yaw-rate noise grow with speed and adding zero-rate updates at standstill fixed that.
- **Back-calculation gain** defaults to `1/Tt` with `Tt = sqrt(Ti*Td)` (or `Ti` for PI), the usual
  Astrom-Hagglund choice.
- **CAN FD telemetry** carries its own CRC-16 and alive counter as an end-to-end check on top of the
  bus CRC, so corrupted or stale frames from a misbehaving gateway are rejected in software.

## Limitations

- **ROS 2 was not available in this environment.** `ros2/qkrt_drive_ros` (nodes, launch files,
  parameters, `package.xml`) was written against the rclcpp API but has not been compiled or launched
  here. The library it wraps is built and tested, and its CMake package export was checked with a
  small `find_package(qkrt_core)` consumer project.
- **No MATLAB/Simulink/Stateflow models** are included; the controller tuning was done in the C++
  simulator instead.
- All results are from the simulator. The motor model is first-order, slip is a longitudinal slip
  ratio per wheel (no lateral dynamics), and GPS is modelled as independent Gaussian noise on a local
  frame, so absolute numbers will not transfer directly to the real rover.
- Timing numbers were measured in a Docker container on WSL2 on a general-purpose kernel and only
  reflect that machine.
- The CAN layer encodes and decodes frames; there is no SocketCAN transport in the core library.

## License

MIT, see `LICENSE`.
