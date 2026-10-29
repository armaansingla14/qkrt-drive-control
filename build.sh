#!/usr/bin/env bash
# Configure, build and test the ROS-independent core inside the devtools container.
set -euo pipefail
JOBS="${JOBS:-2}"
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/work -w /work devtools:latest bash -c "
  cmake -S core -B build -G Ninja -DCMAKE_BUILD_TYPE=Release &&
  cmake --build build -j${JOBS} &&
  ctest --test-dir build --output-on-failure ${*:-}"
