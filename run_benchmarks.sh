#!/usr/bin/env bash
# Build the core and run every benchmark, writing CSV/Markdown into results/.
# Pass --quick for a short smoke run.
set -euo pipefail
./build.sh
mkdir -p results
docker run --rm --user "$(id -u):$(id -g)" -v "$PWD":/work -w /work devtools:latest bash -c "
  set -e
  for b in bench_control_loop bench_localization bench_collision bench_planners; do
    echo \"== \$b\"
    ./build/\$b --out results $*
  done"
