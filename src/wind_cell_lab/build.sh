#!/usr/bin/env bash
set -euo pipefail
lab_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_major="${1:-}"
if [[ ! "$sim_major" =~ ^(7|8|9)$ ]]; then
  echo 'Usage: bash build.sh GAZEBO_MAJOR'
  echo 'First run: gz sim --versions'
  echo 'Choose the version used by your running simulation: 7=Garden, 8=Harmonic, 9=Ionic.'
  exit 2
fi
cmake -S "$lab_dir/plugin" -B "$lab_dir/build" -DGZ_SIM_MAJOR="$sim_major"
cmake --build "$lab_dir/build" -j 2
echo 'Built the wind plugin. Next run python3 prepare.py.'
