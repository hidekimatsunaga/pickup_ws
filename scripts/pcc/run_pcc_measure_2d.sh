#!/usr/bin/env bash
# Simple runner for PCC 2D measurement recorder
# Usage: ./scripts/run_pcc_measure_2d.sh [CSV_PATH] [HZ]
# Defaults:
#  CSV_PATH: src/pcc_test/pcc_test/lut_csv/pcc_measure_1024.csv
#  HZ      : 5.0

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${SCRIPT_DIR}/.."
DEFAULT_CSV="${WS_ROOT}/src/pcc_test/pcc_test/lut_csv/pcc_measure_1024_v2.csv"
CSV_PATH="${1:-$DEFAULT_CSV}"
HZ="${2:-5.0}"

echo "Running pcc_measure recorder (2D)"
echo "  csv_path: $CSV_PATH"
echo "  record_hz: $HZ"

# Source workspace if available
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  # temporarily disable nounset to avoid errors from setup scripts that reference
  # variables not defined in this shell (COLCON_TRACE etc.)
  set +u
  source "${WS_ROOT}/install/setup.bash"
  set -u
else
  echo "Warning: workspace install/setup.bash not found. Ensure ROS2 is sourced and package is built." >&2
fi

# If DRY_RUN=1 is set, skip actually running ros2 for quick checks
if [ "${DRY_RUN:-0}" = "1" ]; then
  echo "DRY_RUN=1: skipping ros2 run (would run lut_measure_recorder with csv_path=$CSV_PATH, record_hz=$HZ)"
  exit 0
fi

# Run the node with parameters. This will block in the foreground.
ros2 run pcc_test lut_measure_recorder \
  --ros-args -p csv_path:="$CSV_PATH" -p use_2d:=true -p drop_axis:=z -p record_hz:="$HZ" -p mode:=timer

# End
