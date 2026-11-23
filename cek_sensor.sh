#!/usr/bin/env bash
# Print available lidar/camera topics and check that all expected sensors exist.

set -uo pipefail

SENSORS=(
  /lidar_left
  /lidar_right
  /lidar_up
  /lidar_down
  /lidar_front
  /lidar_back
  /camera_front
  /camera_down
)

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: 'ros2' not found. Source your ROS 2 installation first." >&2
  exit 2
fi

if ! keluaran=$(ros2 topic list 2>&1); then
  echo "error: 'ros2 topic list' failed." >&2
  echo "$keluaran" >&2
  exit 2
fi

topik=$(printf '%s\n' "$keluaran" | grep '^/' || true)

echo 'Available sensor topics:'
tersedia=$(printf '%s\n' "$topik" | grep -E 'lidar|camera' | sort || true)
if [[ -n "$tersedia" ]]; then
  printf '%s\n' "$tersedia" | sed 's/^/  /'
else
  echo '  (none)'
fi

hilang=()
for sensor in "${SENSORS[@]}"; do
  if ! printf '%s\n' "$topik" | grep -Fxq -- "$sensor"; then
    hilang+=("$sensor")
  fi
done

echo
echo "Expected: $(( ${#SENSORS[@]} - ${#hilang[@]} ))/${#SENSORS[@]} present"
if (( ${#hilang[@]} > 0 )); then
  echo 'Missing:'
  printf '  %s\n' "${hilang[@]}"
  exit 1
fi

echo 'All expected sensors are present.'
