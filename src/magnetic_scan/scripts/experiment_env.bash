#!/usr/bin/env bash
# Source from this workspace to load only the robot, camera and TagSLAM overlays.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo 'Use: source src/magnetic_scan/scripts/experiment_env.bash' >&2
  exit 1
fi

_magnetic_scan_environment() {
  local magnetic_workspace_root magnetic_dependency_root magnetic_setup
  magnetic_workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)" || return
  magnetic_dependency_root="$(dirname "$magnetic_workspace_root")"
  for magnetic_setup in \
    "$magnetic_workspace_root/devel/setup.bash" \
    "${MAGNETIC_SCAN_ZED_WS:-$magnetic_dependency_root/zed_ws}/devel/local_setup.bash" \
    "${MAGNETIC_SCAN_TAGSLAM_WS:-$magnetic_dependency_root/tagslam_root}/devel/local_setup.bash"; do
    if [[ ! -f "$magnetic_setup" ]]; then
      echo "Missing experiment environment: $magnetic_setup" >&2
      return 1
    fi
  done
  source "$magnetic_workspace_root/devel/setup.bash" || return
  # local_setup avoids pulling in unrelated cached build-time underlays.
  source "${MAGNETIC_SCAN_ZED_WS:-$magnetic_dependency_root/zed_ws}/devel/local_setup.bash" || return
  source "${MAGNETIC_SCAN_TAGSLAM_WS:-$magnetic_dependency_root/tagslam_root}/devel/local_setup.bash" || return
  # Run ROS environment hooks again to refresh ROS_PACKAGE_PATH for all prefixes.
  source /opt/ros/noetic/setup.bash --extend
}
_magnetic_scan_environment
