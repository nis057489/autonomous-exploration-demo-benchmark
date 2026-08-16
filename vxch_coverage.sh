#!/usr/bin/env bash
set -euo pipefail

# Measures gtest coverage for the vxch codec (voxelcodec_ros) via gcov/gcovr.
#
# Builds voxelcodec_ros with --coverage instrumentation into a dedicated
# build-coverage/install-coverage tree (kept separate from the normal
# build/ and install/ so this doesn't disturb the regular Release build),
# runs its gtest binaries, then summarizes line/branch coverage with gcovr.
#
# Must run inside the jazzy_env distrobox (ROS2/colcon build environment).
#
# Usage: ./vxch_coverage.sh [--html]
#   --html  also emit an HTML report at vxch_coverage_html/index.html

HTML=0
if [[ "${1:-}" == "--html" ]]; then
  HTML=1
fi

if [[ -z "${CONTAINER_ID:-}" ]]; then
  echo "==> Re-running inside distrobox 'jazzy_env'"
  exec distrobox enter jazzy_env -- bash -lc "$(printf '%q ' "$0" "$@")"
fi

# distrobox enter with a non-login bash -c doesn't source ~/.bashrc, so make
# sure the ROS environment is present regardless of how this got invoked.
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${REPO_ROOT}"

BUILD_BASE="build-coverage"
INSTALL_BASE="install-coverage"
PKG_SRC="exploration_packages/vxch/voxelcodec_ros"
PKG_BUILD="${BUILD_BASE}/voxelcodec_ros"

echo "==> Building voxelcodec_ros with --coverage instrumentation"
colcon build \
  --packages-up-to voxelcodec_ros \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --cmake-args -DENABLE_COVERAGE=ON -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug

echo "==> Running vxch gtests"
# Test binaries that only depend on generated message libs via
# ament_target_dependencies (no direct target_link_libraries(${PROJECT_NAME}))
# don't inherit a build-tree RPATH to them -- source the install workspace's
# own setup so LD_LIBRARY_PATH covers every dependency, not just the ones
# that happen to already resolve without it.
# colcon's generated setup.bash references unset vars internally; nounset
# must be off while sourcing it.
set +u
source "${INSTALL_BASE}/setup.bash"
set -u
find "${PKG_BUILD}" -name "*.gcda" -delete
"${PKG_BUILD}/test_codec" --gtest_color=yes
"${PKG_BUILD}/test_ddil_stale_epoch" --gtest_color=yes
"${PKG_BUILD}/test_ros_messages" --gtest_color=yes
if [[ -x "${PKG_BUILD}/test_nav2_voxel_grid_logic" ]]; then
  "${PKG_BUILD}/test_nav2_voxel_grid_logic" --gtest_color=yes
fi
if [[ -x "${PKG_BUILD}/test_ddil_proxy_logic" ]]; then
  "${PKG_BUILD}/test_ddil_proxy_logic" --gtest_color=yes
fi
if [[ -x "${PKG_BUILD}/test_tile_scheduler" ]]; then
  "${PKG_BUILD}/test_tile_scheduler" --gtest_color=yes
fi
if [[ -x "${PKG_BUILD}/test_tile_reconstructor" ]]; then
  "${PKG_BUILD}/test_tile_reconstructor" --gtest_color=yes
fi

echo "==> Generating coverage report"
GCOVR_ARGS=(
  --root "${PKG_SRC}"
  --filter "${PKG_SRC}/src/"
  --filter "${PKG_SRC}/include/"
  --exclude ".*progressive_panel.*"
  --exclude ".*bandwidth_panel.*"
  --exclude ".*voxel_display.*"
  --object-directory "${PKG_BUILD}"
  --print-summary
)
gcovr "${GCOVR_ARGS[@]}"

if [[ "${HTML}" -eq 1 ]]; then
  mkdir -p vxch_coverage_html
  gcovr "${GCOVR_ARGS[@]}" --html-details vxch_coverage_html/index.html
  echo "==> HTML report: vxch_coverage_html/index.html"
fi
