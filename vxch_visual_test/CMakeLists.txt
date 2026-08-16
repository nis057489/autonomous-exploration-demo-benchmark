cmake_minimum_required(VERSION 3.16)
project(vxch_visual_test CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Real vxch codec package this test harness exercises -- compiled straight
# from its own sources (not copied) so this tool can never drift from what
# actually ships. Only codec.cpp + haar_forward.hpp/types.hpp are needed:
# no ROS/rclcpp dependency, those live entirely in the *_node.cpp files this
# CLI intentionally does not build.
set(VXCH_PKG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../exploration_packages/vxch/voxelcodec_ros")
if(NOT EXISTS "${VXCH_PKG_DIR}/src/codec.cpp")
  message(FATAL_ERROR "Could not find voxelcodec_ros at ${VXCH_PKG_DIR} -- "
    "is vxch_visual_test still a direct subdirectory of the repo root?")
endif()

find_package(nlohmann_json CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

add_executable(vxch_cli
  src/main.cpp
  "${VXCH_PKG_DIR}/src/codec.cpp"
)
target_include_directories(vxch_cli PRIVATE "${VXCH_PKG_DIR}/include")
target_link_libraries(vxch_cli PRIVATE
  nlohmann_json::nlohmann_json
  PkgConfig::ZSTD
)
