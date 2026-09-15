cmake_minimum_required(VERSION 3.8)
project(bme_ros2_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(ground_truth_coverage scripts/ground_truth_coverage.cpp)
target_compile_features(ground_truth_coverage PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Every native laser beam is evaluated; avoid slowing the simulator in
  # colcon's default configuration, which otherwise has no optimization.
  target_compile_options(ground_truth_coverage PRIVATE -O2)
endif()
target_link_libraries(ground_truth_coverage gz-transport13::core gz-msgs10::core)
install(TARGETS ground_truth_coverage DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(_share_dirs
  launch
  worlds
  rviz
  urdf
  meshes
  config
  maps
)

set(_existing_share_dirs "")
foreach(_dir IN LISTS _share_dirs)
  if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    list(APPEND _existing_share_dirs "${_dir}")
  endif()
endforeach()

if(_existing_share_dirs)
  install(DIRECTORY
    ${_existing_share_dirs}
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/team_map_fusion.py
  scripts/per_robot_map_compositor.py
  scripts/link_scheduler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
