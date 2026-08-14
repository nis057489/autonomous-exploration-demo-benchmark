cmake_minimum_required(VERSION 3.8)
project(voxelcodec_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(voxelcodec_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_msgs QUIET)
find_package(nav2_voxel_grid QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

# RViz panel (optional)
find_package(pluginlib QUIET)
find_package(rviz_common QUIET)
find_package(Qt5 QUIET COMPONENTS Widgets)

add_library(${PROJECT_NAME}
  src/codec.cpp
  src/ros_messages.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    nlohmann_json::nlohmann_json
    PkgConfig::ZSTD
)
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    std_msgs
    voxelcodec_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_executable(occupancy_grid_vxch_node
  src/occupancy_grid_vxch_node.cpp
)
target_compile_features(occupancy_grid_vxch_node PUBLIC cxx_std_17)
target_link_libraries(occupancy_grid_vxch_node ${PROJECT_NAME})
ament_target_dependencies(occupancy_grid_vxch_node
  geometry_msgs
  nav_msgs
  rclcpp
  voxelcodec_msgs
)
install(TARGETS occupancy_grid_vxch_node RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(vxch_occupancy_grid_node
  src/vxch_occupancy_grid_node.cpp
)
target_compile_features(vxch_occupancy_grid_node PUBLIC cxx_std_17)
target_link_libraries(vxch_occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(vxch_occupancy_grid_node
  geometry_msgs
  nav_msgs
  rclcpp
  voxelcodec_msgs
)
install(TARGETS vxch_occupancy_grid_node RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(ddil_proxy_node
  src/ddil_proxy_node.cpp
)
target_compile_features(ddil_proxy_node PUBLIC cxx_std_17)
ament_target_dependencies(ddil_proxy_node rclcpp voxelcodec_msgs)
install(TARGETS ddil_proxy_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(nav2_msgs_FOUND AND nav2_voxel_grid_FOUND)
  add_executable(nav2_voxel_grid_publisher
    src/nav2_voxel_grid_publisher.cpp
  )
  target_compile_features(nav2_voxel_grid_publisher PUBLIC cxx_std_17)
  target_link_libraries(nav2_voxel_grid_publisher ${PROJECT_NAME})
  ament_target_dependencies(nav2_voxel_grid_publisher
    geometry_msgs
    nav_msgs
    rclcpp
    nav2_msgs
    nav2_voxel_grid
    sensor_msgs
  )

  install(
    TARGETS nav2_voxel_grid_publisher
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "nav2_msgs or nav2_voxel_grid not found; skipping nav2_voxel_grid_publisher")
endif()

find_package(rviz_rendering QUIET)

if(rviz_common_FOUND AND Qt5_FOUND)
  set(CMAKE_AUTOMOC ON)
  add_library(progressive_panel SHARED
    src/progressive_panel.cpp
    include/voxelcodec_ros/progressive_panel.hpp
  )
  target_compile_features(progressive_panel PUBLIC cxx_std_17)
  target_include_directories(progressive_panel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(progressive_panel
    Qt5::Widgets
  )
  ament_target_dependencies(progressive_panel
    rclcpp
    rviz_common
    voxelcodec_msgs
  )

  add_library(bandwidth_panel SHARED
    src/bandwidth_panel.cpp
    include/voxelcodec_ros/bandwidth_panel.hpp
  )
  target_compile_features(bandwidth_panel PUBLIC cxx_std_17)
  target_include_directories(bandwidth_panel PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(bandwidth_panel
    Qt5::Widgets
  )
  ament_target_dependencies(bandwidth_panel
    rclcpp
    rviz_common
    sensor_msgs
  )

  install(TARGETS progressive_panel bandwidth_panel
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  if(rviz_rendering_FOUND)
    add_library(voxel_display SHARED
      src/voxel_display.cpp
      include/voxelcodec_ros/voxel_display.hpp
    )
    target_compile_features(voxel_display PUBLIC cxx_std_17)
    target_include_directories(voxel_display PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(voxel_display
      ${PROJECT_NAME}
      Qt5::Widgets
    )
    ament_target_dependencies(voxel_display
      rclcpp
      rviz_common
      rviz_rendering
      pluginlib
      voxelcodec_msgs
    )
    install(TARGETS voxel_display
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  else()
    message(STATUS "rviz_rendering not found; skipping voxel_display")
  endif()

  pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
else()
  message(STATUS "rviz_common or Qt5 not found; skipping RViz plugins")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_codec
    test/test_codec.cpp
  )
  if(TARGET test_codec)
    target_link_libraries(test_codec ${PROJECT_NAME})
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs voxelcodec_msgs nlohmann_json)
ament_package()