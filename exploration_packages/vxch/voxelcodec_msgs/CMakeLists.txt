cmake_minimum_required(VERSION 3.8)
project(voxelcodec_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/MetadataEntry.msg"
  "msg/VoxelChannelDescriptor.msg"
  "msg/VoxelManifest.msg"
  "msg/VoxelChannel.msg"
)

set(srv_files
  "srv/SetBand.srv"
  "srv/SetLevel.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()