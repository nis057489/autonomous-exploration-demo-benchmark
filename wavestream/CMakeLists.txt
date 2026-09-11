cmake_minimum_required(VERSION 3.16)
project(wavestream LANGUAGES CXX)

# The single header lives beside the ROS adapters so the existing package
# installation exports it too. This project builds without ROS or codec.cpp.
set(WAVESTREAM_PACKAGE "${CMAKE_CURRENT_SOURCE_DIR}/../exploration_packages/vxch/voxelcodec_ros")
find_path(WAVESTREAM_ZSTD_INCLUDE_DIR zstd.h REQUIRED)
find_library(WAVESTREAM_ZSTD_LIBRARY NAMES zstd REQUIRED)
add_library(wavestream INTERFACE)
add_library(wavestream::wavestream ALIAS wavestream)
target_compile_features(wavestream INTERFACE cxx_std_17)
target_include_directories(wavestream INTERFACE
  "${WAVESTREAM_PACKAGE}/include" "${WAVESTREAM_ZSTD_INCLUDE_DIR}")
target_link_libraries(wavestream INTERFACE "${WAVESTREAM_ZSTD_LIBRARY}")

option(WAVESTREAM_BUILD_TESTS "Build standalone codec and adapter tests" ON)
option(WAVESTREAM_BUILD_EXAMPLE "Build the paper pipeline example" ON)
if(WAVESTREAM_BUILD_EXAMPLE)
  add_executable(wavestream_example examples/paper_pipeline.cpp)
  target_link_libraries(wavestream_example PRIVATE wavestream::wavestream)
endif()
if(WAVESTREAM_BUILD_TESTS)
  enable_testing()
  add_executable(wavestream_test tests/codec_test.cpp tests/second_translation_unit.cpp)
  target_link_libraries(wavestream_test PRIVATE wavestream::wavestream)
  add_test(NAME wavestream_codec COMMAND wavestream_test)

  # Only the compatibility test needs the legacy archive codec and JSON.
  find_path(WAVESTREAM_JSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../vxch_visual_test/web/third_party" REQUIRED)
  add_executable(wavestream_adapter_test tests/adapter_test.cpp "${WAVESTREAM_PACKAGE}/src/codec.cpp")
  target_include_directories(wavestream_adapter_test PRIVATE "${WAVESTREAM_JSON_INCLUDE_DIR}")
  target_link_libraries(wavestream_adapter_test PRIVATE wavestream::wavestream)
  add_test(NAME wavestream_adapters COMMAND wavestream_adapter_test)
endif()
