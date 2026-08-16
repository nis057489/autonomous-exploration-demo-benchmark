cmake_minimum_required(VERSION 3.16)
project(vxch_wasm_demo CXX C)

if(NOT EMSCRIPTEN)
  message(FATAL_ERROR
    "This CMakeLists.txt is Emscripten-only -- configure it with emcmake, "
    "e.g.: source ~/emsdk/emsdk_env.sh && emcmake cmake -S web -B web/build")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# zstd, built from its own vendored core sources (common+compress+decompress
# only -- no legacy/deprecated/dictBuilder/dll, none of which
# compress_payload/decompress_payload in codec.cpp touch). See
# third_party/zstd for provenance.
file(GLOB ZSTD_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zstd/lib/common/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zstd/lib/compress/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zstd/lib/decompress/*.c"
)
add_library(zstd_wasm STATIC ${ZSTD_SOURCES})
target_include_directories(zstd_wasm PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/zstd/lib"
)
# huf_decompress.c's x86-asm BMI2 fast path never applies under Emscripten
# (it's gated on __x86_64__, which Emscripten doesn't define), but disabling
# it explicitly avoids relying on that gate staying correct in a future zstd
# vendor-refresh -- portable C decompress is plenty fast for demo-sized maps.
target_compile_definitions(zstd_wasm PRIVATE ZSTD_DISABLE_ASM)

# Same vendored voxelcodec_ros slice the native CLI builds against, copied
# here so this directory is buildable on its own -- see codec/VENDORED.md.
add_executable(vxch_demo
  bindings.cpp
  codec/codec.cpp
)
target_include_directories(vxch_demo PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/codec"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
)
target_link_libraries(vxch_demo PRIVATE zstd_wasm)

set(EMS_LINK_FLAGS
  "--bind"
  "-sMODULARIZE=1"
  "-sEXPORT_NAME=createVxchModule"
  "-sENVIRONMENT=web"
  "-sALLOW_MEMORY_GROWTH=1"
  "-sEXPORTED_RUNTIME_METHODS=[]"
  "-sNO_EXIT_RUNTIME=1"
)
list(JOIN EMS_LINK_FLAGS " " EMS_LINK_FLAGS_STR)
set_target_properties(vxch_demo PROPERTIES
  LINK_FLAGS "${EMS_LINK_FLAGS_STR}"
  OUTPUT_NAME "vxch"
)

# vxch.js/vxch.wasm build directly into site/, next to the hand-authored
# index.html/app.js/style.css -- so a plain rebuild always picks up frontend
# edits with no separate copy step to go stale. publish_demo.sh deploys this
# directory as-is.
set_target_properties(vxch_demo PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/site"
)
