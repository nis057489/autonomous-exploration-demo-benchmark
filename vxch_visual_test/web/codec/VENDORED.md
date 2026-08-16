# Vendored from exploration_packages/vxch/voxelcodec_ros

`codec.cpp` and `voxelcodec_ros/{codec.hpp,haar_forward.hpp,types.hpp}` in this
directory are byte-for-byte copies of the files at
`../../exploration_packages/vxch/voxelcodec_ros/{src,include/voxelcodec_ros}`,
as of commit `37374d9e3a4f26974028d422a2d4310344459b4e`.

They're copied rather than referenced by relative path (unlike
`vxch_visual_test/CMakeLists.txt`'s native build, which builds directly
against the real package) because the Emscripten toolchain output only ever
publishes as a static site (see `../../README.md`'s web section) -- there's
no reason for that publish step to also walk back out to the ROS package
tree, and copying keeps `web/` buildable on its own.

If `codec.cpp`/`haar_forward.hpp`/`types.hpp` change upstream, re-copy them
here and rebuild before republishing the demo -- there's no automatic sync.
