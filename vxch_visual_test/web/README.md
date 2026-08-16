# vxch WASM demo

Browser build of the vxch codec test tool -- same codec math as `../src`
and `../gui`, compiled to WebAssembly with Emscripten instead of run as a
native CLI/Tkinter app. Built for public science-communication hosting
(see `../publish_demo.sh`) where a link is easier to share than "clone,
build, run inside distrobox."

## Layout

```
bindings.cpp          Embind wrapper exposing VxchDemo to JS (in-memory
                       equivalent of src/main.cpp's gen-map/encode/step,
                       reusing the exact same codec/ calls)
codec/                 vendored voxelcodec_ros slice -- see codec/VENDORED.md
third_party/zstd/      vendored zstd core lib sources (common+compress+decompress)
third_party/nlohmann/  vendored nlohmann/json single-header (matches the
                       version the native build links against)
site/                  the actual site: hand-authored index.html/app.js/
                       style.css, plus vxch.js/vxch.wasm built directly
                       into this directory (gitignored -- see CMakeLists.txt)
CMakeLists.txt         Emscripten-only build (errors if not run via emcmake)
```

## Build

Needs the Emscripten SDK:

```
git clone https://github.com/emscripten-core/emsdk.git ~/emsdk
~/emsdk/emsdk install latest
~/emsdk/emsdk activate latest
```

Then, every time you want to (re)build:

```
source ~/emsdk/emsdk_env.sh
cd vxch_visual_test/web
emcmake cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # once
cmake --build build -j                                    # every rebuild
```

`site/vxch.js` + `site/vxch.wasm` land next to the hand-authored files.
Serve `site/` with any static file server to try it locally, e.g.
`python3 -m http.server -d site 8000`.

## Publishing

`../publish_demo.sh` copies `site/`'s deployable files (not this
directory's C++/vendored source) to a separate local checkout of the
public demo repo and pushes. See that script for the target repo path --
by design, the public repo gets only the built static site, not this
directory.

## Notes

- `codec/VENDORED.md` explains why the codec files here are copies, not
  references, and how to re-sync them if `../../exploration_packages/vxch`
  changes.
- Perf: `bindings.cpp`'s `VxchDemo` reconstructs and pastes only the tile
  that just received a band on each `sendNext()` call, not every tracked
  tile from scratch -- for a large map (hundreds of tiles), redoing every
  other tile's already-unchanged reconstruction on every single band was
  the dominant cost of "Send All", not the codec math itself.
