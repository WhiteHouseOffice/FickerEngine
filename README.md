
# FickerEngine — WebGPU Bootstrap

Minimal C/C++ skeleton targeting WebAssembly (Emscripten) with a WebGPU renderer stub, a browser host, and deterministic `stepOnce()` for headless capture in CI.

## Build (web)

Install Emscripten, then:
```bash
emcmake cmake -S . -B webbuild -D BUILD_WEB=ON -G Ninja
cmake --build webbuild
# Copy webbuild/bin/* into webbuild/site/ and open webbuild/site/index.html
```
CI will do this automatically and upload PNG frames in `out/web_frames`.

## Repo structure

See `docs/FOLDER_STRUCTURE.md`.

## Physics

See `docs/PHYSICS.md`.
