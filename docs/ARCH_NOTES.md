# Architecture Notes

## Layout
- `engine/core` – loop, timing, config
- `engine/render` – backends; WebGPU first
- `runtime` – entrypoints (web/native)
- `web` – browser host (index.html), PWA hooks
- `scripts/ci` – capture scripts

## Conventions
- PIMPL for large classes (`Engine`), destructor in `.cpp`
- Deterministic stepping via `window.stepOnce()`
- Visual CI is the source of truth for graphics regressions
