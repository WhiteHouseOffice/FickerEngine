# FickerEngine folder structure

Keep responsibilities separated. This makes the engine easy to reason about and easy to ship in smaller subsets.

## engine/core/
Engine loop + platform glue + timing.
Input API is authoritative here: `engine/core/Input.h` (do not add a second input system).

## engine/math/
Math helpers.

## engine/geom/
CPU-only mesh/geometry generation.
- Produces vertices/indices in CPU memory
- No gameplay logic
- No physics logic
- No GPU calls

## engine/render/
GPU upload + draw only.
- Render backend integration
- No gameplay logic

## engine/game/
Gameplay + simulation (scene graph, objects, movement, physics integration).

### engine/game/physics/
Ultra-light physics aimed at scaling to many objects:
- `RigidBody` (linear motion only)
- `PhysicsWorld` (gravity + ground plane + optional AABB box collisions)
- Optional kinematic player-sphere proxy used to *push* dynamic boxes

## engine/platform/web/
Web/Emscripten entry + web platform glue.

## engine/platform/native/
Native desktop entry + native platform glue.
