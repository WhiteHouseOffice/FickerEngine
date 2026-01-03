# Physics

Physics lives in `engine/game/physics/` so `engine/geom/` can stay mesh-only.

## Goals
- **Simple and fast** (support many objects)
- Keep integration minimal so we don't break rendering or camera controls
- Start with primitives we can reason about (AABB boxes + a player proxy sphere)

## Current model

### RigidBody
`RigidBody` is ultra-light:
- Linear velocity only (no rotation yet)
- `invMass == 0` means static
- Gravity toggle + linear damping + force accumulator

### PhysicsWorld
`PhysicsWorld::step(dt, objects)`:
- Semi-implicit Euler integration
- Optional infinite ground plane at `y = groundY`
- Optional AABB-vs-AABB collision resolution for objects that have a box collider
- Optional kinematic **player sphere proxy** that pushes dynamic boxes (does not move the player)

### Colliders
Colliders are owned by `GameObject`, not by `geom` meshes:
- `enableBoxCollider(halfExtents)`
- This keeps `engine/geom/` clean and easy to exclude for small code shares

## Test scene
`Scene::init()` spawns:
- Static platforms (matching the debug boxes)
- Dynamic cubes (one placed on a platform so you can push it off)
- Additional cubes on the ground

`Scene::renderDebug()` draws all colliders as wire boxes in native builds.
