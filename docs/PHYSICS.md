# Physics

Physics code lives in `engine/game/physics/`.

## Current approach (dev/body-physics)

**Rigid-body boxes (sequential impulse solver)**

Dynamic props are simulated as oriented boxes (OBB) with:
- position + orientation (quat)
- linear + angular velocity
- mass + inertia (box inertia)
- friction, restitution
- sleeping (idle bodies stop simulating)

Static world collision is currently represented as:
- an infinite ground plane (`y = 0`)
- a small set of static AABBs (platforms) built from `Scene` objects

The player is represented to physics as a **kinematic sphere**:
- Scene resolves sphere-vs-props/world collisions
- the corrected sphere position/velocity is applied back to `Game` so you can stand on props and push them.

## Why this

This is the “Source / Havok style” baseline that makes:
- stacking stable (standing on crates)
- interpenetration rare (with small tolerance)
- tipping possible (angular velocity + OBB contacts)

## Next steps

- Replace the “platform AABBs” with the world topology store (`WorldMesh`) as the static collision source.
- Add “compound colliders” (multiple boxes per rigid body) for concave/dented shapes.
- Add on-demand collider generation from destructed chunks (voxel/mesh → compound boxes / convex decomposition).
