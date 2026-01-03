# Physics proxies

Dynamic props do not simulate collisions against their full render mesh.

Instead we generate a collision proxy at spawn time:
- multi-sphere cluster (fast + stable)
- cube starts as 8 corner spheres
- later: more spheres for complex chunks

This keeps physics cheap and allows sleeping inactive bodies.
