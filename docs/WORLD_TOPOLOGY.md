# World topology (canonical editable world)

The game world is stored as:
- Vertices (positions)
- Edges (vertex pairs)
- Faces (triangles for now) + color

This data is used for:
- in-game editing and deformation
- chunk extraction (e.g. blasting stone out of a mountain)
- generating physics props on demand

## Physics props
When a chunk becomes a physics object, we generate a *proxy collider* (multi-sphere cluster).
Only active physics bodies are simulated. Bodies sleep when idle.
