# Level-aware tiled map format

The online map is split into 25 m horizontal tiles. Coordinates use mathematical floor,
so `-0.001 m` belongs to tile `-1`, while `-25.0 m` belongs to tile `-1`. A tile is
identified by `(level_id, tile_x, tile_y)`; XY coordinates alone never combine floors.

`manifest.yaml` is written last and contains the schema version, map ID, monotonically
increasing generation, frame, tile size, creation/config hashes, descriptor-index path,
level z-bands, and one record per tile. Every tile record contains bounds, point count,
voxel size, relative file path, and checksum (`fnv1a64:` in schema version 1).

Localization validates the complete manifest before loading point data. It rejects an
unsupported schema, frame mismatch, duplicate tile, unknown level, overlapping level
bands, missing file, or checksum mismatch. Paths are relative to the generation root.
A map covering 100,000 m² needs roughly 160 occupied 25 m tiles for full coverage;
only the current same-level neighborhood is loaded online.
