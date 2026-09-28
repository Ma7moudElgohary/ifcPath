# Real IFC Qualification Matrix

Use this matrix before treating IFCPath graph generation as robust enough for production Digital Twin navigation.

| Fixture | Required IFC content | Expected result |
|---|---|---|
| Single room | 1 storey, 1 space, 1 exterior door | One connected walk component and one exit |
| Two rooms | 2 spaces, internal door, exterior door | Both spaces connected and exit reachable |
| Corridor | Multiple rooms opening to corridor | Door portals connect rooms to corridor without cross-wall shortcuts |
| Two storeys | Floors plus stair/stair flights | One connected cross-level graph |
| Ramp | Sloped walkable ramp between levels | Traversable vertical connection under slope limit |
| Multiple exits | At least two exterior doors | Both exits classified and reachable |
| Disconnected BIM | Deliberately isolated room | Validator reports disconnected component / unreachable navigation |
| Imperfect semantics | Missing space-boundary relationships | Geometric door-space fallback still produces useful connectivity |

For every real fixture run:

```bash
ifcpath build fixture.ifc -o fixture.inav --strict
ifcpath validate fixture.inav --json > fixture.validation.json
```

Then test in Unreal:

1. Load the `.inav` file.
2. Call `DrawDebugGraph`.
3. Pick arbitrary world start/end positions and call `FindPathFromWorldPositions`.
4. Draw the result with `DrawDebugPath`.
5. Block a door portal and confirm a route either changes or correctly becomes unreachable.

Important metrics are connected-component count, largest-component ratio, spaces with no navigation, isolated nodes, classified exits, and exit reachability. These metrics should drive whether the current point-graph backend is sufficient or whether a Recast backend is necessary.
