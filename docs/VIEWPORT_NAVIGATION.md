# Desktop viewport navigation

The IFCPath Builder viewport is intentionally Qt-native, but its interaction model follows the same performance principle used by high-performance BIM viewers such as That Open Company's Fragments: **camera motion stays cheap and full geometry refinement happens after the camera settles**.

## Controls

| Action | Input |
| --- | --- |
| Orbit | Right-drag |
| Orbit alternative | Alt + left-drag |
| Pan | Middle-drag |
| Pan alternative | Shift + right-drag |
| Zoom | Mouse wheel / trackpad scroll |
| Precision zoom | Ctrl + wheel / trackpad scroll |
| Focus / set orbit target | Double-click a walkable surface |
| Fit model / reset orbit target | `F` or `Home` |
| Cancel Start/Goal picking | `Esc` |

In Top view, right-drag pans instead of orbiting.

## Why navigation used to feel slow

The original projected viewport rebuilt the complete `QGraphicsScene` for every orbit mouse-move event. For a preview near the 18,000-triangle cap that meant repeatedly:

1. destroying thousands of graphics items;
2. reprojecting every BIM triangle;
3. sorting projected triangles in Python;
4. recreating BIM/navmesh/graph/portal items;
5. recomputing a tessellation-weighted centre.

Pan was not implemented as a dedicated BIM navigation gesture, and wheel zoom applied one fixed factor per event regardless of high-resolution trackpad deltas.

## New interaction architecture

### 1. Transform-only pan and zoom

Pan and zoom no longer rebuild geometry. They change the existing `QGraphicsView` transform/scroll position. `AnchorUnderMouse` keeps zoom centred around the pointer.

Wheel input is continuous and exponential, so high-resolution touchpads and ordinary mouse wheels produce consistent motion. Zoom is clamped relative to the current Fit scale to prevent losing the model through extreme scaling.

### 2. Coalesced orbit updates

Raw mouse-move events can arrive much faster than the display can use them. Orbit changes therefore update camera yaw/pitch immediately but coalesce expensive projected-scene refreshes to a bounded interaction frame interval.

### 3. Interaction LOD + rest refinement

While orbiting, IFCPath renders deterministic representative subsets of:

- BIM triangles;
- navmesh cells;
- portals.

The dense metric graph is omitted during motion. After a short camera-rest delay the complete scene is restored automatically.

The subset is evenly sampled rather than randomized, preventing visual flicker between frames.

### 4. Cached level visibility

Visible BIM triangles, navmesh cells and portals are filtered once when the model/level changes instead of rescanning the entire model for every camera frame.

### 5. Stable orbit target

The default orbit target is now the physical bounding-box centre of visible geometry, not the average of all tessellated vertices. Dense geometry on one side of a building can therefore no longer pull the camera pivot away from the building centre.

Double-clicking a walkable surface uses the existing exact CDT pick to move the orbit target to that world point and focus the viewport there.

### 6. Scenario/evacuation compatibility

Live scenario and agent updates inherit the same interaction-quality state. If a simulation update arrives while the user is orbiting, it cannot force the viewport back into a full-detail redraw in the middle of camera motion.

## Performance qualification

Desktop CI includes a viewport regression that creates 4,000 BIM preview triangles, verifies that interactive rendering is bounded by the configured interaction LOD, and verifies that the exact full scene is restored afterward.

The pure navigation math is tested separately for:

- yaw wrapping / pitch limits;
- mouse-wheel and trackpad zoom equivalence;
- fit-relative zoom limits;
- deterministic representative sampling.

## Reference ideas

The implementation is original PySide6/Qt code. Architectural behaviour was compared against current That Open / Fragments documentation and community camera-control work, especially:

- keeping geometry/model state resident during camera movement;
- culling/LOD updates connected to camera update/rest events;
- smart orbit targets;
- throttling expensive camera-adjacent work.

No Three.js, Fragments, or That Open runtime dependency is introduced into IFCPath.
