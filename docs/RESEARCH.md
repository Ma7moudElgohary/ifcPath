# IFCPath research basis

IFCPath is intended to be a production-oriented implementation assembled from open BIM / computational-geometry building blocks, not a new navigation algorithm invented from scratch. This note records the research choices that directly influence the generator and runtime architecture.

## 1. IFC navigation survey

Liu, Li, Zlatanova & van Oosterom, **Indoor navigation supported by the Industry Foundation Classes (IFC): A survey**, Automation in Construction 121 (2021), 103436. DOI: https://doi.org/10.1016/j.autcon.2020.103436

Key implications for IFCPath:

- IFC geometry, semantics and topology should be used together rather than reducing BIM to a raw triangle soup.
- Navigation-model generation is the most active IFC-navigation research area, but robust full automation remains difficult.
- Vertical navigation, conversion to standardized indoor-navigation models and integration of live operational data are explicit research gaps.

IFCPath response:

- Preserve `IfcSpace`, `IfcDoor`, storey and vertical-circulation semantics in INAV.
- Keep geometry generation independent from runtime dynamic state.
- Qualify against real IFC files instead of assuming ideal IFC semantics.

## 2. Early IFC grid / Fast Marching approach

Lin et al., **The IFC-based path planning for 3D indoor spaces**, Advanced Engineering Informatics 27(2) (2013), 189-205. DOI: https://doi.org/10.1016/j.aei.2012.10.001

Method:

1. extract IFC geometric and semantic information;
2. discretize it into a planar grid;
3. compute routes with the Fast Marching Method.

IFCPath uses this work as evidence that geometry and semantics must remain coupled, but does **not** adopt a dense whole-building grid because the Digital Twin use case needs a compact reusable network and target-independent interchange.

## 3. i-GIT: polygon regularization + constrained Delaunay triangulation

Lin & Lin, **Intelligent generation of indoor topology (i-GIT) for human indoor pathfinding based on IFC models and 3D GIS technology**, Automation in Construction 94 (2018). DOI: https://doi.org/10.1016/j.autcon.2018.07.016

The paper is directly relevant to the IFCPath metric network:

- derives indoor boundaries directly from IFC geometry;
- regularizes polygons to reduce excessive vertices;
- uses **Constrained Delaunay Triangulation (CDT)** for level paths;
- separately generates non-level paths for stairs, ramps and elevators;
- reported average path availability above 96% and route-length error around 5% in its validation cases.

IFCPath response:

- CDT is the preferred floor backend.
- `IfcSpace` bottom geometry is converted to a valid 2D polygon before triangulation.
- CDT boundaries are preserved and the triangle dual graph forms a sparse metric network.
- stairs and ramps remain a separate vertical-transfer layer.
- the older sampled-radius graph remains a fallback for malformed IFC spaces.

Implementation uses Shapely 2.1 / GEOS constrained Delaunay triangulation rather than copying research code.

## 4. Straight-skeleton indoor networks

Fu, Liu, Qi & Issa, **Generating straight skeleton-based navigation networks with Industry Foundation Classes for indoor way-finding**, Automation in Construction 112 (2020), 103057. DOI: https://doi.org/10.1016/j.autcon.2019.103057

Important findings:

- medial/straight-skeleton routes align well with human indoor wayfinding;
- irregular rooms and detours require more than simplistic centerlines;
- doors should be connected explicitly to corridor networks;
- floor-to-floor navigation needs dedicated stair/elevator algorithms;
- blocked spaces must be excluded from network generation.

IFCPath response:

- CDT dual paths are our permissively licensed first approximation of a medial-style network.
- a future optional skeleton simplification stage may reduce corridor graphs further.
- CGAL Straight Skeleton is **not** selected as a required dependency because that package is GPL/commercial licensed; IFCPath should keep a permissive commercial deployment path.

## 5. BiMov: hierarchical navigation and user profiles

Hamieh et al., **A BIM-based method to plan indoor paths**, Automation in Construction 113 (2020), 103120. DOI: https://doi.org/10.1016/j.autcon.2020.103120

Relevant architecture:

- separates BIM topological analysis from route-user semantics;
- accounts for user/object profiles and operational accessibility of spaces/transitions;
- structures navigation at multiple levels of detail.

IFCPath response:

- INAV should evolve into two explicit layers:
  - **semantic dual graph**: spaces and transfer portals;
  - **metric movement graph/navmesh**: exact local movement geometry.
- route profiles (public, wheelchair, firefighter, security, maintenance) should affect transfer availability and cost without regenerating IFC geometry.

## 6. IFC-Graph: semantics-first building connectivity

Zhu et al., **Semantics-based connectivity graph for indoor pathfinding powered by IFC-Graph**, Automation in Construction 171 (2025), 106019. DOI: https://doi.org/10.1016/j.autcon.2025.106019

This is especially relevant to the Digital Twin architecture because it separates **semantic building connectivity** from detailed metric path geometry. The paper constructs horizontal connectivity primarily from IFC semantics and relationships, including spaces, space boundaries and connecting elements, and uses geometry where needed for vertical links and visualisation. It demonstrates object-to-object queries such as space-to-space, space-to-exit and space-to-facility pathfinding.

Important implications:

- geometry-only graph generation can be unnecessarily expensive and semantically weak;
- `IfcSpace` and `IfcRelSpaceBoundary*` should be first-class graph inputs, not only validation metadata;
- doors and other connecting elements should become explicit semantic transitions;
- vertical connectivity needs an explicit representation because ordinary IFC relationships are often insufficient to express storey-to-storey navigability;
- graph semantics make it easier to attach operational information for Digital Twin scenarios.

IFCPath response:

- INAV will expose an explicit semantic connectivity layer in addition to the metric floor graph;
- horizontal semantic transitions are derived from IFC space-boundary relationships when available, with geometry-based inference only as fallback;
- vertical transitions will be modeled explicitly for stairs, ramps, elevators and escalators instead of being left as accidental proximity edges;
- target software such as Unreal can perform hierarchical routing: semantic building route first, local metric route second.

## 7. Obstacles and space subdivision

Xu et al., **BIM-based indoor path planning considering obstacles**, ISPRS Annals IV-2/W4 (2017), 417-423. DOI: https://doi.org/10.5194/isprs-annals-IV-2-W4-417-2017

The work shows why furniture/fixed obstacles cannot be ignored in accurate indoor movement geometry and investigates both 2D and 3D space subdivision methods.

IFCPath roadmap:

- subtract fixed obstacle footprints (columns and selected fixed furnishing/equipment classes) from each walkable space polygon before CDT;
- keep movable furniture as optional scenario data rather than permanently burning it into every network;
- support agent-clearance erosion of walkable polygons.

## 8. IndoorGML / primal-dual model

OGC **IndoorGML 2.0 Part 1 – Conceptual Model** and IndoorGML 1.1: https://www.ogc.org/standards/indoorgml/

IndoorGML provides the cleanest standard conceptual model for IFCPath's portable schema:

- primal space: navigable cells / geometry;
- dual space: `State` nodes and `Transition` edges;
- connectivity is distinct from geometric adjacency;
- transfer spaces represent doors and vertical transitions;
- anchor spaces connect indoor and outdoor networks.

IFCPath should remain JSON/engine-friendly rather than emitting GML internally, but INAV semantics should remain mappable to IndoorGML concepts.

## 9. Dynamic Digital Twin routing

Rashidian & Malek, **An IFC-based framework for semantic integration of BIM and mobile crowd sensing in real-time evacuation routing**, Advanced Engineering Informatics 72 (2026), 104500. DOI: https://doi.org/10.1016/j.aei.2026.104500

The study transforms IFC information into a dynamic decision graph and updates route weights using operational factors including hazard proximity, travel time, exit availability, environmental safety and crowd density.

Deng et al., **An ontology-based approach to dynamic indoor fire emergency evacuation path planning with BIM integration**, Journal of Building Engineering (2025), 112562. DOI: https://doi.org/10.1016/j.jobe.2025.112562

The work emphasizes dynamically updating accessibility of navigation nodes from fire/sensor information.

IFCPath response:

- geometry is generated once;
- runtime changes update portal/space/edge state and cost;
- blocked spaces are treated as no-entry while allowing occupants already inside to escape;
- hazard/crowd/safety data should update route cost without rebuilding INAV.

## Current algorithm decision

```text
IFC
 ↓
IfcOpenShell semantics + world geometry
 ↓
┌──────────────────────────────┬──────────────────────────────┐
│ semantic connectivity       │ metric movement geometry      │
│ IfcSpace / boundaries       │ IfcSpace bottom polygon       │
│ doors / vertical transfers  │ make-valid / clearance        │
│ space-to-space graph        │ constrained Delaunay          │
└──────────────┬───────────────┴──────────────┬───────────────┘
               └───────────────┬──────────────┘
                               ↓
                 hierarchical INAV navigation model
                               ↓
                     Unreal / other targets
```

The semantic graph answers **which spaces/transfers must be traversed**. The metric graph answers **the exact local movement geometry inside each traversed space**.

## Accuracy gates

A backend is not accepted because it looks correct visually. Real IFC qualification should measure at least:

- split spaces;
- isolated nodes;
- portal-side attachment failures;
- exit-reachable ratio;
- graph density;
- component count per legitimate navigation domain;
- route-length error against known/manual reference paths;
- accessibility-profile correctness;
- stair/elevator connectivity;
- obstacle and clearance compliance.

The buildingSMART Duplex fixture is the first gate, not the final validation set. The fixture matrix should expand to irregular rooms, atria, multiple stairs, ramps, elevators, narrow doors, furniture/columns and disconnected building wings.
