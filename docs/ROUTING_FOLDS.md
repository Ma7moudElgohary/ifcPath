# Fold-aware surface routing

IfcPath treats the continuous `NavCell` surface as the routing source of truth.

A classic funnel/string-pulling algorithm is planar. Running it directly in IFC
XY is correct for a flat floor, but it is not correct for a folded indoor surface
such as discrete stair treads and landings. Different walkable cells can overlap
in XY while being separated in Z; an XY-only funnel may then collapse several
folds into one 3D chord through empty space.

The surface router therefore uses two modes inside one corridor:

1. Consecutive coplanar cells are projected into their local metric plane and
   shortened with the normal funnel algorithm. This covers floors and planar
   ramps without bias from the building XY projection.
2. Every non-coplanar fold is preserved explicitly. The stored crossing portal
   is projected onto each adjacent triangle, producing an exit point and an
   entry point. The resulting bridge is part of the route and cannot be removed
   by planar string pulling.

The browser route demo implements the same rule as the Python routing kernel so
visible routes and exported/runtime navigation use the same geometry principle.
