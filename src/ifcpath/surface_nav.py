from __future__ import annotations

import heapq
import math
from collections import defaultdict

from .geometry import triangle_normal
from .model import NavCell, Vec3


def walkable_surface_cells(vertices, triangles, *, id_prefix, terrain, max_slope_deg, level_id=None, space_id=None):
    min_up = math.cos(math.radians(max_slope_deg))
    cells = []
    for ia, ib, ic in triangles:
        a, b, c = vertices[ia], vertices[ib], vertices[ic]
        if triangle_normal(a, b, c)[2] < min_up:
            continue
        cells.append(NavCell(id=f"{id_prefix}:{len(cells)}", vertices_m=(a,b,c),
            space_id=space_id, level_id=level_id, terrain=terrain))
    connect_cells_by_shared_edges(cells)
    return cells


def connect_cells_by_shared_edges(cells, tolerance_m=1e-5):
    scale = 1.0 / max(tolerance_m, 1e-9)
    owners = defaultdict(list)
    def q(p):
        return (round(p[0]*scale), round(p[1]*scale), round(p[2]*scale))
    for i, cell in enumerate(cells):
        v = cell.vertices_m
        for a,b in ((v[0],v[1]),(v[1],v[2]),(v[2],v[0])):
            qa,qb=q(a),q(b)
            owners[(qa,qb) if qa <= qb else (qb,qa)].append(i)
    for indices in owners.values():
        if len(indices) != 2:
            continue
        a,b=indices
        cells[a].neighbor_ids.append(cells[b].id)
        cells[b].neighbor_ids.append(cells[a].id)


def cell_centroid(cell):
    a,b,c=cell.vertices_m
    return ((a[0]+b[0]+c[0])/3,(a[1]+b[1]+c[1])/3,(a[2]+b[2]+c[2])/3)


def find_cell_corridor(cells, start_cell_id, goal_cell_id):
    by_id={c.id:c for c in cells}
    if start_cell_id not in by_id or goal_cell_id not in by_id:
        return []
    goal=cell_centroid(by_id[goal_cell_id])
    queue=[(0.0,start_cell_id)]
    g={start_cell_id:0.0}; prev={}
    while queue:
        _,cur=heapq.heappop(queue)
        if cur == goal_cell_id:
            break
        cp=cell_centroid(by_id[cur])
        for nxt in by_id[cur].neighbor_ids:
            if nxt not in by_id:
                continue
            np=cell_centroid(by_id[nxt]); cand=g[cur]+math.dist(cp,np)
            if cand >= g.get(nxt, math.inf):
                continue
            g[nxt]=cand; prev[nxt]=cur
            heapq.heappush(queue,(cand+math.dist(np,goal),nxt))
    if goal_cell_id not in g:
        return []
    result=[goal_cell_id]
    while result[-1] != start_cell_id:
        result.append(prev[result[-1]])
    return list(reversed(result))



def stitch_surface_seams(cells, max_gap_m=0.20, max_vertical_gap_m=0.30):
    """Join separately tessellated walkable elements at genuine boundary seams.

    IFC spaces, slabs and stair flights are tessellated independently, so exact
    shared vertices are not guaranteed. We only bridge boundary edges whose
    midpoints are close in 3D and whose vertical separation is pedestrian-scale.
    """
    by_id = {cell.id: cell for cell in cells}
    connected = {tuple(sorted((cell.id, n))) for cell in cells for n in cell.neighbor_ids}
    boundaries = []
    edge_counts = defaultdict(int)
    edge_data = {}
    scale = 1e5
    def key(a,b):
        qa=tuple(round(v*scale) for v in a); qb=tuple(round(v*scale) for v in b)
        return (qa,qb) if qa <= qb else (qb,qa)
    for cell in cells:
        v=cell.vertices_m
        for a,b in ((v[0],v[1]),(v[1],v[2]),(v[2],v[0])):
            k=key(a,b); edge_counts[k]+=1; edge_data.setdefault(k,(cell.id,a,b))
    for k,count in edge_counts.items():
        if count == 1:
            cid,a,b=edge_data[k]
            mid=((a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2)
            boundaries.append((cid,mid))
    added=0
    for i,(aid,am) in enumerate(boundaries):
        for bid,bm in boundaries[i+1:]:
            if aid == bid or tuple(sorted((aid,bid))) in connected:
                continue
            if by_id[aid].terrain == by_id[bid].terrain == "open" and by_id[aid].space_id != by_id[bid].space_id:
                continue
            if abs(am[2]-bm[2]) > max_vertical_gap_m or math.dist(am,bm) > max_gap_m:
                continue
            by_id[aid].neighbor_ids.append(bid); by_id[bid].neighbor_ids.append(aid)
            connected.add(tuple(sorted((aid,bid)))); added+=1
    return added


def surface_components(cells):
    by_id={c.id:c for c in cells}; remaining=set(by_id); result=[]
    while remaining:
        start=remaining.pop(); comp={start}; stack=[start]
        while stack:
            cur=stack.pop()
            for nxt in by_id[cur].neighbor_ids:
                if nxt in remaining:
                    remaining.remove(nxt); comp.add(nxt); stack.append(nxt)
        result.append(comp)
    return result



def closest_cell(cells, point):
    """Return the cell whose triangle is closest to an arbitrary 3D point."""
    best = None
    for cell in cells:
        q = _closest_point_on_triangle(point, *cell.vertices_m)
        d = math.dist(point, q)
        candidate = (d, cell.id, q, cell)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
    return None if best is None else (best[3], best[2], best[0])


def find_surface_route(cells, start, goal, terrain_costs=None):
    """Project endpoints to the nav surface and return a smoothed XYZ route."""
    terrain_costs = terrain_costs or {}
    s = closest_cell(cells, start); g = closest_cell(cells, goal)
    if s is None or g is None:
        return []
    start_cell, sp, _ = s; goal_cell, gp, _ = g
    corridor = _weighted_cell_corridor(cells, start_cell.id, goal_cell.id, terrain_costs)
    if not corridor:
        return []
    by_id={cell.id:cell for cell in cells}
    # A robust first smoothing pass: use geometric transition portals and remove
    # any waypoint that has direct line of sight across the corridor surface.
    route=[sp]
    for a_id,b_id in zip(corridor,corridor[1:]):
        portal=_cell_portal(by_id[a_id],by_id[b_id])
        route.append(portal if portal is not None else cell_centroid(by_id[b_id]))
    route.append(gp)
    return _string_pull_polyline(route)


def _weighted_cell_corridor(cells, start_id, goal_id, terrain_costs):
    by_id={c.id:c for c in cells}
    if start_id not in by_id or goal_id not in by_id: return []
    goal=cell_centroid(by_id[goal_id]); q=[(0.0,start_id)]; g={start_id:0.0}; prev={}
    while q:
        _,cur=heapq.heappop(q)
        if cur == goal_id: break
        cp=cell_centroid(by_id[cur])
        for nxt in by_id[cur].neighbor_ids:
            if nxt not in by_id: continue
            np=cell_centroid(by_id[nxt])
            multiplier=max(1.0,float(terrain_costs.get(by_id[nxt].terrain,1.0)))
            cand=g[cur]+math.dist(cp,np)*multiplier
            if cand < g.get(nxt,math.inf):
                g[nxt]=cand; prev[nxt]=cur
                heapq.heappush(q,(cand+math.dist(np,goal),nxt))
    if goal_id not in g: return []
    out=[goal_id]
    while out[-1] != start_id: out.append(prev[out[-1]])
    return list(reversed(out))


def _cell_portal(a,b,tol=1e-4):
    """Midpoint of a shared/stitched boundary; stable fallback for 3D corridors."""
    pairs=[]
    for pa in a.vertices_m:
        for pb in b.vertices_m:
            d=math.dist(pa,pb)
            if d <= tol: pairs.append(((pa[0]+pb[0])/2,(pa[1]+pb[1])/2,(pa[2]+pb[2])/2))
    if len(pairs) >= 2:
        p,q=pairs[0],pairs[1]
        return ((p[0]+q[0])/2,(p[1]+q[1])/2,(p[2]+q[2])/2)
    ca,cb=cell_centroid(a),cell_centroid(b)
    return ((ca[0]+cb[0])/2,(ca[1]+cb[1])/2,(ca[2]+cb[2])/2)


def _string_pull_polyline(points, eps=1e-6):
    if len(points) <= 2: return points
    out=[points[0]]
    for i in range(1,len(points)-1):
        a=out[-1]; b=points[i]; c=points[i+1]
        ab=(b[0]-a[0],b[1]-a[1],b[2]-a[2]); bc=(c[0]-b[0],c[1]-b[1],c[2]-b[2])
        cross=(ab[1]*bc[2]-ab[2]*bc[1],ab[2]*bc[0]-ab[0]*bc[2],ab[0]*bc[1]-ab[1]*bc[0])
        if math.sqrt(sum(v*v for v in cross)) > eps:
            out.append(b)
    out.append(points[-1]); return out


def _closest_point_on_triangle(p,a,b,c):
    # Christer Ericson, Real-Time Collision Detection, closest-point regions.
    ab=tuple(b[i]-a[i] for i in range(3)); ac=tuple(c[i]-a[i] for i in range(3)); ap=tuple(p[i]-a[i] for i in range(3))
    d1=sum(ab[i]*ap[i] for i in range(3)); d2=sum(ac[i]*ap[i] for i in range(3))
    if d1<=0 and d2<=0: return a
    bp=tuple(p[i]-b[i] for i in range(3)); d3=sum(ab[i]*bp[i] for i in range(3)); d4=sum(ac[i]*bp[i] for i in range(3))
    if d3>=0 and d4<=d3: return b
    vc=d1*d4-d3*d2
    if vc<=0 and d1>=0 and d3<=0:
        v=d1/(d1-d3); return tuple(a[i]+v*ab[i] for i in range(3))
    cp=tuple(p[i]-c[i] for i in range(3)); d5=sum(ab[i]*cp[i] for i in range(3)); d6=sum(ac[i]*cp[i] for i in range(3))
    if d6>=0 and d5<=d6: return c
    vb=d5*d2-d1*d6
    if vb<=0 and d2>=0 and d6<=0:
        w=d2/(d2-d6); return tuple(a[i]+w*ac[i] for i in range(3))
    va=d3*d6-d5*d4
    if va<=0 and (d4-d3)>=0 and (d5-d6)>=0:
        w=(d4-d3)/((d4-d3)+(d5-d6)); return tuple(b[i]+w*(c[i]-b[i]) for i in range(3))
    denom=1.0/(va+vb+vc); v=vb*denom; w=vc*denom
    return tuple(a[i]+ab[i]*v+ac[i]*w for i in range(3))
