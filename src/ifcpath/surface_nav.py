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
