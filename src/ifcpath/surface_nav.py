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
