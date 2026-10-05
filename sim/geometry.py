"""Grid helpers shared by both robots: line of sight, ray casting, path planning."""
import heapq
import math

N8 = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


def neighbours(cell, passable):
    r, c = cell
    for dr, dc in N8:
        n = (r + dr, c + dc)
        if not passable(n):
            continue
        if dr and dc and not (passable((r + dr, c)) and passable((r, c + dc))):
            continue                              # no corner cutting
        yield n, (math.sqrt(2) if dr and dc else 1.0)


def bresenham(a, b):
    (r0, c0), (r1, c1) = a, b
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
    err = dc - dr
    r, c = r0, c0
    while True:
        yield r, c
        if (r, c) == (r1, c1):
            return
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


def line_of_sight(a, b, blocks):
    return not any(blocks(p) for p in list(bresenham(a, b))[1:-1])


def dijkstra(start, passable, cost=None):
    """Distances (in cells) from start to every reachable cell."""
    dist = {start: 0.0}
    pq = [(0.0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in neighbours(u, passable):
            nd = d + w * (cost(v) if cost else 1.0)
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    return dist


def astar(start, goal, passable, cost=None, limit=math.inf):
    if start == goal:
        return [start]
    h = lambda p: math.hypot(p[0] - goal[0], p[1] - goal[1])
    g = {start: 0.0}
    came = {}
    pq = [(h(start), start)]
    while pq:
        _, u = heapq.heappop(pq)
        if u == goal:
            path = [u]
            while u in came:
                u = came[u]
                path.append(u)
            return path[::-1]
        if g[u] > limit:
            break
        for v, w in neighbours(u, passable):
            ng = g[u] + w * (cost(v) if cost else 1.0)
            if ng < g.get(v, math.inf):
                g[v] = ng
                came[v] = u
                heapq.heappush(pq, (ng + h(v), v))
    return None


def path_length(path):
    return sum(math.hypot(a[0] - b[0], a[1] - b[1]) for a, b in zip(path, path[1:]))
