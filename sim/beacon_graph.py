"""Beacon navigation graph shared by the ONA (briefing) and the Executor (re-routing)."""
import heapq
import math

from .beacon import FIRE

FIRE_RADIUS_M = 2.5
FIRE_PENALTY_M = 40.0
FIRE_MIN_CONF = 0.15


class BeaconGraph:
    def __init__(self, nodes, edges):
        """nodes: {id: {"x", "y", "type", "conf", "event_xy"}}, edges: {(a, b): metres} (undirected)."""
        self.nodes = nodes
        self.adj = {i: {} for i in nodes}
        for (a, b), w in edges.items():
            if a in nodes and b in nodes and a != b:
                w = min(w, self.adj[a].get(b, math.inf))
                self.adj[a][b] = self.adj[b][a] = w

    def edges(self):
        return {(a, b): w for a in self.adj for b, w in self.adj[a].items() if a < b}

    def fire_points(self):
        out = []
        for n in self.nodes.values():
            if n["type"] == FIRE and n["conf"] >= FIRE_MIN_CONF:
                out.append(n.get("event_xy") or (n["x"], n["y"]))
        return out

    def risky(self, node_id, fires=None):
        n = self.nodes[node_id]
        fires = self.fire_points() if fires is None else fires
        return any(math.hypot(n["x"] - fx, n["y"] - fy) < FIRE_RADIUS_M for fx, fy in fires)

    def route(self, start, goal, removed_nodes=(), removed_edges=(), avoid_fire=True, start_xy=None):
        """Dijkstra. start may be a node id, or None with start_xy (virtual node linked to beacons < 6 m)."""
        fires = self.fire_points() if avoid_fire else []
        blocked = {frozenset(e) for e in removed_edges}
        def cost(a, b, w):
            pen = FIRE_PENALTY_M if fires and (self.risky(b, fires) and b != goal) else 0.0
            return w + pen
        dist, prev, pq = {}, {}, []
        if start is None:
            for i, n in self.nodes.items():
                if i in removed_nodes:
                    continue
                d = math.hypot(n["x"] - start_xy[0], n["y"] - start_xy[1])
                if d < 6.0:
                    c = cost(None, i, d * 1.2)
                    if c < dist.get(i, math.inf):
                        dist[i] = c
                        heapq.heappush(pq, (c, i))
        else:
            dist[start] = 0.0
            pq = [(0.0, start)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf):
                continue
            if u == goal:
                break
            for v, w in self.adj[u].items():
                if v in removed_nodes or frozenset((u, v)) in blocked:
                    continue
                nd = d + cost(u, v, w)
                if nd < dist.get(v, math.inf):
                    dist[v], prev[v] = nd, u
                    heapq.heappush(pq, (nd, v))
        if goal not in dist:
            return None, math.inf
        path, u = [goal], goal
        while u in prev:
            u = prev[u]
            path.append(u)
        return path[::-1], dist[goal]
