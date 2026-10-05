"""Common robot: local occupancy map from lidar, heat hazards, odometry with drift, planning."""
import math

from .geometry import astar, dijkstra, neighbours

UNKNOWN, FREE, OCC = -1, 0, 1


class Robot:
    def __init__(self, name, world, cfg, rcfg, rng, t0=0.0):
        self.name, self.world, self.cfg, self.rcfg, self.rng = name, world, cfg, rcfg, rng
        self.cell = world.entrance
        self.known = [[UNKNOWN] * world.W for _ in range(world.H)]
        self.seen_chars = {}
        self.hazard = set()
        self.t = self.t0 = t0
        self.odo = 0.0
        self.heading_err = 0.0
        self.drift = [0.0, 0.0]
        self.trail = [self.cell]
        self.log = []                         # (t, text) messages for the operator
        self.plan = []

    # ---------- bookkeeping ----------------------------------------------
    def say(self, text):
        self.log.append((self.t, f"[{self.name}] {text}"))

    def believed(self):
        x, y = self.world.to_private(self.cell)
        return x + self.drift[0], y + self.drift[1]

    def known_at(self, p):
        return self.known[p[0]][p[1]] if self.world.inside(p) else OCC

    def passable_known(self, p):
        return self.known_at(p) == FREE and p not in self.hazard

    def passable_optimistic(self, p):
        return self.world.inside(p) and self.known_at(p) != OCC and p not in self.hazard

    # ---------- sensing --------------------------------------------------
    def sense(self):
        seen = self.world.lidar(self.cell, self.rcfg.lidar_range_m, self.rcfg.lidar_rays)
        for p, ch in seen.items():
            self.known[p[0]][p[1]] = FREE if ch in ".E" else OCC
            self.seen_chars[p] = ch
        temp = self.world.temperature(self.cell)
        if temp > self.cfg.writer.fire_temp_threshold_c:
            # thermal camera localises the hot cells in view; keep 1 cell clearance around them
            for f in self.world.fire_cells:
                if f in seen and self.world.dist_m(self.cell, f) <= 2.5:
                    self.known[f[0]][f[1]] = OCC
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            q = (f[0] + dr, f[1] + dc)
                            if self.world.inside(q) and q != self.world.entrance:
                                self.hazard.add(q)
        return seen, temp

    # ---------- motion ---------------------------------------------------
    def move_to(self, nxt):
        x0, y0 = self.world.to_private(self.cell)
        x1, y1 = self.world.to_private(nxt)
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        dt = dist / self.cfg.speed_mps
        # odometry: gyro heading error grows with time, plus wheel noise
        self.heading_err += math.radians(self.cfg.writer.odo_gyro_bias_dps) * dt
        ca, sa = math.cos(self.heading_err), math.sin(self.heading_err)
        k = 1 + self.cfg.writer.odo_scale_error
        mx, my = k * (ca * dx - sa * dy), k * (sa * dx + ca * dy)
        noise = self.rcfg.odo_noise_m if "odo_noise_m" in self.rcfg else 0.01
        self.drift[0] += mx - dx + self.rng.gauss(0, noise)
        self.drift[1] += my - dy + self.rng.gauss(0, noise)
        self.cell = nxt
        self.odo += dist
        self.t += dt
        self.trail.append(nxt)
        return dist

    def snap_to(self, x, y):
        """Re-localise on a beacon: believed position := beacon's stored position."""
        tx, ty = self.world.to_private(self.cell)
        self.drift = [x - tx, y - ty]

    # ---------- planning -------------------------------------------------
    def frontier_path(self):
        dist = dijkstra(self.cell, self.passable_known)
        best = None
        for p, d in dist.items():
            if any(self.known_at((p[0] + a, p[1] + b)) == UNKNOWN and self.world.inside((p[0] + a, p[1] + b))
                   for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                if best is None or d < best[0]:
                    best = (d, p)
        if best is None:
            return None
        return astar(self.cell, best[1], self.passable_known)

    def is_frontier(self, p):
        return self.passable_known(p) and any(
            self.world.inside(q) and self.known_at(q) == UNKNOWN
            for q in ((p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)))

    def path_home(self):
        return astar(self.cell, self.world.entrance, self.passable_known)

    def step_along(self):
        """Move one cell along self.plan; returns False if the next cell became impassable."""
        if len(self.plan) < 2:
            return False
        nxt = self.plan[1]
        ok = any(n == nxt for n, _ in neighbours(self.cell, self.passable_optimistic))
        if not ok:
            return False
        self.move_to(nxt)
        self.plan = self.plan[1:]
        return True
