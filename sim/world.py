"""Ground truth: the building, hidden events, physically placed beacons, sensor models."""
import math
from dataclasses import dataclass, field

from .beacon import BLOCKED, FIRE, VICTIM
from .config import ROOT
from .geometry import line_of_sight

FREE = ".E"
EVENT_CHARS = {"F": FIRE, "V": VICTIM, "X": BLOCKED}


@dataclass
class PlacedBeacon:
    beacon_id: int
    cell: tuple
    packet: bytes
    alive: bool = True


@dataclass
class Event:
    event_id: int
    kind: int
    cells: list = field(default_factory=list)

    @property
    def center(self):
        return (sum(r for r, _ in self.cells) / len(self.cells),
                sum(c for _, c in self.cells) / len(self.cells))


class World:
    def __init__(self, cfg):
        self.cfg = cfg
        lines = (ROOT / cfg.map_file).read_text().split("\n")
        self.grid = [list(l) for l in lines if l]
        self.H, self.W = len(self.grid), len(self.grid[0])
        self.cell_m = cfg.cell_m
        self.entrance = next((r, c) for r in range(self.H) for c in range(self.W)
                             if self.grid[r][c] == "E")
        self.beacons = {}
        self.events = self._find_events()
        self.fire_cells = [p for e in self.events if e.kind == FIRE for p in e.cells]

    # ---------- geometry -------------------------------------------------
    def inside(self, p):
        return 0 <= p[0] < self.H and 0 <= p[1] < self.W

    def char(self, p):
        return self.grid[p[0]][p[1]]

    def passable(self, p):
        return self.inside(p) and self.char(p) in FREE

    def blocks_light(self, p):
        return not self.inside(p) or self.char(p) not in FREE

    def blocks_radio(self, p):
        return not self.inside(p) or self.char(p) in "#X"

    def to_private(self, cell):
        """Grid cell -> private frame metres (origin = entrance, +x into the building, +y left)."""
        er, ec = self.entrance
        return ((ec - cell[1]) * self.cell_m, (cell[0] - er) * self.cell_m)

    def to_cell(self, x, y):
        er, ec = self.entrance
        return (int(round(er + y / self.cell_m)), int(round(ec - x / self.cell_m)))

    def dist_m(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1]) * self.cell_m

    def free_cells(self):
        return [(r, c) for r in range(self.H) for c in range(self.W) if self.passable((r, c))]

    def add_debris(self, cells):
        for p in cells:
            self.grid[p[0]][p[1]] = "X"

    def _find_events(self):
        seen, events = set(), []
        for r in range(self.H):
            for c in range(self.W):
                ch = self.grid[r][c]
                if ch not in EVENT_CHARS or (r, c) in seen:
                    continue
                ev, stack = Event(len(events), EVENT_CHARS[ch]), [(r, c)]
                seen.add((r, c))
                while stack:
                    p = stack.pop()
                    ev.cells.append(p)
                    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        q = (p[0] + d[0], p[1] + d[1])
                        if self.inside(q) and q not in seen and self.char(q) == ch:
                            seen.add(q)
                            stack.append(q)
                events.append(ev)
        return events

    # ---------- sensors --------------------------------------------------
    def temperature(self, cell):
        t = 25.0
        for f in self.fire_cells:
            d = self.dist_m(cell, f)
            if d < 4.0 and line_of_sight(cell, f, lambda p: self.inside(p) and self.char(p) == "#"):
                t += 400.0 * math.exp(-d / 0.6)
        return t

    def lidar(self, cell, range_m, rays):
        """Cast rays; returns {cell: char} for every cell observed (free or the first hit)."""
        seen = {cell: self.char(cell)}
        rng = range_m / self.cell_m
        r0, c0 = cell[0] + 0.5, cell[1] + 0.5
        for k in range(rays):
            a = 2 * math.pi * k / rays
            dr, dc = math.sin(a), math.cos(a)
            s = 0.2
            while s <= rng:
                p = (int(math.floor(r0 + dr * s)), int(math.floor(c0 + dc * s)))
                if not self.inside(p):
                    break
                seen[p] = self.char(p)
                if self.char(p) not in FREE:
                    break
                s += 0.2
        return seen

    # ---------- beacons --------------------------------------------------
    def place_beacon(self, beacon_id, cell, packet):
        self.beacons[beacon_id] = PlacedBeacon(beacon_id, cell, packet)

    def hear_beacons(self, cell, range_m, rng):
        """Beacons whose broadcast reaches `cell`: list of (packet, rssi_dbm, true_distance_m)."""
        out = []
        for b in self.beacons.values():
            if not b.alive:
                continue
            d = self.dist_m(cell, b.cell)
            if d <= range_m and line_of_sight(cell, b.cell, self.blocks_radio):
                rssi = -45 - 20 * math.log10(max(d, 0.2)) + rng.gauss(0, 1.5)
                out.append((b.packet, rssi, d))
        return out
