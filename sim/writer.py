"""Writer robot: explores a GPS-denied building, detects events, drops beacons, brings data out."""
import math
import struct

from .beacon import (BLOCKED, EXIT, FIRE, NO_BEACON, TTL_MIN, VICTIM, WAYPOINT, EVENT_NAME,
                     Beacon, heading_byte)
from .robot import OCC, Robot

EXPLORE, RETURN, DONE, DEAD = "EXPLORE", "RETURN", "DONE", "DEAD"
DUMP_MAGIC = b"LMAP"


class Writer(Robot):
    def __init__(self, world, cfg, rng, die_at_step=None):
        super().__init__("Writer", world, cfg, cfg.writer, rng)
        w = cfg.writer
        self.battery = float(w.battery_steps)
        self.stock = w.beacon_stock
        self.state = EXPLORE
        self.entries = []              # log entries (what will be dumped to the ONA)
        self.known_events = []         # (kind, x, y) believed
        self.beacon_exit_dist = {}     # beacon id -> path distance to exit (m), for next_id choice
        self.truth = {}                # beacon id -> true cell (evaluation only, never dumped)
        self.odo_at_last_drop = 0.0
        self.last_beacon = None
        self.steps = 0
        self.die_at_step = die_at_step
        self.path_log = []             # (x, y, odo) believed trajectory
        self.closure = None
        self.sense()
        self.path_log.append((0.0, 0.0, 0.0))
        self.drop(EXIT, 255)

    # ---------- beacons --------------------------------------------------
    def heard(self):
        out = []
        for pkt, rssi, d in self.world.hear_beacons(self.cell, self.cfg.beacon.radio_range_m, self.rng):
            out.append((Beacon.decode(pkt), rssi, d))
        return out

    def drop(self, kind, severity, event_xy=None):
        if self.stock <= 0:
            self.say(f"no beacons left, {EVENT_NAME[kind]} kept in memory only")
            self.known_events.append((kind, *(event_xy or self.believed())))
            return None
        bid = len(self.entries)
        x, y = self.believed()
        heard = [b for b, _, d in self.heard() if b.beacon_id in self.beacon_exit_dist]
        parent, parent_path = NO_BEACON, 0.0
        if heard:
            p = min(heard, key=lambda b: self.beacon_exit_dist[b.beacon_id] + math.hypot(b.x_cm / 100 - x, b.y_cm / 100 - y))
            parent = p.beacon_id
            parent_path = math.hypot(p.x_cm / 100 - x, p.y_cm / 100 - y)
        elif self.last_beacon is not None:
            parent = self.last_beacon.beacon_id
            parent_path = self.odo - self.odo_at_last_drop
        if parent != NO_BEACON:
            pb = next(e["beacon"] for e in self.entries if e["beacon"].beacon_id == parent)
            heading = heading_byte(pb.x_cm / 100 - x, pb.y_cm / 100 - y)
            self.beacon_exit_dist[bid] = self.beacon_exit_dist[parent] + parent_path
        else:
            heading = 0
            self.beacon_exit_dist[bid] = 0.0
        b = Beacon(bid, kind, int(round(x * 100)), int(round(y * 100)), int(self.t),
                   int(severity), parent, heading, TTL_MIN[kind])
        pkt = b.encode()
        self.world.place_beacon(bid, self.cell, pkt)
        self.truth[bid] = self.cell
        self.entries.append({"beacon": b, "packet": pkt, "odo": self.odo, "parent_path": parent_path,
                             "neighbours": sorted({h.beacon_id for h in heard}),
                             "event_xy": event_xy})
        self.stock -= 1
        self.odo_at_last_drop = self.odo
        self.last_beacon = b
        if event_xy is not None:
            self.known_events.append((kind, *event_xy))
        what = EVENT_NAME[kind] if kind != WAYPOINT else "waypoint"
        self.say(f"beacon B{bid} dropped ({what}, sev {int(severity)}) at x={x:.1f} y={y:.1f} m")
        return b

    def _new_event(self, kind, xy):
        return all(not (k == kind and math.hypot(x - xy[0], y - xy[1]) < 2.0)
                   for k, x, y in self.known_events)

    def detect(self, seen, temp):
        w = self.cfg.writer
        bx, by = self.believed()
        tx, ty = self.world.to_private(self.cell)
        ox, oy = bx - tx, by - ty                      # robot sees events relative to itself
        found = []
        if temp > w.fire_temp_threshold_c:
            hot = [f for f in self.world.fire_cells if f in seen]
            if hot:
                c = (sum(r for r, _ in hot) / len(hot), sum(c for _, c in hot) / len(hot))
                px, py = self.world.to_private(c)
                found.append((FIRE, min(255, 2 * (temp - 25)), (px + ox, py + oy)))
        for p, ch in seen.items():
            d = self.world.dist_m(self.cell, p)
            if ch == "V" and d <= w.victim_range_m:
                px, py = self.world.to_private(p)
                found.append((VICTIM, 255 * (1 - 0.3 * d / w.victim_range_m), (px + ox, py + oy)))
            elif ch == "X":
                px, py = self.world.to_private(p)
                found.append((BLOCKED, 255, (px + ox, py + oy)))
        for kind, sev, xy in found:
            if self._new_event(kind, xy):
                self.say(f"{EVENT_NAME[kind].upper()} detected near x={xy[0]:.1f} y={xy[1]:.1f} m"
                         + (f" ({temp:.0f} C)" if kind == FIRE else ""))
                self.drop(kind, sev, xy)

    def maybe_breadcrumb(self):
        w = self.cfg.writer
        if self.stock <= w.event_reserve:
            return
        near = [d for _, _, d in self.heard() if d < 1.5]
        r, c = self.cell
        doorway = ((self.known_at((r, c - 1)) == OCC and self.known_at((r, c + 1)) == OCC) or
                   (self.known_at((r - 1, c)) == OCC and self.known_at((r + 1, c)) == OCC))
        if near:
            return
        if doorway:
            self.say("doorway reached")
            self.drop(WAYPOINT, 0)
        elif self.odo - self.odo_at_last_drop >= w.breadcrumb_every_m:
            self.drop(WAYPOINT, 0)

    # ---------- main loop ------------------------------------------------
    def step(self):
        if self.state in (DONE, DEAD):
            return
        self.steps += 1
        if self.die_at_step is not None and self.steps >= self.die_at_step:
            self.state = DEAD
            self.say("!!! power failure inside the building — robot lost")
            return
        w = self.cfg.writer
        if self.state == EXPLORE:
            home = self.path_home()
            cost = (len(home) - 1) if home else 0
            if self.battery < w.return_margin * cost + w.return_reserve:
                self.state = RETURN
                self.say(f"battery {self.battery / w.battery_steps:.0%} — returning to exit to save the data")
                self.plan = []
            else:
                if not self.plan or not self.is_frontier(self.plan[-1]):
                    self.plan = self.frontier_path() or []
                    if not self.plan:
                        self.state = RETURN
                        self.say("no frontier left — exploration complete, returning")
        if self.state == RETURN:
            if self.cell == self.world.entrance:
                self.finish()
                return
            if not self.plan or self.plan[-1] != self.world.entrance:
                self.plan = self.path_home() or []
        if not self.step_along():
            self.plan = []
            return
        self.battery -= math.hypot(self.trail[-1][0] - self.trail[-2][0], self.trail[-1][1] - self.trail[-2][1])
        self.path_log.append((*self.believed(), self.odo))
        seen, temp = self.sense()
        if self.state == EXPLORE:
            self.detect(seen, temp)
            self.maybe_breadcrumb()
        if self.state == RETURN and self.cell == self.world.entrance:
            self.finish()

    def finish(self):
        self.state = DONE
        self.closure = tuple(self.believed())          # true position is (0, 0)
        self.say(f"back at the exit; loop-closure error {math.hypot(*self.closure):.2f} m — crossing the radio line")

    # ---------- data dump (Writer -> ONA, short range at the exit) -------
    def dump(self) -> bytes:
        """Binary log: header, entries (each beacon packet stored twice), believed trajectory."""
        cx, cy = self.closure
        out = bytearray(DUMP_MAGIC + struct.pack("<BHii", 1, len(self.entries), int(cx * 100), int(cy * 100)))
        for e in self.entries:
            out += e["packet"] + e["packet"]
            out += struct.pack("<IHB", int(e["odo"] * 100), int(e["parent_path"] * 100), len(e["neighbours"]))
            out += bytes(e["neighbours"])
            if e["event_xy"] is None:
                out += b"\x00"
            else:
                out += b"\x01" + struct.pack("<hh", int(e["event_xy"][0] * 100), int(e["event_xy"][1] * 100))
        out += struct.pack("<H", len(self.path_log))
        for x, y, odo in self.path_log:
            out += struct.pack("<hhI", int(x * 100), int(y * 100), int(odo * 100))
        return bytes(out)
