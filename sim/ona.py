"""Outside Network Area: RECEIVE the Writer's dump, TRANSLATE to GPS, CARRY to the command post, BRIEF the Executor."""
import math
import struct

from . import uplink
from .beacon import EVENT_NAME, EXIT, FIRE, NO_BEACON, VICTIM, Beacon, PACKET_LEN
from .beacon_graph import BeaconGraph
from .frame import correct_points, estimate_gyro_drift, to_enu, enu_to_gps
from .writer import DUMP_MAGIC


class ONA:
    def __init__(self, cfg, rng):
        self.cfg, self.rng = cfg, rng
        a = cfg.anchor
        self.lat0, self.lon0, self.psi = a.lat0, a.lon0, a.psi_deg
        self.log = []
        self.role = None
        self.beacons = {}          # id -> Beacon (decoded)
        self.meta = {}             # id -> odo, parent_path, neighbours, event_xy
        self.closure = (0.0, 0.0)
        self.path = []             # [(x, y, odo)] believed
        self.rx = {"entries": 0, "crc_rejected_copies": 0, "lost": 0, "bytes": 0}
        self.corrected = {}        # id -> {"x","y","lat","lon","sigma","event_xy"}
        self.path_gps = []
        self.graph = None

    def say(self, t, text):
        self.log.append((t, f"[ONA:{self.role}] {text}"))

    # ---------- ROLE 1: RECEIVE ------------------------------------------
    def receive(self, t, data: bytes):
        self.role = "RECEIVE"
        self.rx["bytes"] = len(data)
        if data[:4] != DUMP_MAGIC:
            raise ValueError("not a Living Map dump")
        ver, n, cx, cy = struct.unpack_from("<BHii", data, 4)
        self.closure = (cx / 100, cy / 100)
        off = 4 + struct.calcsize("<BHii")
        for _ in range(n):
            copies = [data[off:off + PACKET_LEN], data[off + PACKET_LEN:off + 2 * PACKET_LEN]]
            off += 2 * PACKET_LEN
            odo, ppath, nn = struct.unpack_from("<IHB", data, off)
            off += struct.calcsize("<IHB")
            neigh = list(data[off:off + nn])
            off += nn
            has_ev = data[off]
            off += 1
            ev = None
            if has_ev:
                ex, ey = struct.unpack_from("<hh", data, off)
                off += 4
                ev = (ex / 100, ey / 100)
            b = None
            for c in copies:
                try:
                    b = Beacon.decode(c)
                    break
                except ValueError:
                    self.rx["crc_rejected_copies"] += 1
            self.rx["entries"] += 1
            if b is None:
                self.rx["lost"] += 1
                continue
            self.beacons[b.beacon_id] = b
            self.meta[b.beacon_id] = {"odo": odo / 100, "parent_path": ppath / 100, "neighbours": neigh,
                                      "event_xy": ev}
        (npts,) = struct.unpack_from("<H", data, off)
        off += 2
        for _ in range(npts):
            x, y, odo = struct.unpack_from("<hhI", data, off)
            off += struct.calcsize("<hhI")
            self.path.append((x / 100, y / 100, odo / 100))
        self.say(t, f"dump received: {len(data)} B, {len(self.beacons)} beacons, "
                    f"{self.rx['crc_rejected_copies']} corrupted copies rejected by CRC, {self.rx['lost']} lost")

    # ---------- ROLE 2: TRANSLATE ----------------------------------------
    def translate(self, t):
        self.role = "TRANSLATE"
        total = self.path[-1][2] if self.path else 1.0
        self.beta, err_before, err_after = estimate_gyro_drift(self.path)
        ids = sorted(self.beacons)
        pts = [(self.beacons[i].x_cm / 100, self.beacons[i].y_cm / 100) for i in ids]
        odo = [self.meta[i]["odo"] for i in ids]
        fixed, path_fixed = correct_points(self.path, self.beta, pts, odo)
        evs = [self.meta[i]["event_xy"] or (0.0, 0.0) for i in ids]
        evs_fixed, _ = correct_points(self.path, self.beta, evs, odo)
        for i, (x, y), ev, d in zip(ids, fixed, evs_fixed, odo):
            lat, lon = enu_to_gps(*to_enu(x, y, self.psi), self.lat0, self.lon0)
            sigma = 0.10 + 0.02 * min(d, total - d) * (err_after / max(err_before, 1e-6) + 0.1)
            self.corrected[i] = {"x": x, "y": y, "lat": lat, "lon": lon, "sigma": sigma,
                                 "event_xy": ev if self.meta[i]["event_xy"] else None}
        self.path_enu = [to_enu(x, y, self.psi) for x, y in path_fixed]
        self.path_gps = [enu_to_gps(e, n, self.lat0, self.lon0) for e, n in self.path_enu]
        self.say(t, f"loop closure: exit error {err_before:.2f} m -> gyro drift {math.degrees(self.beta) * 100:.2f} deg/100 m "
                    f"removed (residual {err_after:.2f} m); {len(ids)} beacons -> GPS "
                    f"(anchor {self.lat0:.5f}, {self.lon0:.5f}, psi {self.psi:.0f} deg)")
        self.graph = self._build_graph(t)

    def _build_graph(self, t):
        nodes, edges = {}, {}
        for i, b in self.beacons.items():
            c = self.corrected[i]
            nodes[i] = {"x": c["x"], "y": c["y"], "type": b.event_type, "conf": b.confidence(t),
                        "event_xy": c["event_xy"], "t": b.t_written, "severity": b.severity,
                        "ttl_min": b.ttl_min, "next_id": b.next_id}
        for i, b in self.beacons.items():
            if b.next_id != NO_BEACON and b.next_id in nodes:
                edges[(i, b.next_id)] = max(self.meta[i]["parent_path"], 0.1)
            for j in self.meta[i]["neighbours"]:
                if j in nodes:
                    edges[(i, j)] = math.hypot(nodes[i]["x"] - nodes[j]["x"], nodes[i]["y"] - nodes[j]["y"])
        return BeaconGraph(nodes, edges)

    # ---------- ROLE 3: CARRY --------------------------------------------
    def uplink_frames(self, link, t, writer_status):
        self.role = "CARRY"
        recs = []
        for i in sorted(self.beacons):
            b, c = self.beacons[i], self.corrected[i]
            ev = c["event_xy"]
            lat, lon = (enu_to_gps(*to_enu(*ev, self.psi), self.lat0, self.lon0) if ev else (c["lat"], c["lon"]))
            recs.append({"id": i, "type": b.event_type, "lat": lat, "lon": lon, "t": b.t_written,
                         "severity": b.severity, "conf": b.confidence(t), "sigma": c["sigma"],
                         "next_id": b.next_id})
        frames = [link.frame(uplink.STATUS, uplink.status_payload(*writer_status))]
        frames += [link.frame(uplink.BEACONS, p) for p in uplink.beacon_payloads(recs)]
        step = max(1, len(self.path_enu) // 48)
        frames += [link.frame(uplink.PATH, p) for p in uplink.path_payloads(self.path_enu[::step] + self.path_enu[-1:])]
        link.push(frames)
        self.say(t, f"{len(frames)} frames queued for the command post ({sum(map(len, frames))} B)")
        return frames

    # ---------- ROLE 4: BRIEF --------------------------------------------
    def brief(self, t, target_id, policy="commander"):
        self.role = "BRIEF"
        g = self.graph
        for i, n in g.nodes.items():
            n["conf"] = self.beacons[i].confidence(t)
        start = next(i for i, b in self.beacons.items() if b.event_type == EXIT)
        route, cost = g.route(start, target_id)
        avoid = [i for i in g.nodes if g.risky(i) and i != target_id]
        mission = {
            "mission_id": 1, "issued_at": int(t), "policy": policy,
            "target_beacon": target_id,
            "target": f"{EVENT_NAME[g.nodes[target_id]['type']]}@B{target_id}",
            "route": route, "route_cost_m": round(cost, 1), "avoid": avoid,
            "beacon_table": {i: dict(n) for i, n in g.nodes.items()},
            "edges": [[a, b, round(w, 2)] for (a, b), w in g.edges().items()],
        }
        self.say(t, f"Executor briefed: target B{target_id}, route {route} (cost {cost:.1f}), keep clearance from fire beacons {avoid}")
        return mission
