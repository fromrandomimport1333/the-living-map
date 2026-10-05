"""Runs the whole chain: Writer -> beacons -> ONA (receive/translate/carry/brief) -> command post -> Executor."""
import json
import math
import random
from pathlib import Path

from . import uplink
from .beacon import EXIT, PACKET_LEN
from .command_post import CommandPost, STATES
from .executor import DONE as EX_DONE, Executor
from .frame import to_enu
from .geometry import astar
from .link import LoRaLink
from .ona import ONA
from .robot import OCC
from .world import World
from .writer import DEAD, DONE, Writer

FAILS = ("none", "beacon", "block", "writer", "link", "battery")


class Scenario:
    def __init__(self, cfg, fail="none", out_dir="out"):
        assert fail in FAILS, fail
        self.cfg, self.fail = cfg, fail
        self.rng = random.Random(cfg.seed)
        self.world = World(cfg)
        if fail == "battery":
            cfg.writer["battery_steps"] = 110
        self.writer = Writer(self.world, cfg, self.rng, die_at_step=70 if fail == "writer" else None)
        lcfg = dict(cfg.link)
        if fail == "link":
            lcfg["loss"] = 0.45
        from .config import Cfg
        self.link = LoRaLink(Cfg(lcfg), self.rng)
        self.ona = ONA(cfg, self.rng)
        self.cp = CommandPost(cfg, out_dir)
        self.out = Path(out_dir)
        self.executor = None
        self.t = 0.0
        self.phase = "WRITER"
        self.messages = []
        self._cursor = {}
        self.mission = None
        self.results = {"scenario": fail}
        self.link_events = []

    # ---------- message plumbing -----------------------------------------
    def _drain(self):
        batch = []
        for src in (self.writer, self.ona, self.cp, self.executor):
            if src is None:
                continue
            k = id(src)
            new = src.log[self._cursor.get(k, 0):]
            self._cursor[k] = len(src.log)
            batch.extend(new)
        self.messages.extend(sorted(batch, key=lambda m: m[0]))

    def coverage(self):
        free = self.world.free_cells()
        return sum(1 for p in free if self.writer.known_at(p) == 0) / len(free)

    def _corrupt(self, data):
        """Short-range dump channel: a few packet copies get a flipped byte (CRC must catch it)."""
        data = bytearray(data)
        off = 4 + 11
        for e in self.writer.entries:
            for copy in range(2):
                if self.rng.random() < self.cfg.beacon.bit_error_packets:
                    data[off + copy * PACKET_LEN + self.rng.randrange(PACKET_LEN)] ^= 0x5A
            off += 2 * PACKET_LEN + 7 + len(e["neighbours"]) + (5 if e["event_xy"] else 1)
        return bytes(data)

    def _carry(self, label):
        while self.link.queue:
            f, dt, line = self.link.send_next()
            self.t += dt
            if f is not None:
                self.cp.receive(self.t, f)
            self.link_events.append((self.t, line))
            self.ona.say(self.t, line)
            self.cp.write_live_map(self.t, label)
            yield

    # ---------- the story ------------------------------------------------
    def run(self):
        w = self.writer
        while w.state not in (DONE, DEAD):
            w.step()
            self.t = w.t
            self._drain()
            yield
        self.results["writer"] = {
            "state": w.state, "steps": w.steps, "time_s": round(w.t, 1), "distance_m": round(w.odo, 1),
            "coverage": round(self.coverage(), 3), "battery_left": round(w.battery / self.cfg.writer.battery_steps, 3),
            "beacons_dropped": len(w.entries), "beacons_left": w.stock,
            "events_found": [e["beacon"].name for e in w.entries if e["event_xy"] is not None],
            "events_total": len(self.world.events),
        }
        if w.state == DEAD:
            yield from self._writer_lost()
        else:
            yield from self._normal()
        self._drain()
        self.results["link"] = vars(self.link.stats)
        self.results["link"]["airtime_s"] = round(self.link.stats.airtime_s, 2)
        self.results["total_time_s"] = round(self.t, 1)
        self.phase = "DONE"
        self.cp.write_live_map(self.t, "DONE")
        (self.out / f"results_{self.fail}.json").write_text(json.dumps(self.results, indent=1))
        yield

    def _normal(self):
        w, ona, cp = self.writer, self.ona, self.cp
        self.phase = "RECEIVE"
        self.t += self.cfg.ona.dump_time_s
        ona.receive(self.t, self._corrupt(w.dump()))
        self._drain()
        for _ in range(15):
            yield
        self.phase = "TRANSLATE"
        self.t += 2
        ona.translate(self.t)
        self._evaluate_translation()
        self._drain()
        for _ in range(15):
            yield
        self.phase = "CARRY"
        ona.uplink_frames(self.link, self.t, (STATES.index(w.state), self.coverage(),
                                              w.battery / self.cfg.writer.battery_steps, len(w.entries)))
        self._drain()
        yield from self._carry("CARRY")
        self.phase = "DECISION"
        self.t += self.cfg.ona.decision_time_s
        target = cp.decide(self.t)
        self._drain()
        for _ in range(15):
            yield
        self.link.push([self.link.frame(uplink.DECISION, uplink.decision_payload(target, 0))])
        f, dt, line = self.link.send_next()          # downlink: command post -> ONA
        while f is None:
            self.t += dt
            f, dt, line = self.link.send_next()
        self.t += dt
        ona.say(self.t, f"decision received from command post: target B{f[2]}")
        self.phase = "BRIEF"
        self.t += self.cfg.ona.brief_time_s
        self.mission = ona.brief(self.t, f[2])
        (self.out / "mission_brief.json").write_text(json.dumps(self.mission, indent=1))
        self._inject_failure()
        self._drain()
        for _ in range(15):
            yield
        self.executor = Executor(self.world, self.cfg, self.rng, self.t)
        self.executor.load_brief(self.mission)
        yield from self._run_executor()

    def _writer_lost(self):
        self.phase = "WAIT"
        self.ona.role = "RECEIVE"
        self.ona.say(self.t, "Writer silent... waiting at the radio line")
        self._drain()
        for _ in range(20):
            yield
        self.t = max(self.t, self.cfg.ona.writer_timeout_s)
        self.ona.say(self.t, f"Writer declared LOST after {self.t / 60:.0f} min — its beacons are the only memory left")
        self.ona.role = "CARRY"
        self.link.push([self.link.frame(uplink.STATUS, uplink.status_payload(STATES.index(DEAD), 0, 0, 0))])
        self._drain()
        yield from self._carry("WRITER LOST")
        self.phase = "BRIEF"
        self.ona.role = "BRIEF"
        self.ona.say(self.t, "default policy: Executor sent in RECOVERY mode (no map, read the beacons)")
        self._drain()
        self.executor = Executor(self.world, self.cfg, self.rng, self.t)
        self.executor.start_recovery()
        yield from self._run_executor()

    def _inject_failure(self):
        route = self.mission["route"]
        if self.fail == "beacon" and len(route) > 2:
            bid = route[len(route) // 2]
            self.world.beacons[bid].alive = False
            self.ona.say(self.t, f"[FAILURE INJECTED] beacon B{bid} battery dies")
        if self.fail == "block":
            tgt = self.world.beacons[self.mission["target_beacon"]].cell
            path = astar(self.world.entrance, tgt, self.world.passable)
            doors = [p for p in path if (self.world.char((p[0], p[1] - 1)) == "#" and self.world.char((p[0], p[1] + 1)) == "#")
                     or (self.world.char((p[0] - 1, p[1])) == "#" and self.world.char((p[0] + 1, p[1])) == "#")]
            if doors:
                self.world.add_debris([doors[-1]])
                self.ona.say(self.t, f"[FAILURE INJECTED] ceiling collapse blocks the doorway at {doors[-1]} after the Writer passed")

    def _run_executor(self):
        ex = self.executor
        self.phase = "EXECUTOR"
        while ex.state != EX_DONE and ex.steps < 1500:
            ex.step()
            self.t = ex.t
            self._drain()
            yield
        self.results["executor"] = {
            "mode": "recovery" if self.fail == "writer" else "briefed",
            "steps": ex.steps, "time_s": round(ex.t - ex.t0, 1),
            "distance_m": round(ex.odo, 1), "success": ex.success, "reroutes": ex.reroutes,
            "dead_beacons": sorted(ex.dead), "blocked_edges": sorted(ex.blocked),
            "target": ex.target, "beacons_read": len(ex.read),
            "relocalisations": len(ex.snap_errors),
        }
        # report goes back through the ONA, never directly to the command post
        self.phase = "REPORT"
        self.ona.role = "CARRY"
        a = self.cfg.anchor
        tc = self.world.beacons[ex.target].cell if ex.target in self.world.beacons else ex.cell
        x, y = self.world.to_private(tc)
        from .frame import to_gps
        lat, lon = to_gps(x, y, a.lat0, a.lon0, a.psi_deg)
        frames = []
        if self.fail == "writer":
            recs = []
            for i, b in sorted(ex.read.items()):
                la, lo = to_gps(b.x_cm / 100, b.y_cm / 100, a.lat0, a.lon0, a.psi_deg)
                recs.append({"id": i, "type": b.event_type, "lat": la, "lon": lo, "t": b.t_written,
                             "severity": b.severity, "conf": b.confidence(self.t), "sigma": 1.0, "next_id": b.next_id})
            frames += [self.link.frame(uplink.BEACONS, p) for p in uplink.beacon_payloads(recs)]
            self.ona.say(self.t, f"Executor returned {len(recs)} beacons read inside — the Writer's memory is recovered")
        pts = [to_enu(*self.world.to_private(c), a.psi_deg) for c in ex.trail[::3] + ex.trail[-1:]]
        frames += [self.link.frame(uplink.EXEC_PATH, p) for p in uplink.path_payloads(pts)]
        frames.append(self.link.frame(uplink.REPORT, uplink.report_payload(
            ex.target if ex.target is not None else 255, ex.success, lat, lon, self.t, ex.reroutes, len(ex.dead))))
        self.link.push(frames)
        yield from self._carry("REPORT")

    def _evaluate_translation(self):
        w, ona = self.writer, self.ona
        raw, fixed = [], []
        for i, cell in w.truth.items():
            if i not in ona.beacons:
                continue
            tx, ty = self.world.to_private(cell)
            b = ona.beacons[i]
            raw.append(math.hypot(b.x_cm / 100 - tx, b.y_cm / 100 - ty))
            c = ona.corrected[i]
            fixed.append(math.hypot(c["x"] - tx, c["y"] - ty))
        self.results["translation"] = {
            "closure_error_m": round(math.hypot(*ona.closure), 3),
            "gyro_drift_deg_per_100m": round(math.degrees(ona.beta) * 100, 3),
            "beacon_error_raw_mean_m": round(sum(raw) / len(raw), 3), "beacon_error_raw_max_m": round(max(raw), 3),
            "beacon_error_corrected_mean_m": round(sum(fixed) / len(fixed), 3),
            "beacon_error_corrected_max_m": round(max(fixed), 3),
            "crc_rejected_copies": ona.rx["crc_rejected_copies"], "packets_lost": ona.rx["lost"],
            "dump_bytes": ona.rx["bytes"],
        }
