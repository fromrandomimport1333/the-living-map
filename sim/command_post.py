"""Command post: decodes uplink frames, keeps the live map, takes the commander's decision."""
import json
import math
from pathlib import Path

from . import uplink
from .beacon import EVENT_NAME, FIRE, TAU_S, VICTIM, NO_BEACON
from .frame import enu_to_gps

STATES = ["EXPLORE", "RETURN", "DONE", "DEAD"]


def _metres(a, b):
    lat = math.radians((a[0] + b[0]) / 2)
    return math.hypot((a[0] - b[0]) * 111320, (a[1] - b[1]) * 111320 * math.cos(lat))


class CommandPost:
    def __init__(self, cfg, out_dir):
        self.cfg = cfg
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.beacons, self.writer_path, self.exec_path = {}, [], []
        self.status, self.report, self.decision = None, None, None
        self.log, self.frames_rx = [], 0
        self.seen_seq = set()

    def say(self, t, text):
        self.log.append((t, f"[CommandPost] {text}"))

    def receive(self, t, frame):
        kind, seq, payload = frame[0], frame[1], frame[2:]
        if seq in self.seen_seq:
            return
        self.seen_seq.add(seq)
        self.frames_rx += 1
        if kind == uplink.STATUS:
            self.status = uplink.decode_status(payload)
            self.say(t, f"status: Writer {STATES[self.status['writer_state']]}, coverage {self.status['coverage']:.0%}")
        elif kind == uplink.BEACONS:
            for r in uplink.decode_beacons(payload):
                r["rx_t"] = t
                self.beacons[r["id"]] = r
        elif kind == uplink.PATH:
            self.writer_path += [enu_to_gps(e, n, self.cfg.anchor.lat0, self.cfg.anchor.lon0)
                                 for e, n in uplink.decode_path(payload)]
        elif kind == uplink.EXEC_PATH:
            self.exec_path += [enu_to_gps(e, n, self.cfg.anchor.lat0, self.cfg.anchor.lon0)
                               for e, n in uplink.decode_path(payload)]
        elif kind == uplink.REPORT:
            self.report = uplink.decode_report(payload)
            self.say(t, f"MISSION REPORT: target B{self.report['target']} "
                        f"{'REACHED' if self.report['success'] else 'NOT reached'} at "
                        f"{self.report['lat']:.6f}, {self.report['lon']:.6f}")

    # ---------- commander decision ---------------------------------------
    def conf_now(self, r, t):
        if r["type"] not in (FIRE, VICTIM):
            return 1.0
        age = t - r["t"]
        if age > {FIRE: 30, VICTIM: 90}[r["type"]] * 60:
            return 0.0
        return r["severity"] / 255 * math.exp(-age / TAU_S[r["type"]])

    def chain(self, bid):
        out, seen = [], set()
        while bid in self.beacons and bid not in seen:
            seen.add(bid)
            out.append(bid)
            bid = self.beacons[bid]["next_id"]
        return out

    def decide(self, t):
        fires = [(r["lat"], r["lon"]) for r in self.beacons.values()
                 if r["type"] == FIRE and self.conf_now(r, t) >= 0.15]
        best, lines = None, []
        for r in self.beacons.values():
            if r["type"] != VICTIM:
                continue
            ch = self.chain(r["id"])
            cost = sum(_metres((self.beacons[a]["lat"], self.beacons[a]["lon"]),
                               (self.beacons[b]["lat"], self.beacons[b]["lon"])) for a, b in zip(ch, ch[1:]))
            risky = any(_metres((self.beacons[b]["lat"], self.beacons[b]["lon"]), f) < 3.0 for b in ch for f in fires)
            conf = self.conf_now(r, t)
            score = conf / (1 + cost / 30) * (0.3 if risky else 1.0)
            lines.append(f"B{r['id']}: conf {conf:.2f}, ~{cost:.0f} m, {'near fire' if risky else 'clear'} -> {score:.2f}")
            if best is None or score > best[0]:
                best = (score, r["id"])
        for l in lines:
            self.say(t, "candidate " + l)
        if best is None:
            return None
        self.decision = best[1]
        self.say(t, f"COMMANDER DECISION: rescue victim at B{best[1]}")
        return best[1]

    # ---------- live map -------------------------------------------------
    def snapshot(self, t, phase):
        return {
            "t": round(t, 1), "phase": phase,
            "anchor": [self.cfg.anchor.lat0, self.cfg.anchor.lon0],
            "beacons": [dict(r, name=EVENT_NAME[r["type"]], conf_now=round(self.conf_now(r, t), 2))
                        for r in sorted(self.beacons.values(), key=lambda r: r["id"])],
            "writer_path": self.writer_path, "exec_path": self.exec_path,
            "status": self.status, "decision": self.decision, "report": self.report,
            "frames_rx": self.frames_rx,
        }

    def write_live_map(self, t, phase):
        snap = self.snapshot(t, phase)
        (self.out / "live_map.json").write_text(json.dumps(snap, indent=1))
        html = LIVE_MAP_HTML.replace("__DATA__", json.dumps(snap))
        (self.out / "live_map.html").write_text(html)


LIVE_MAP_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="3">
<title>Living Map — Command Post</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
 body{margin:0;font:14px system-ui,sans-serif;background:#0f172a;color:#e2e8f0}
 #map{position:absolute;top:0;bottom:0;left:0;right:340px}
 #side{position:absolute;top:0;bottom:0;right:0;width:340px;overflow:auto;padding:14px;box-sizing:border-box}
 h1{font-size:17px;margin:0 0 6px} .k{color:#94a3b8} table{width:100%;border-collapse:collapse;font-size:12px}
 td,th{padding:3px 4px;border-bottom:1px solid #1e293b;text-align:left} .pill{padding:1px 6px;border-radius:8px;color:#fff}
 @media (max-width:700px){#map{right:0;bottom:45%}#side{top:55%;width:100%}}
</style></head><body>
<div id="map"></div><div id="side"></div>
<script>
const D = __DATA__;
const COL = {waypoint:'#3b82f6', exit:'#a855f7', fire:'#ef4444', victim:'#22c55e', blocked:'#a16207'};
const map = L.map('map', {maxZoom: 23}).setView(D.anchor, 20);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxNativeZoom: 19, maxZoom: 23,
  attribution: '&copy; OpenStreetMap'}).addTo(map);
if (D.writer_path.length) L.polyline(D.writer_path, {color:'#f97316', weight:2, opacity:.8}).addTo(map);
if (D.exec_path.length) L.polyline(D.exec_path, {color:'#16a34a', weight:3}).addTo(map);
L.marker(D.anchor).addTo(map).bindPopup('Entrance / ONA anchor');
for (const b of D.beacons) {
  const c = COL[b.name];
  L.circle([b.lat, b.lon], {radius: Math.max(b.sigma, .2), color: c, weight: 1, fillOpacity: .12}).addTo(map);
  L.circleMarker([b.lat, b.lon], {radius: b.name==='waypoint' ? 4 : 8, color: c, fillColor: c,
     fillOpacity: b.name==='waypoint' ? .6 : Math.max(.15, b.conf_now)}).addTo(map)
   .bindTooltip(`B${b.id} ${b.name}` + (['fire','victim','blocked'].includes(b.name) ? ` conf ${b.conf_now}` : ''));
}
if (D.report) L.circleMarker([D.report.lat, D.report.lon], {radius: 14, color: '#facc15', weight: 3, fill: false})
   .addTo(map).bindTooltip('Executor reached target');
if (D.beacons.length) map.fitBounds(L.latLngBounds(D.beacons.map(b => [b.lat, b.lon])).pad(0.15));
const ev = D.beacons.filter(b => b.name !== 'waypoint');
document.getElementById('side').innerHTML = `
 <h1>Command Post — Live Map</h1>
 <div class="k">t = ${D.t}s · phase <b>${D.phase}</b> · ${D.frames_rx} frames received</div>
 ${D.status ? `<p>Writer coverage <b>${Math.round(D.status.coverage*100)}%</b>, battery ${Math.round(D.status.battery*100)}%, ${D.status.beacons} beacons</p>` : '<p>Waiting for the ONA uplink…</p>'}
 ${D.decision !== null ? `<p>Decision: rescue <b>B${D.decision}</b></p>` : ''}
 ${D.report ? `<p>Executor: target B${D.report.target} <b>${D.report.success ? 'REACHED' : 'NOT reached'}</b>, re-routes ${D.report.rerouted}, dead beacons ${D.report.dead_beacons}</p>` : ''}
 <table><tr><th>id</th><th>event</th><th>age</th><th>conf</th><th>±m</th></tr>
 ${ev.map(b => `<tr><td>B${b.id}</td><td><span class="pill" style="background:${COL[b.name]}">${b.name}</span></td>
   <td>${Math.round((D.t-b.t)/60)} min</td><td>${b.conf_now}</td><td>${b.sigma}</td></tr>`).join('')}
 </table>`;
</script></body></html>
"""
