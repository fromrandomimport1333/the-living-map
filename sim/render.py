"""Pygame view: building (truth + Writer knowledge), ONA roles, command-post live map, event log."""
import math

import pygame

from .beacon import BLOCKED, EXIT, FIRE, VICTIM, WAYPOINT, Beacon
from .frame import gps_to_enu
from .robot import UNKNOWN

W, H = 1280, 720
CELL = 15
BX, BY = 20, 70
BG = (15, 23, 42)
PANEL = (30, 41, 59)
TEXT = (226, 232, 240)
MUTED = (148, 163, 184)
WRITER = (249, 115, 22)
EXECUTOR = (34, 197, 94)
ONA_BLUE = (37, 99, 235)
RED = (220, 38, 38)
EVENT_COL = {WAYPOINT: (59, 130, 246), EXIT: (168, 85, 247), FIRE: (239, 68, 68),
             VICTIM: (34, 197, 94), BLOCKED: (180, 120, 40)}
ROLE_OF_PHASE = {"RECEIVE": 0, "WAIT": 0, "TRANSLATE": 1, "CARRY": 2, "DECISION": 2, "REPORT": 2, "BRIEF": 3}


def _font(size, bold=False):
    for name in ("dejavusans", "liberationsans", "arial"):
        f = pygame.font.SysFont(name, size, bold=bold)
        if f:
            return f
    return pygame.font.Font(None, size + 4)


class Renderer:
    def __init__(self, scenario, surface=None):
        pygame.font.init()
        self.s = scenario
        self.screen = surface or pygame.Surface((W, H))
        self.f12, self.f14, self.f16b, self.f20b = _font(12), _font(14), _font(15, True), _font(20, True)
        self.frame = 0

    # ---------- helpers --------------------------------------------------
    def text(self, txt, pos, font=None, col=TEXT):
        self.screen.blit((font or self.f14).render(txt, True, col), pos)

    def cell_px(self, cell):
        return BX + cell[1] * CELL + CELL // 2, BY + cell[0] * CELL + CELL // 2

    def dashed(self, a, b, col, dash=8, width=2):
        (x0, y0), (x1, y1) = a, b
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / dash))
        for i in range(0, n, 2):
            p = (x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n)
            q = (x0 + (x1 - x0) * min(i + 1, n) / n, y0 + (y1 - y0) * min(i + 1, n) / n)
            pygame.draw.line(self.screen, col, p, q, width)

    # ---------- main -----------------------------------------------------
    def draw(self):
        self.frame += 1
        self.screen.fill(BG)
        self.header()
        self.building()
        self.ona_panel()
        self.command_post()
        self.log_panel()
        return self.screen

    def header(self):
        s = self.s
        self.text("THE LIVING MAP — Spatial Memory for Emergency Robots", (20, 14), self.f20b)
        self.text("TSYP14 · IEEE RAS × IEEE AESS · Phase 1 simulation", (20, 40), self.f12, MUTED)
        tag = f"scenario: {s.fail}" if s.fail != "none" else "nominal scenario"
        self.text(f"PHASE  {s.phase}", (760, 14), self.f20b, (250, 204, 21))
        self.text(f"t = {s.t / 60:5.1f} min   ·   {tag}", (760, 40), self.f14, MUTED)

    def building(self):
        s, wd, wr = self.s, self.s.world, self.s.writer
        reveal_all = s.phase not in ("WRITER",)
        for r in range(wd.H):
            for c in range(wd.W):
                ch = wd.grid[r][c]
                known = wr.known[r][c] != UNKNOWN or (s.executor and s.executor.known[r][c] != UNKNOWN)
                rect = (BX + c * CELL, BY + r * CELL, CELL, CELL)
                if ch == "#":
                    col = (100, 116, 139) if known else (51, 65, 85)
                elif ch in ".E":
                    col = (203, 213, 225) if known else (39, 51, 71)
                elif ch == "F":
                    flick = 30 * math.sin(self.frame * 0.4 + r + c)
                    col = (239, int(80 + flick), 30) if known else (110, 50, 30)
                elif ch == "V":
                    col = (22, 163, 74) if known else (30, 80, 50)
                else:
                    col = (146, 96, 40) if known else (80, 60, 40)
                pygame.draw.rect(self.screen, col, rect)
                if (r, c) in wr.hazard and ch in ".E" and known:
                    pygame.draw.rect(self.screen, (252, 165, 165), rect)
        pygame.draw.rect(self.screen, (71, 85, 105), (BX - 2, BY - 2, wd.W * CELL + 4, wd.H * CELL + 4), 2)
        self.text("THE BUILDING — no GPS · no network", (BX, BY + wd.H * CELL + 6), self.f16b)
        self.text("grey = seen · dark = unknown · pink = heat no-go", (BX, BY + wd.H * CELL + 26),
                  self.f12, MUTED)
        # radio boundary
        x = BX + wd.W * CELL + 14
        self.dashed((x, BY - 10), (x, BY + wd.H * CELL + 10), RED)
        lbl = self.f12.render("RADIO STOPS HERE", True, RED)
        self.screen.blit(lbl, (x - lbl.get_width() // 2, BY - 24))
        # trails
        for robot, col in ((wr, WRITER), (s.executor, EXECUTOR)):
            if robot and len(robot.trail) > 1:
                pygame.draw.lines(self.screen, col, False, [self.cell_px(p) for p in robot.trail[-400:]], 2)
        # beacons
        for b in wd.beacons.values():
            kind = Beacon.decode(b.packet).event_type
            px = self.cell_px(b.cell)
            col = EVENT_COL[kind] if b.alive else (100, 100, 100)
            rad = 4 if kind == WAYPOINT else 6
            pygame.draw.circle(self.screen, (15, 23, 42), px, rad + 2)
            pygame.draw.circle(self.screen, col, px, rad)
            if not b.alive:
                pygame.draw.line(self.screen, RED, (px[0] - 6, px[1] - 6), (px[0] + 6, px[1] + 6), 2)
                pygame.draw.line(self.screen, RED, (px[0] - 6, px[1] + 6), (px[0] + 6, px[1] - 6), 2)
            if kind != WAYPOINT or b.beacon_id % 3 == 0:
                self.text(f"B{b.beacon_id}", (px[0] + 6, px[1] - 14), self.f12, (15, 23, 42) if kind else MUTED)
        # robots
        if wr.state != "DEAD" or self.frame % 10 < 5:
            pygame.draw.circle(self.screen, WRITER, self.cell_px(wr.cell), 7)
            pygame.draw.circle(self.screen, (0, 0, 0), self.cell_px(wr.cell), 7, 1)
        ex = s.executor
        if ex and ex.state != "WAIT":
            p = self.cell_px(ex.cell)
            if ex.state == "GOTO" and ex.idx < len(ex.route) and ex.route[ex.idx] in wd.beacons:
                self.dashed(p, self.cell_px(wd.beacons[ex.route[ex.idx]].cell), EXECUTOR, 5, 1)
            pygame.draw.rect(self.screen, EXECUTOR, (p[0] - 7, p[1] - 7, 14, 14))
            pygame.draw.rect(self.screen, (0, 0, 0), (p[0] - 7, p[1] - 7, 14, 14), 1)
        # legend
        lx, ly = BX + 365, BY + wd.H * CELL + 6
        for i, (name, col) in enumerate((("Writer", WRITER), ("Executor", EXECUTOR), ("waypoint", EVENT_COL[0]),
                                          ("fire", EVENT_COL[1]), ("victim", EVENT_COL[2]), ("blocked", EVENT_COL[3]))):
            pygame.draw.circle(self.screen, col, (lx + (i % 3) * 80, ly + 8 + (i // 3) * 18), 5)
            self.text(name, (lx + 9 + (i % 3) * 80, ly + 1 + (i // 3) * 18), self.f12, MUTED)

    def ona_panel(self):
        s = self.s
        x0, y0, w, h = 668, 70, 300, 300
        pygame.draw.rect(self.screen, (23, 37, 84), (x0, y0, w, h), border_radius=8)
        self.dashed((x0, y0), (x0 + w, y0), ONA_BLUE)
        self.dashed((x0, y0 + h), (x0 + w, y0 + h), ONA_BLUE)
        self.dashed((x0, y0), (x0, y0 + h), ONA_BLUE)
        self.dashed((x0 + w, y0), (x0 + w, y0 + h), ONA_BLUE)
        self.text("OUTSIDE NETWORK AREA", (x0 + 52, y0 + 8), self.f16b, (147, 197, 253))
        ona, active = s.ona, ROLE_OF_PHASE.get(s.phase)
        rx, tr = ona.rx, s.results.get("translation", {})
        st = s.link.stats
        info = [
            ("1 · RECEIVE", [f"{rx['bytes']} B dump" if rx["bytes"] else "waiting for Writer",
                             f"{len(ona.beacons)} beacons · {rx['crc_rejected_copies']} CRC rej." if rx["bytes"] else ""]),
            ("2 · TRANSLATE", [f"exit error {tr['closure_error_m']:.2f} m" if tr else "private -> GPS",
                               f"beacon err {tr['beacon_error_corrected_mean_m']:.2f} m" if tr else ""]),
            ("3 · CARRY", [f"LoRa {st.delivered}/{st.frames} frames", f"{st.attempts} tx, {st.lost_data + st.lost_ack} lost"]),
            ("4 · BRIEF", [f"target B{s.mission['target_beacon']}" if s.mission else
                           ("recovery mode" if s.executor and s.executor.state == "RECOVERY" else "mission plan"),
                           f"route {len(s.mission['route'])} beacons" if s.mission else ""]),
        ]
        for i, (title, lines) in enumerate(info):
            bx, by = x0 + 10 + (i % 2) * 145, y0 + 40 + (i // 2) * 125
            on = active == i
            pygame.draw.rect(self.screen, (59, 130, 246) if on else (30, 58, 138), (bx, by, 135, 112), border_radius=6)
            if on:
                pygame.draw.rect(self.screen, (250, 204, 21), (bx, by, 135, 112), 2, border_radius=6)
            self.text(title, (bx + 8, by + 8), self.f16b)
            for k, l in enumerate(lines):
                self.text(l, (bx + 8, by + 36 + k * 20), self.f12, TEXT)
        # executor waiting to be briefed
        ex = s.executor
        if ex is None or ex.state == "WAIT":
            px, py = x0 + 120, y0 + h + 30
            pygame.draw.rect(self.screen, EXECUTOR, (px, py, 22, 16), border_radius=3)
            self.text("Executor waiting to be briefed", (px + 30, py), self.f12, EXECUTOR)
        if s.phase == "BRIEF":
            self.dashed((x0 + 150, y0 + h), (x0 + 130, y0 + h + 28), EXECUTOR)

    def command_post(self):
        s, cp = self.s, self.s.cp
        x0, y0, w, h = 988, 70, 272, 300
        pygame.draw.rect(self.screen, PANEL, (x0, y0, w, h), border_radius=8)
        self.text("COMMAND POST — LIVE MAP", (x0 + 12, y0 + 8), self.f16b)
        self.text("far away · only reachable through the ONA", (x0 + 12, y0 + 28), self.f12, MUTED)
        mx, my, mw, mh = x0 + 10, y0 + 50, w - 20, 170
        pygame.draw.rect(self.screen, (15, 23, 42), (mx, my, mw, mh))
        a = s.cfg.anchor
        E0, E1, N0, N1 = -21.0, 3.0, -9.0, 9.0
        sc = min(mw / (E1 - E0), mh / (N1 - N0))
        def px(lat, lon):
            e, n = gps_to_enu(lat, lon, a.lat0, a.lon0)
            return mx + (e - E0) * sc, my + (N1 - n) * sc
        for path, col in ((cp.writer_path, WRITER), (cp.exec_path, EXECUTOR)):
            if len(path) > 1:
                pygame.draw.lines(self.screen, col, False, [px(*p) for p in path], 1 if col == WRITER else 2)
        for b in cp.beacons.values():
            p = px(b["lat"], b["lon"])
            col = EVENT_COL[b["type"]]
            if b["type"] != WAYPOINT:
                pygame.draw.circle(self.screen, col, p, max(3, int(b["sigma"] * sc)), 1)
            pygame.draw.circle(self.screen, col, p, 2 if b["type"] == WAYPOINT else 4)
        pygame.draw.polygon(self.screen, (250, 204, 21), [px(a.lat0, a.lon0), (px(a.lat0, a.lon0)[0] + 5, px(a.lat0, a.lon0)[1] + 9),
                                                         (px(a.lat0, a.lon0)[0] - 5, px(a.lat0, a.lon0)[1] + 9)])
        if cp.report:
            pygame.draw.circle(self.screen, (250, 204, 21), px(cp.report["lat"], cp.report["lon"]), 9, 2)
        self.text("N ↑   1 m grid = %.0f px" % sc, (mx + 4, my + mh - 16), self.f12, MUTED)
        y = my + mh + 8
        st = cp.status
        lines = [f"frames received: {cp.frames_rx}",
                 ("Writer: LOST — no data dump" if st['writer_state'] == 3 else f"Writer: {['EXPLORE', 'RETURN', 'DONE'][st['writer_state']]}, coverage {st['coverage']:.0%}") if st
                 else "waiting for uplink…",
                 f"decision: rescue B{cp.decision}" if cp.decision is not None else "",
                 (f"report: B{cp.report['target']} {'REACHED' if cp.report['success'] else 'not reached'}" if cp.report else "")]
        for l in lines:
            if l:
                self.text(l, (x0 + 12, y), self.f12, TEXT)
                y += 17
        # uplink arrow
        self.dashed((968, y0 + 150), (988, y0 + 150), ONA_BLUE, 4, 3)

    def log_panel(self):
        x0, y0 = 668, 400
        pygame.draw.rect(self.screen, PANEL, (x0, y0, W - x0 - 20, H - y0 - 20), border_radius=8)
        self.text("MISSION LOG", (x0 + 12, y0 + 8), self.f16b)
        msgs = [m for m in self.s.messages if "delivered after 1 attempt" not in m[1]][-14:]
        for i, (t, m) in enumerate(msgs):
            col = WRITER if "[Writer]" in m else EXECUTOR if "[Executor]" in m else (147, 197, 253) if "[ONA" in m \
                else (250, 204, 21) if "[CommandPost]" in m else TEXT
            if "FAILURE" in m or "!!!" in m or "dead" in m or "LOST" in m:
                col = (248, 113, 113)
            line = f"{t / 60:5.1f}′ {m}"
            while self.f12.size(line)[0] > W - x0 - 50:
                line = line[:-2]
            self.text(line, (x0 + 12, y0 + 32 + i * 19), self.f12, col)
