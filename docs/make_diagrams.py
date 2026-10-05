"""Generate the technical diagrams in docs/ (python docs/make_diagrams.py)."""
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle

OUT = Path(__file__).parent
ORANGE, GREEN, BLUE, DARK, RED, GREY = "#c2410c", "#15803d", "#1e5a8a", "#1f2937", "#dc2626", "#6b7280"


def box(ax, x, y, w, h, title, body="", fc="white", ec=BLUE, tc="white", lw=1.5, title_h=0.32, fs=9):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06", fc=fc, ec=ec, lw=lw))
    ax.add_patch(Rectangle((x, y + h - title_h), w, title_h, fc=ec, ec=ec))
    ax.text(x + 0.08, y + h - title_h / 2, title, color=tc, fontsize=fs, fontweight="bold", va="center")
    ax.text(x + 0.08, y + h - title_h - 0.08, body, fontsize=fs - 1.5, va="top", color=DARK, linespacing=1.35)


def arrow(ax, a, b, col, text="", off=(0, 0.12), ls="-", fs=8, rad=0.0):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=14, color=col, lw=1.8, ls=ls,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, color=col, fontsize=fs,
                ha="center", fontweight="bold")


def architecture():
    fig, ax = plt.subplots(figsize=(13, 5.6))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5.6)
    ax.axis("off")
    # building
    ax.add_patch(Rectangle((0.2, 0.6), 3.1, 4.3, fc="#f3f4f6", ec=DARK, lw=2))
    ax.text(0.35, 4.6, "THE BUILDING", fontweight="bold", fontsize=11)
    ax.text(0.35, 4.33, "no GPS · no network · GPS-denied", fontsize=8, color=GREY)
    for (x, y, c, l) in ((0.8, 3.6, BLUE, "B3 waypoint"), (2.2, 3.3, RED, "B5 fire"),
                         (1.0, 2.4, GREEN, "B8 victim"), (2.4, 1.9, "#a16207", "B17 blocked")):
        ax.add_patch(plt.Circle((x, y), 0.13, fc=c, ec="white", lw=1.5))
        ax.text(x + 0.18, y - 0.05, l, fontsize=7)
    ax.text(0.35, 1.45, "16-byte BLE beacons on the floor\n(what · where to go · when)", fontsize=7.5, color=BLUE,
            style="italic")
    ax.add_patch(Rectangle((0.5, 0.75), 0.45, 0.3, fc=ORANGE))
    ax.text(1.05, 0.83, "WRITER explores, detects, drops\nbeacons, carries its log out", fontsize=7.5, color=ORANGE)
    # radio line
    ax.plot([3.6, 3.6], [0.4, 5.2], ls="--", color=RED, lw=2)
    ax.text(3.6, 5.3, "RADIO STOPS HERE", color=RED, fontsize=9, fontweight="bold", ha="center")
    # ONA
    ax.add_patch(FancyBboxPatch((3.95, 0.55), 5.4, 4.4, boxstyle="round,pad=0.02,rounding_size=0.1",
                                fc="#eaf2fa", ec=BLUE, lw=2, ls="--"))
    ax.text(6.65, 4.62, "THE OUTSIDE NETWORK AREA (gateway at the entrance)", color=BLUE, fontweight="bold",
            ha="center", fontsize=10)
    box(ax, 4.15, 2.65, 2.45, 1.75, "ROLE 1 — RECEIVE",
        "Writer's log dumped at the exit\n(Wi-Fi/BLE, 2.4 KB)\nCRC-16 on every packet,\neach packet stored twice")
    box(ax, 6.75, 2.65, 2.45, 1.75, "ROLE 2 — TRANSLATE",
        "Gyro-drift loop closure\nprivate (x, y) -> ENU -> GPS\nanchor: entrance GPS + bearing ψ\n± uncertainty per point")
    box(ax, 4.15, 0.75, 2.45, 1.75, "ROLE 3 — CARRY",
        "LoRa 869.5 MHz, SF7, 51-B frames\nstop-and-wait ARQ + ACK\nstore-and-forward queue\n(satellite modem as backup)")
    box(ax, 6.75, 0.75, 2.45, 1.75, "ROLE 4 — BRIEF",
        "mission packet (JSON):\ntarget, beacon route,\nbeacon table + graph,\nfire beacons to keep clear of")
    # robots arrows
    arrow(ax, (0.95, 1.05), (4.1, 3.2), ORANGE, "log dump\n(short range)", off=(0.55, -0.05))
    ax.add_patch(Rectangle((10.0, 0.6), 0.6, 0.38, fc=GREEN))
    ax.text(10.3, 0.35, "EXECUTOR\nbriefed before entry", color=GREEN, fontsize=8, ha="center", fontweight="bold", va="top")
    arrow(ax, (9.2, 1.4), (10.0, 0.85), GREEN, "brief", off=(0.1, 0.18))
    arrow(ax, (10.0, 0.7), (3.3, 0.9), GREEN, "", ls="--", rad=-0.12)
    ax.text(6.6, 0.18, "Executor enters and navigates beacon-to-beacon (inherited memory)", color=GREEN,
            fontsize=8, ha="center", style="italic")
    # command post
    box(ax, 10.5, 2.6, 2.3, 1.9, "COMMAND POST",
        "far away · no infrastructure\nLIVE MAP (Leaflet/OSM)\nevents + confidence + ±m\ncommander decides target", ec=DARK)
    arrow(ax, (9.35, 3.3), (10.45, 3.3), BLUE, "uplink\n(LoRa)", off=(0, 0.2))
    arrow(ax, (10.45, 2.85), (9.35, 2.85), DARK, "decision", off=(0, -0.3), ls="--")
    ax.text(11.65, 2.25, "NO direct link between\nthe robots and the command post", color=RED, fontsize=8,
            ha="center", fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "architecture.png", dpi=170)
    plt.close(fig)


def dataflow():
    lanes = ["Writer", "Beacons", "ONA", "Command post", "Executor"]
    cols = [ORANGE, BLUE, "#2563eb", DARK, GREEN]
    steps = [
        (0, 0, "explore (frontier + A*), lidar + thermal + camera"),
        (0, 1, "event / doorway / 3 m -> drop beacon (16 B)"),
        (0, 0, "battery < 1.3 x cost-to-exit -> return"),
        (0, 2, "exit: binary log dump (packets x2, odometry path)"),
        (2, 2, "RECEIVE: CRC check, keep the valid copy"),
        (2, 2, "TRANSLATE: estimate gyro drift, private -> GPS"),
        (2, 3, "CARRY: 20+ LoRa frames, ACK + retry"),
        (3, 3, "live map update · commander scores victims"),
        (3, 2, "decision frame (target beacon)"),
        (2, 4, "BRIEF: route, beacon table, graph, fire clearance"),
        (4, 1, "enter, listen, go beacon-to-beacon, re-localise"),
        (1, 4, "beacon says: fire 9.8 min old, conf 0.14"),
        (4, 4, "dead beacon / blocked passage -> re-route"),
        (4, 4, "victim confirmed, aid kit dropped"),
        (4, 2, "exit: mission report"),
        (2, 3, "CARRY: report + Executor path -> live map"),
    ]
    fig, ax = plt.subplots(figsize=(12, 7.2))
    n = len(steps)
    ax.set_xlim(-0.6, len(lanes) - 0.4)
    ax.set_ylim(-n - 0.6, 1.0)
    ax.axis("off")
    for i, (l, c) in enumerate(zip(lanes, cols)):
        ax.add_patch(FancyBboxPatch((i - 0.42, 0.25), 0.84, 0.55, boxstyle="round,pad=0.02", fc=c, ec=c))
        ax.text(i, 0.52, l, ha="center", va="center", color="white", fontweight="bold")
        ax.plot([i, i], [0.25, -n - 0.3], color=c, lw=1, ls=":")
    for k, (a, b, label) in enumerate(steps):
        y = -k - 0.4
        if a == b:
            ax.add_patch(Rectangle((a - 0.04, y - 0.2), 0.08, 0.4, fc=cols[a], ec=cols[a]))
            ax.text(a + 0.1, y, label, va="center", fontsize=8.5, color=DARK,
                    bbox=dict(fc="white", ec="none", pad=0.5))
        else:
            arrow(ax, (a, y), (b, y), cols[a])
            ax.text((a + b) / 2, y + 0.13, label, ha="center", fontsize=8.5, color=DARK,
                    bbox=dict(fc="white", ec="none", pad=0.5))
    ax.text(len(lanes) / 2 - 0.5, -n - 0.55, "All inside <-> outside traffic passes through the ONA. Robots never talk "
            "to the command post.", ha="center", color=RED, fontsize=9, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / "dataflow.png", dpi=170)
    plt.close(fig)


def packet():
    fields = [("beacon_id", 1, "#60a5fa"), ("type", 1, "#f87171"), ("x_cm", 2, "#34d399"), ("y_cm", 2, "#34d399"),
              ("t_written (s)", 4, "#fbbf24"), ("severity", 1, "#f87171"), ("next_id", 1, "#a78bfa"),
              ("heading", 1, "#a78bfa"), ("ttl_min", 1, "#fbbf24"), ("CRC-16", 2, "#9ca3af")]
    groups = {"#f87171": "WHAT was found", "#34d399": "WHERE (private frame)", "#a78bfa": "WHERE TO GO (toward exit)",
              "#fbbf24": "WHEN / aging", "#60a5fa": "identity", "#9ca3af": "integrity"}
    fig, ax = plt.subplots(figsize=(12, 2.3))
    ax.set_xlim(0, 16)
    ax.set_ylim(-1.2, 1.4)
    ax.axis("off")
    x = 0
    for name, n, c in fields:
        ax.add_patch(Rectangle((x, 0), n, 1, fc=c, ec="white", lw=2))
        ax.text(x + n / 2, 0.5, name, ha="center", va="center", fontsize=8.5, fontweight="bold")
        ax.text(x + n / 2, 1.12, f"{n} B" if n > 1 else "1 B", ha="center", fontsize=7.5, color=GREY)
        x += n
    for i, (c, label) in enumerate(groups.items()):
        ax.add_patch(Rectangle((i * 2.68, -0.85), 0.3, 0.3, fc=c))
        ax.text(i * 2.68 + 0.38, -0.7, label, va="center", fontsize=8)
    ax.text(8, -1.15, "16 bytes total — fits one BLE legacy advertisement (31 B payload); broadcast every 500 ms",
            ha="center", fontsize=8.5, color=DARK)
    fig.tight_layout()
    fig.savefig(OUT / "beacon_packet.png", dpi=170)
    plt.close(fig)


def writer_fsm():
    dot = """digraph G { rankdir=LR; bgcolor="white"; node [shape=box, style="rounded,filled", fontname="Helvetica",
    fontsize=11, fillcolor="#fff7ed", color="#c2410c"]; edge [fontname="Helvetica", fontsize=9, color="#374151"];
    START [label="START\\ndrop B0 (exit)", fillcolor="#ede9fe", color="#7c3aed"];
    EXPLORE [label="EXPLORE\\nnearest frontier + A*\\nlidar · thermal · camera"];
    EVENT [label="EVENT\\nnew fire / victim / blocked\\n(dedupe < 2 m)"];
    DROP [label="DROP BEACON\\nevent · doorway · every 3 m\\nkeep 4 for events"];
    RETURN [label="RETURN\\nA* to the exit"];
    DUMP [label="AT EXIT\\nloop-closure error\\ndump log to ONA", fillcolor="#dbeafe", color="#1e5a8a"];
    DEAD [label="POWER LOST\\nbeacons keep the memory", fillcolor="#fee2e2", color="#dc2626"];
    START -> EXPLORE; EXPLORE -> EVENT [label="sensor fires"]; EVENT -> DROP; EXPLORE -> DROP [label="doorway / 3 m"];
    DROP -> EXPLORE; EXPLORE -> RETURN [label="battery < 1.3 x cost-to-exit + 15\\nor no frontier left"];
    RETURN -> DUMP [label="at entrance"]; EXPLORE -> DEAD [style=dashed, color="#dc2626"]; }"""
    subprocess.run(["dot", "-Tpng", "-Gdpi=150", "-o", str(OUT / "writer_fsm.png")], input=dot.encode(), check=True)


if __name__ == "__main__":
    architecture()
    dataflow()
    packet()
    writer_fsm()
    print("diagrams written to", OUT)
