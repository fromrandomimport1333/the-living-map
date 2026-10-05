"""Run the scenarios and produce the result figures/tables used in the report (python docs/make_figures.py)."""
import json
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from sim import config                                   # noqa: E402
from sim.frame import to_enu                             # noqa: E402
from sim.scenario import FAILS, Scenario                 # noqa: E402

OUT = Path(__file__).parent
SEEDS = (7, 11, 23, 42, 99)


def run(fail="none", seed=7):
    s = Scenario(config.load(seed=seed), fail=fail, out_dir=tempfile.mkdtemp())
    for _ in s.run():
        pass
    return s


def translation_figure(s):
    psi = s.cfg.anchor.psi_deg
    wd, ona = s.world, s.ona
    fig, ax = plt.subplots(figsize=(8, 5.4))
    for r in range(wd.H):
        for c in range(wd.W):
            if wd.grid[r][c] == "#":
                e, n = to_enu(*wd.to_private((r, c)), psi)
                ax.add_patch(plt.Rectangle((e - 0.25, n - 0.25), 0.5, 0.5, fc="#cbd5e1", ec="none"))
    raw = [to_enu(b.x_cm / 100, b.y_cm / 100, psi) for b in ona.beacons.values()]
    fix = [to_enu(ona.corrected[i]["x"], ona.corrected[i]["y"], psi) for i in ona.beacons]
    tru = [to_enu(*wd.to_private(s.writer.truth[i]), psi) for i in ona.beacons]
    for (a, b), (c, d) in zip(tru, raw):
        ax.plot([a, c], [b, d], color="#f97316", lw=0.8)
    ax.scatter(*zip(*tru), s=28, c="#111827", label="true beacon position", zorder=3)
    ax.scatter(*zip(*raw), s=22, c="#f97316", marker="x", label="Writer odometry (raw)", zorder=3)
    ax.scatter(*zip(*fix), s=26, facecolors="none", edgecolors="#2563eb", label="after gyro-drift loop closure", zorder=3)
    pe = [to_enu(x, y, psi) for x, y, _ in s.ona.path]
    ax.plot(*zip(*pe), color="#f97316", lw=0.6, alpha=0.5, label="raw odometry path")
    ax.plot(*zip(*ona.path_enu), color="#2563eb", lw=0.6, alpha=0.6, label="corrected path")
    ax.plot(0, 0, marker="^", color="#eab308", ms=12, ls="none", label="entrance = GPS anchor")
    ax.set_aspect("equal")
    ax.set_xlabel("East of anchor (m)")
    ax.set_ylabel("North of anchor (m)")
    ax.legend(fontsize=7.5, loc="lower left", ncol=2)
    t = s.results["translation"]
    ax.set_title(f"Frame translation — beacon error raw {t['beacon_error_raw_mean_m']:.2f} m (max {t['beacon_error_raw_max_m']:.2f})"
                 f" -> corrected {t['beacon_error_corrected_mean_m']:.2f} m (max {t['beacon_error_corrected_max_m']:.2f})",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(OUT / "fig_translation.png", dpi=170)
    plt.close(fig)


def main():
    nominal = run()
    translation_figure(nominal)
    seeds = []
    for seed in SEEDS:
        r = run(seed=seed).results
        seeds.append({"seed": seed, **r["translation"], "coverage": r["writer"]["coverage"],
                      "writer_time_s": r["writer"]["time_s"], "success": r["executor"]["success"],
                      "link_attempts": r["link"]["attempts"], "link_frames": r["link"]["frames"]})
    failures = {}
    for f in FAILS:
        r = run(f).results
        failures[f] = {"success": r["executor"]["success"], "writer_state": r["writer"]["state"],
                       "coverage": r["writer"]["coverage"], "executor_steps": r["executor"]["steps"],
                       "reroutes": r["executor"]["reroutes"], "dead_beacons": r["executor"]["dead_beacons"],
                       "mode": r["executor"]["mode"], "total_time_min": round(r["total_time_s"] / 60, 1),
                       "link": f"{r['link']['delivered']}/{r['link']['frames']} frames, {r['link']['attempts']} tx"}
    mean = lambda k: round(sum(x[k] for x in seeds) / len(seeds), 3)
    summary = {"nominal": nominal.results, "seeds": seeds,
               "seed_mean": {k: mean(k) for k in ("beacon_error_raw_mean_m", "beacon_error_raw_max_m",
                                                  "beacon_error_corrected_mean_m", "beacon_error_corrected_max_m",
                                                  "closure_error_m")},
               "failures": failures}
    (OUT / "results_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary["seed_mean"], indent=1))
    print(json.dumps(failures, indent=1))


if __name__ == "__main__":
    main()
