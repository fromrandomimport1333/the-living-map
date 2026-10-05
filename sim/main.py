"""Run the Living Map scenario.

  python -m sim.main                       # interactive window (SPACE pause, +/- speed, ESC quit)
  python -m sim.main --fail beacon         # failure injection: beacon | block | writer | link | battery
  python -m sim.main --headless            # no window, print the mission log + results
  python -m sim.main --record media/demo.mp4
"""
import argparse
import json
import os
import subprocess
import sys

from . import config
from .scenario import FAILS, Scenario


def parse():
    p = argparse.ArgumentParser(description="The Living Map — Phase 1 simulation")
    p.add_argument("--fail", choices=FAILS, default="none")
    p.add_argument("--seed", type=int)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--record", metavar="MP4")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--out", default="out")
    return p.parse_args()


def make_scenario(args):
    over = {"seed": args.seed} if args.seed is not None else {}
    return Scenario(config.load(**over), fail=args.fail, out_dir=args.out)


def frames_for(s, last_phase):
    """How many video frames to spend on this tick (slow down on the important moments)."""
    if s.phase != last_phase:
        return 25
    return {"WRITER": 2, "EXECUTOR": 2, "CARRY": 3, "REPORT": 3}.get(s.phase, 2)


def headless(args):
    s = make_scenario(args)
    for _ in s.run():
        pass
    for t, m in s.messages:
        if "delivered after 1 attempt" not in m:
            print(f"{t / 60:6.1f} min  {m}")
    print(json.dumps(s.results, indent=1))
    print(f"\nlive map: {os.path.join(args.out, 'live_map.html')}")


def record(args):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    from .render import H, W, Renderer
    pygame.init()
    s = make_scenario(args)
    r = Renderer(s)
    os.makedirs(os.path.dirname(args.record) or ".", exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{W}x{H}", "-r", str(args.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                           "-crf", "23", args.record], stdin=subprocess.PIPE)
    last, n = None, 0
    for _ in s.run():
        k = frames_for(s, last)
        last = s.phase
        for _ in range(k):
            ff.stdin.write(pygame.image.tobytes(r.draw(), "RGB"))
            n += 1
    for _ in range(args.fps * 4):
        ff.stdin.write(pygame.image.tobytes(r.draw(), "RGB"))
        n += 1
    pygame.image.save(r.draw(), os.path.splitext(args.record)[0] + "_final.png")
    ff.stdin.close()
    ff.wait()
    print(f"wrote {args.record} ({n / args.fps:.0f} s)")


def interactive(args):
    import pygame
    from .render import H, W, Renderer
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("The Living Map — TSYP14 RAS x AESS")
    s = make_scenario(args)
    r = Renderer(s, screen)
    clock, it, speed, paused, done = pygame.time.Clock(), s.run(), 1, False, False
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                pygame.quit()
                return
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    paused = not paused
                elif e.unicode == "+":
                    speed = min(speed * 2, 32)
                elif e.unicode == "-":
                    speed = max(speed // 2, 1)
        if not paused and not done:
            for _ in range(speed):
                try:
                    next(it)
                except StopIteration:
                    done = True
                    print(f"finished — live map at {os.path.join(args.out, 'live_map.html')}")
                    break
        r.draw()
        pygame.display.flip()
        clock.tick(20)


def main():
    args = parse()
    if args.headless:
        headless(args)
    elif args.record:
        record(args)
    else:
        interactive(args)


if __name__ == "__main__":
    sys.exit(main())
