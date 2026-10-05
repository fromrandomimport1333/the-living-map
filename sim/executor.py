"""Executor robot: briefed by the ONA, navigates beacon-to-beacon, executes the mission, returns."""
import math

from .beacon import EVENT_NAME, EXIT, VICTIM, Beacon
from .beacon_graph import BeaconGraph
from .geometry import astar, path_length
from .robot import OCC, Robot

WAIT, GOTO, MISSION, RETURN, DONE, RECOVERY = "WAIT", "GOTO", "MISSION", "RETURN", "DONE", "RECOVERY"


class Executor(Robot):
    def __init__(self, world, cfg, rng, t0):
        super().__init__("Executor", world, cfg, cfg.executor, rng, t0)
        self.state = WAIT
        self.graph = None
        self.route, self.idx, self.target = [], 0, None
        self.prev = None
        self.dead, self.blocked = set(), set()
        self.reroutes = 0
        self.silent = 0
        self.read = {}                 # id -> Beacon heard directly (what the beacons themselves say)
        self.mission_steps = 0
        self.success = False
        self.obstructions = set()
        self.steps = 0
        self.snap_errors = []

    # ---------- briefing -------------------------------------------------
    def load_brief(self, mission):
        nodes = {int(k): v for k, v in mission["beacon_table"].items()}
        self.graph = BeaconGraph(nodes, {(a, b): w for a, b, w in mission["edges"]})
        self.route, self.target = list(mission["route"]), mission["target_beacon"]
        self.idx, self.prev = 0, self.route[0]
        self.state = GOTO
        self.sense()
        self.say(f"briefed by ONA: go to B{self.target} via {self.route}")

    def start_recovery(self):
        self.graph = BeaconGraph({}, {})
        self.state = RECOVERY
        self.sense()
        self.say("no data from the Writer — entering in RECOVERY mode: explore and read the Writer's beacons")

    # ---------- beacons --------------------------------------------------
    def listen(self):
        heard = {}
        for pkt, rssi, d in self.world.hear_beacons(self.cell, self.cfg.beacon.radio_range_m, self.rng):
            try:
                b = Beacon.decode(pkt)
            except ValueError:
                continue
            heard[b.beacon_id] = (b, rssi, d)
            if b.beacon_id not in self.read:
                self.read[b.beacon_id] = b
                if b.event_type not in (0, EXIT):
                    age = b.age_s(self.t)
                    state = "STALE -> unverified" if b.stale(self.t) else f"confidence {b.confidence(self.t):.2f}"
                    self.say(f"reads B{b.beacon_id}: {EVENT_NAME[b.event_type]} written {age / 60:.1f} min ago, {state}")
                if self.state == RECOVERY and b.beacon_id not in self.graph.nodes:
                    self.graph.nodes[b.beacon_id] = {"x": b.x_cm / 100, "y": b.y_cm / 100, "type": b.event_type,
                                                     "conf": b.confidence(self.t), "event_xy": None}
                    self.graph.adj[b.beacon_id] = {}
        return heard

    def target_cell(self, node):
        c = self.world.to_cell(node["x"], node["y"])
        if self.world.inside(c) and self.known_at(c) != OCC and c not in self.hazard:
            return c
        best = None
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                q = (c[0] + dr, c[1] + dc)
                if self.world.inside(q) and self.known_at(q) != OCC and q not in self.hazard:
                    d = abs(dr) + abs(dc)
                    if best is None or d < best[0]:
                        best = (d, q)
        return best[1] if best else c

    def reroute(self, why):
        self.reroutes += 1
        route, cost = self.graph.route(None, self.target, removed_nodes=self.dead,
                                       removed_edges=self.blocked, start_xy=self.believed())
        self.route = route or [self.target]
        self.idx = 0
        self.say(f"{why} -> re-planned over the beacon graph: {self.route}")

    def arrive(self, bid, d):
        n = self.graph.nodes[bid]
        if d < 0.3:                      # on top of the beacon: re-localise on its stored position
            bx, by = self.believed()
            self.snap_errors.append(math.hypot(bx - n["x"], by - n["y"]))
            self.snap_to(n["x"], n["y"])
        self.prev = bid
        self.silent = 0
        self.idx += 1

    # ---------- main loop ------------------------------------------------
    def step(self):
        if self.state in (WAIT, DONE):
            return
        self.steps += 1
        seen, _ = self.sense()
        for p, ch in seen.items():
            if ch == "X" and p not in self.obstructions:
                self.obstructions.add(p)
                if len(self.obstructions) == 1 or all(self.world.dist_m(p, q) > 1.5 for q in self.obstructions - {p}):
                    self.say("obstruction detected by lidar")
        heard = self.listen()
        if self.state == RECOVERY:
            return self._recovery(heard)
        if self.state == GOTO:
            return self._goto(heard)
        if self.state == MISSION:
            return self._mission(seen)
        if self.state == RETURN:
            if self.cell == self.world.entrance:
                self.state = DONE
                self.say("back at the entrance — handing the mission report to the ONA")
                return
            if not self.plan or self.plan[-1] != self.world.entrance:
                self.plan = self.path_home() or []
            if not self.step_along():
                self.plan = []

    def _goto(self, heard):
        if self.idx >= len(self.route):
            self.reroute("route exhausted")
        bid = self.route[self.idx]
        if bid in heard and heard[bid][2] <= self.cfg.beacon.near_range_m:
            self.arrive(bid, heard[bid][2])
            if bid == self.target:
                self.state = MISSION
                self.say(f"reached target beacon B{bid}")
            return
        node = self.graph.nodes[bid]
        tc = self.target_cell(node)
        if self.cell == tc or self.world.dist_m(self.cell, tc) < 0.6:
            if bid in heard:
                # RSSI gradient: step to the free neighbour where the beacon sounds strongest
                b_cell = self.world.beacons[bid].cell
                opts = [q for q in ((self.cell[0] + a, self.cell[1] + c) for a in (-1, 0, 1) for c in (-1, 0, 1))
                        if self.passable_optimistic(q) and self.world.passable(q)]
                nxt = min(opts, key=lambda q: self.world.dist_m(q, b_cell))
                if nxt != self.cell:
                    self.move_to(nxt)
                return
            self.silent += 1
            if self.silent > self.cfg.executor.dead_beacon_patience:
                self.dead.add(bid)
                self.say(f"B{bid} is silent at its recorded position — beacon declared dead")
                if bid == self.target:
                    self.state = MISSION
                    return
                self.silent = 0
                self.reroute(f"dead beacon B{bid}")
            else:
                q = self.rng.choice([(self.cell[0] + a, self.cell[1] + c) for a in (-1, 0, 1) for c in (-1, 0, 1)])
                if self.passable_optimistic(q) and self.world.passable(q):
                    self.move_to(q)
            return
        plan = astar(self.cell, tc, self.passable_optimistic)
        expected = math.hypot(self.cell[0] - tc[0], self.cell[1] - tc[1])
        if plan is None or path_length(plan) > self.cfg.executor.detour_factor * expected + 20:
            if (self.prev, bid) not in self.blocked:
                self.blocked.add((self.prev, bid))
                self.say(f"passage B{self.prev}->B{bid} blocked")
                self.reroute(f"blocked edge B{self.prev}-B{bid}")
                return
            plan = plan or []
        self.plan = plan
        if not self.step_along():
            self.plan = []

    def _mission(self, seen):
        """At the target beacon: find the victim (camera), approach it, confirm, drop the aid kit."""
        self.mission_steps += 1
        victims = [p for p, ch in seen.items() if ch == "V" and self.world.dist_m(self.cell, p) <= 2.5]
        if not self.success and victims:
            v = min(victims, key=lambda p: self.world.dist_m(self.cell, p))
            if self.world.dist_m(self.cell, v) <= 1.0:
                self.success = True
                self.say("victim confirmed by camera — first-aid kit dropped, position marked")
            else:
                goal = min((q for q in ((v[0] + a, v[1] + c) for a in (-1, 0, 1) for c in (-1, 0, 1))
                            if self.passable_optimistic(q) and self.world.passable(q)),
                           key=lambda q: self.world.dist_m(self.cell, q), default=None)
                if goal:
                    self.plan = astar(self.cell, goal, self.passable_optimistic) or []
                    self.step_along()
                return
        node = self.graph.nodes.get(self.target) or {}
        ev = node.get("event_xy")
        if not self.success and self.mission_steps < 20:
            goal = self.target_cell({"x": ev[0], "y": ev[1]}) if ev else None
            if goal and self.world.dist_m(self.cell, goal) > 0.6:
                self.plan = astar(self.cell, goal, self.passable_optimistic) or []
                self.step_along()
                return
            # no exact position: look around the beacon
            q = self.rng.choice([(self.cell[0] + a, self.cell[1] + c) for a in (-1, 0, 1) for c in (-1, 0, 1)])
            if self.passable_optimistic(q) and self.world.passable(q):
                self.move_to(q)
            return
        if self.mission_steps >= 20 or self.success:
            if not self.success:
                self.say("no victim found at the target")
            self.state = RETURN
            self.plan = []
            self.say("mission executed — returning to the exit")

    def _recovery(self, heard):
        victims = [i for i, n in self.graph.nodes.items() if n["type"] == VICTIM and i not in self.dead]
        if victims and self.target is None:
            self.target = max(victims, key=lambda i: self.read[i].confidence(self.t))
            self.route, self.idx, self.prev = [self.target], 0, self.target
            self.state = GOTO
            self.say(f"victim beacon B{self.target} found in the Writer's memory — going there")
            return
        if not self.plan or not self.is_frontier(self.plan[-1]):
            self.plan = self.frontier_path() or []
        if not self.step_along():
            self.plan = []
