"""Long-range ONA <-> command post link: lossy LoRa model with stop-and-wait ARQ + store-and-forward."""
from collections import deque
from dataclasses import dataclass, field


@dataclass
class LinkStats:
    frames: int = 0
    delivered: int = 0
    attempts: int = 0
    lost_data: int = 0
    lost_ack: int = 0
    deferred: int = 0
    bytes: int = 0
    airtime_s: float = 0.0


@dataclass
class LoRaLink:
    cfg: object
    rng: object
    stats: LinkStats = field(default_factory=LinkStats)
    queue: deque = field(default_factory=deque)
    seq: int = 0

    def frame(self, kind, payload: bytes) -> bytes:
        f = bytes([kind, self.seq & 0xFF]) + payload
        if len(f) > self.cfg.max_payload:
            raise ValueError(f"frame {len(f)} B > {self.cfg.max_payload} B")
        self.seq += 1
        return f

    def push(self, frames):
        for f in frames:
            self.queue.append(f)
            self.stats.frames += 1
            self.stats.bytes += len(f)

    def send_next(self):
        """Try to deliver the head-of-queue frame. Returns (frame|None, elapsed_s, log line)."""
        if not self.queue:
            return None, 0.0, None
        f = self.queue[0]
        c, elapsed, got = self.cfg, 0.0, False
        for attempt in range(1, c.max_retries + 1):
            self.stats.attempts += 1
            elapsed += c.airtime_s + c.latency_s + c.duty_gap_s
            self.stats.airtime_s += c.airtime_s
            if self.rng.random() < c.loss:
                self.stats.lost_data += 1
                continue
            got = True                       # receiver has it (duplicates filtered by seq)
            elapsed += c.ack_airtime_s
            if self.rng.random() < c.loss:
                self.stats.lost_ack += 1
                continue
            self.queue.popleft()
            self.stats.delivered += 1
            return f, elapsed, f"frame #{f[1]} ({len(f)} B) delivered after {attempt} attempt(s)"
        # give up for now: keep the frame (store-and-forward) and back off
        self.queue.rotate(-1)
        self.stats.deferred += 1
        elapsed += 10.0
        return (f if got else None), elapsed, f"frame #{f[1]} deferred — link down, kept in store-and-forward queue"
