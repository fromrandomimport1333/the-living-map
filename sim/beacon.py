"""16-byte beacon packet: what was found, where to go, when it was written."""
import binascii
import math
import struct
from dataclasses import dataclass

FMT = "<BBhhIBBBB"                 # 14-byte body, then CRC-16/CCITT
PACKET_LEN = 16
NO_BEACON = 255

WAYPOINT, FIRE, VICTIM, BLOCKED, EXIT = range(5)
EVENT_NAME = {WAYPOINT: "waypoint", FIRE: "fire", VICTIM: "victim", BLOCKED: "blocked", EXIT: "exit"}
TAU_S = {WAYPOINT: math.inf, FIRE: 600, VICTIM: 1800, BLOCKED: 7200, EXIT: math.inf}
TTL_MIN = {WAYPOINT: 255, FIRE: 30, VICTIM: 90, BLOCKED: 240, EXIT: 255}


@dataclass
class Beacon:
    beacon_id: int
    event_type: int
    x_cm: int                      # position in the Writer's private frame
    y_cm: int
    t_written: int                 # seconds since mission start
    severity: int = 0              # 0..255 (temperature, detection score, ...)
    next_id: int = NO_BEACON       # next beacon toward the exit
    heading_next: int = 0          # direction to next_id, 0..255 -> 0..360 deg
    ttl_min: int = 255

    def encode(self) -> bytes:
        body = struct.pack(FMT, self.beacon_id, self.event_type, self.x_cm, self.y_cm,
                           self.t_written, self.severity, self.next_id,
                           self.heading_next, self.ttl_min)
        return body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))

    @staticmethod
    def decode(pkt: bytes) -> "Beacon":
        if len(pkt) != PACKET_LEN:
            raise ValueError("bad length")
        body, (crc,) = pkt[:14], struct.unpack("<H", pkt[14:])
        if binascii.crc_hqx(body, 0xFFFF) != crc:
            raise ValueError("CRC mismatch")
        return Beacon(*struct.unpack(FMT, body))

    @property
    def name(self) -> str:
        return EVENT_NAME[self.event_type]

    def age_s(self, now_s: float) -> float:
        return max(0.0, now_s - self.t_written)

    def stale(self, now_s: float) -> bool:
        return self.age_s(now_s) > self.ttl_min * 60

    def confidence(self, now_s: float) -> float:
        """Trust in the *event* carried by this beacon; decays with age, 0 when stale."""
        if self.event_type in (WAYPOINT, EXIT):
            return 1.0
        if self.stale(now_s):
            return 0.0
        return self.severity / 255 * math.exp(-self.age_s(now_s) / TAU_S[self.event_type])


def heading_byte(dx: float, dy: float) -> int:
    return int(round((math.degrees(math.atan2(dy, dx)) % 360) / 360 * 256)) % 256
