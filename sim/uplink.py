"""Compact frames carried over the long-range link (each <= 51 bytes, LoRa SF7)."""
import struct

BEACONS, PATH, STATUS, DECISION, REPORT, EXEC_PATH = range(1, 7)
BEACON_REC = "<BBiiIBBBB"          # 18 bytes
BEACON_REC_LEN = struct.calcsize(BEACON_REC)
PER_BEACON_FRAME = 2
PER_PATH_FRAME = 12


def beacon_payloads(records):
    """records: dicts with id, type, lat, lon, t, severity, conf (0..1), sigma (m), next_id."""
    out = []
    for i in range(0, len(records), PER_BEACON_FRAME):
        chunk = records[i:i + PER_BEACON_FRAME]
        p = bytes([len(chunk)])
        for r in chunk:
            p += struct.pack(BEACON_REC, r["id"], r["type"], int(round(r["lat"] * 1e7)),
                             int(round(r["lon"] * 1e7)), int(r["t"]), int(r["severity"]),
                             int(round(r["conf"] * 255)), min(255, int(round(r["sigma"] * 10))), r["next_id"])
        out.append(p)
    return out


def decode_beacons(payload):
    n, recs = payload[0], []
    for i in range(n):
        f = struct.unpack_from(BEACON_REC, payload, 1 + i * BEACON_REC_LEN)
        recs.append({"id": f[0], "type": f[1], "lat": f[2] / 1e7, "lon": f[3] / 1e7, "t": f[4],
                     "severity": f[5], "conf": f[6] / 255, "sigma": f[7] / 10, "next_id": f[8]})
    return recs


def path_payloads(enu_points):
    """enu_points: [(east_m, north_m)] relative to the anchor -> int16 decimetres."""
    out = []
    for i in range(0, len(enu_points), PER_PATH_FRAME):
        chunk = enu_points[i:i + PER_PATH_FRAME]
        out.append(bytes([len(chunk)]) + b"".join(struct.pack("<hh", int(round(e * 10)), int(round(n * 10)))
                                                  for e, n in chunk))
    return out


def decode_path(payload):
    return [(e / 10, n / 10) for e, n in (struct.unpack_from("<hh", payload, 1 + 4 * i) for i in range(payload[0]))]


def status_payload(writer_state, coverage, battery, n_beacons):
    return struct.pack("<BBBB", writer_state, int(coverage * 100), int(battery * 100), n_beacons)


def decode_status(p):
    s, cov, bat, n = struct.unpack("<BBBB", p)
    return {"writer_state": s, "coverage": cov / 100, "battery": bat / 100, "beacons": n}


def decision_payload(target_id, policy):
    return struct.pack("<BB", target_id, policy)


def report_payload(target_id, success, lat, lon, t, rerouted, dead_beacons):
    return struct.pack("<BBiiIBB", target_id, success, int(round(lat * 1e7)), int(round(lon * 1e7)),
                       int(t), rerouted, dead_beacons)


def decode_report(p):
    tid, ok, lat, lon, t, rr, dead = struct.unpack("<BBiiIBB", p)
    return {"target": tid, "success": bool(ok), "lat": lat / 1e7, "lon": lon / 1e7, "t": t,
            "rerouted": rr, "dead_beacons": dead}
