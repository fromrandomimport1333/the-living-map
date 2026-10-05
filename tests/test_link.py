import random

from sim import uplink
from sim.config import Cfg
from sim.link import LoRaLink

CFG = Cfg({"loss": 0.3, "max_payload": 51, "airtime_s": 0.1, "ack_airtime_s": 0.05,
           "duty_gap_s": 1.0, "latency_s": 0.3, "max_retries": 8})


def test_frames_fit_lora_payload():
    recs = [{"id": i, "type": 2, "lat": 36.80, "lon": 10.18, "t": 100, "severity": 200, "conf": 0.5,
             "sigma": 0.4, "next_id": 0} for i in range(5)]
    for p in uplink.beacon_payloads(recs) + uplink.path_payloads([(1.0, 2.0)] * 30):
        assert len(p) + 2 <= 51
    back = [r for p in uplink.beacon_payloads(recs) for r in uplink.decode_beacons(p)]
    assert [r["id"] for r in back] == list(range(5))
    assert back[0]["lat"] == 36.80


def test_every_frame_eventually_delivered_despite_loss():
    link = LoRaLink(CFG, random.Random(1))
    link.push([link.frame(uplink.STATUS, bytes(4)) for _ in range(40)])
    got = set()
    while link.queue:
        f, _, _ = link.send_next()
        if f:
            got.add(f[1])
    assert got == set(range(40))
    assert link.stats.attempts > 40          # retries happened
