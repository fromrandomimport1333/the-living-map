import math

import pytest

from sim.beacon import FIRE, PACKET_LEN, VICTIM, WAYPOINT, Beacon, heading_byte


def test_roundtrip_is_16_bytes():
    b = Beacon(7, VICTIM, 1000, -250, 812, 200, 9, 64, 90)
    pkt = b.encode()
    assert len(pkt) == PACKET_LEN == 16
    assert Beacon.decode(pkt) == b


def test_crc_rejects_any_single_byte_corruption():
    pkt = bytearray(Beacon(3, FIRE, -120, 450, 60, 150, 2, 10, 30).encode())
    for i in range(PACKET_LEN):
        bad = bytearray(pkt)
        bad[i] ^= 0x5A
        with pytest.raises(ValueError):
            Beacon.decode(bytes(bad))


def test_message_aging():
    fire = Beacon(1, FIRE, 0, 0, t_written=0, severity=255, ttl_min=30)
    assert fire.confidence(0) == pytest.approx(1.0)
    assert fire.confidence(600) == pytest.approx(math.exp(-1))       # tau = 10 min
    assert fire.stale(31 * 60) and fire.confidence(31 * 60) == 0.0
    victim = Beacon(2, VICTIM, 0, 0, t_written=0, severity=255, ttl_min=90)
    assert victim.confidence(600) > fire.confidence(600)              # victims age slower than fire
    assert Beacon(3, WAYPOINT, 0, 0, 0).confidence(10 ** 6) == 1.0    # waypoints never age


def test_heading_byte():
    assert heading_byte(1, 0) == 0
    assert heading_byte(0, 1) == 64
    assert heading_byte(-1, 0) == 128
