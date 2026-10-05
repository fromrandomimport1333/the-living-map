import pytest

from sim import config
from sim.scenario import FAILS, Scenario


def run(fail, tmp_path):
    s = Scenario(config.load(), fail=fail, out_dir=tmp_path)
    for _ in s.run():
        pass
    return s


def test_nominal_mission(tmp_path):
    s = run("none", tmp_path)
    r = s.results
    assert r["writer"]["state"] == "DONE"
    assert r["writer"]["coverage"] > 0.95
    assert {"fire", "victim", "blocked"} <= set(r["writer"]["events_found"])
    assert r["translation"]["beacon_error_corrected_max_m"] < r["translation"]["beacon_error_raw_max_m"]
    assert r["executor"]["success"]
    assert s.cp.report and s.cp.report["success"]
    assert (tmp_path / "live_map.html").exists()


def test_no_direct_robot_to_command_post_link(tmp_path):
    """Everything the command post knows arrived as LoRa frames from the ONA."""
    s = run("none", tmp_path)
    assert s.cp.frames_rx == s.link.stats.delivered - 1          # minus the downlink decision frame
    assert not hasattr(s.writer, "cp") and not hasattr(s.executor, "cp")


@pytest.mark.parametrize("fail", [f for f in FAILS if f != "none"])
def test_failures_are_survived(fail, tmp_path):
    s = run(fail, tmp_path)
    assert s.results["executor"]["success"], fail
    if fail == "beacon":
        assert s.results["executor"]["dead_beacons"]
    if fail == "writer":
        assert s.results["writer"]["state"] == "DEAD"
        assert s.results["executor"]["mode"] == "recovery"
    if fail == "battery":
        assert s.results["writer"]["coverage"] < 0.9
