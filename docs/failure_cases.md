# Failure Cases

Six of these failures are **injected and survived in the simulation**
(`python -m sim.main --fail <name>`, tested by `tests/test_scenario.py`). Videos are in `media/`.

| # | Failure | Detection | System response | Simulated |
|---|---|---|---|---|
| 1 | **Writer loses power inside** | No return before the 15 min timeout | ONA declares it lost; Executor sent in RECOVERY mode, reads the Writer's beacons, reaches the victim (B8) and brings the beacons out to the live map | ✅ `writer` (61 % explored before death; mission succeeded) |
| 2 | **Beacon battery dies** | Executor at the recorded position hears nothing for 8 steps | Beacon marked dead; re-route over the beacon graph without it | ✅ `beacon` (B19 dead → route [18, 7, 8]) |
| 3 | **Passage blocked after the Writer passed** | Lidar sees new debris; the planned path becomes > 3× longer | Local re-planning; if that fails, the edge is removed from the beacon graph and the route recomputed | ✅ `block` (doorway collapsed; detour via the next door) |
| 4 | **Long-range link degraded** | Missing ACKs | Stop-and-wait ARQ, 8 retries, then a store-and-forward queue | ✅ `link` (45 % loss: 23 frames needed 64 transmissions, nothing lost) |
| 5 | **Low battery** | Battery < 1.3 × cost to exit + reserve | Writer turns back early so its data survives; partial map still briefs a mission | ✅ `battery` (43 % coverage, mission still succeeded) |
| 6 | **Corrupted packet in the dump** | CRC-16 mismatch | Every packet is dumped twice; the valid copy is used | ✅ always on (4 % per-copy corruption rate injected) |
| 7 | **Odometry drift → wrong GPS** | Error measured at the exit (loop closure) | Gyro drift estimated and removed; ± circle on the live map | ✅ always on (max error 0.70 → 0.28 m) |
| 8 | **Stale information** | `age > ttl` or low confidence | Event shown as *unverified*; the commander's scoring and fire avoidance use the decayed confidence | ✅ always on |
| 9 | Wrong anchor / heading ψ | Translated points fall outside the building footprint | Re-measure ψ from 2 GPS fixes along the façade | design |
| 10 | Commander decision never arrives | Downlink timeout | ONA applies the default policy (best-scored victim) and briefs anyway | design |
| 11 | Beacon moved / kicked | RSSI peak far from its stored position | Lower its trust and navigate on its neighbours | design |
| 12 | ONA power failure | Heartbeat lost at the command post | Robots keep their logs; resend on reconnect; the LoRa queue is persisted | design |
| 13 | Executor lost between beacons | No beacon heard for N seconds | Return to the last beacon heard, then retry the next one | design |
