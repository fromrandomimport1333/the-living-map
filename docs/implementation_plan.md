# Implementation Plan (Phase 2: physical prototype, 05/10 → 01/12/2026)

## Timeline

| Week | Dates | Milestone | Owner |
|---|---|---|---|
| W1–W2 | 06–19 Oct | Order hardware. Assemble both chassis. Motor + encoder PID. IMU calibration. | Member 1 (robots) |
| W1–W2 | 06–19 Oct | ESP32-C3 beacons broadcasting the 16-byte packet; RSSI vs distance calibration | Member 3 (beacons) |
| W3 | 20–26 Oct | Writer: odometry + gyro, ToF/lidar obstacle avoidance, occupancy grid (port of `sim/robot.py`) | Member 1 |
| W3 | 20–26 Oct | ONA gateway: Pi Wi-Fi AP + dump receiver + translation (port of `sim/ona.py`) | Member 2 (network) |
| W4 | 27 Oct–2 Nov | Writer: frontier exploration, thermal + camera detection, beacon-drop mechanism (servo + magazine of 12) | Member 1 + 3 |
| W5 | 3–9 Nov | LoRa link ONA ↔ command post laptop. Live map from real frames. Range test on campus. | Member 2 |
| W6 | 10–16 Nov | Executor: brief download, beacon-to-beacon navigation, RSSI arrival, re-routing | Member 1 |
| W7 | 17–23 Nov | Full integration in a mock building (classroom + cardboard walls, heat lamp as fire, mannequin as victim) | All |
| W8 | 24–30 Nov | Failure tests (same 6 scenarios as the simulation), user manual, pitch, final video | All |
| — | 01 Dec | Final submission | All |

## Bill of materials

Prices are estimates in TND; check them with local suppliers.

| Item | Qty | Purpose | Est. unit price |
|---|---|---|---|
| 2WD/4WD robot chassis with encoder motors | 2 | Writer + Executor | 60–90 |
| Raspberry Pi 4 (2 GB) | 2 | Writer brain, ONA gateway | 200–250 |
| ESP32-S3 dev board | 1 | Executor controller | 35–50 |
| L298N / TB6612 motor driver | 2 | Motors | 10–20 |
| VL53L1X ToF sensors (or RPLIDAR A1) | 6 (or 1) | Obstacle sensing / mapping | 25 (or 350) |
| MPU-6050 / BNO055 IMU | 2 | Heading | 10 / 90 |
| MLX90614 IR thermometer (or AMG8833 8×8) | 1 | Fire detection | 40 (or 120) |
| Pi Camera + person detection (MobileNet-SSD) | 1 | Victim detection | 60 |
| ESP32-C3 SuperMini + CR2032 holder | 12 | Beacons | 15 |
| Servo + 3D-printed beacon magazine | 1 | Drop mechanism | 15 |
| SX1276 LoRa modules (868 MHz) + antennas | 2 | ONA ↔ command post | 40 |
| u-blox NEO-M8N GPS + compass | 1 | Anchor (lat0, lon0, ψ) | 60 |
| Li-ion packs + BMS + power bank | 3 | Power | 40 |

**Estimated total: about 1,300–1,600 TND** (lower bound with ToF sensors instead of a lidar).

## Software reuse

The simulation is built around the real interfaces. These parts carry over to the robots almost unchanged:

* `beacon.py`: the packet codec, used as-is on the Pi; ported to C for the ESP32.
* `frame.py`, `ona.py`, `uplink.py`, `command_post.py`: run as-is on the ONA Pi and the command-post laptop.
* `beacon_graph.py`: the Executor's routing.

The robot-side work is mainly replacing the `World` sensor and motion calls with drivers.

## Risks and mitigations

| Risk | Mitigation |
|---|---|
| Lidar too expensive or late | Ring of 6 ToF sensors + wall following. The frontier logic stays the same. |
| RSSI too noisy for "at the beacon" | Add an IR/NFC tag on each beacon for the final 30 cm |
| LoRa duty-cycle limits | 869.4–869.65 MHz sub-band (10 %); only 844 B per mission are needed |
| Integration slips | Simulation-first: every module is tested in the simulation before hardware week W7 |
