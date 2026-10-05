# Outside Network Area (ONA)

The ONA is a portable gateway placed at the building entrance, just outside the point where radio stops. It is the
**only bridge** between the inside (robots, beacons) and the outside (command post). Robots never talk to the
command post.

| Role | What it does | Simulation (`sim/ona.py`) | Phase 2 hardware |
|---|---|---|---|
| **1 · RECEIVE** | Get the Writer's memory when it comes out | Binary dump (≈2.4 KB). Each 16-byte packet is stored twice, and the CRC picks the valid copy. Bit errors are injected on the dump channel. | Raspberry Pi 4 as a Wi-Fi AP. The Writer pushes its log over TCP when it docks at the entrance (BLE fallback). |
| **2 · TRANSLATE** | Private frame → GPS that anyone can use | Gyro-drift loop closure, then private → ENU → WGS-84, plus ± uncertainty. Builds the beacon graph. | Pi + u-blox GPS + compass for ψ |
| **3 · CARRY** | Reach a distant command post with no infrastructure | 51-byte LoRa frames (SF7, 869.5 MHz, 10 % duty-cycle sub-band). Stop-and-wait ARQ with ACK, up to 8 retries, then store-and-forward. Loss model of 10 % (45 % in `--fail link`). | 2× SX1276 LoRa modules (several km line of sight), relay node if needed. Iridium SBD modem as a satellite fallback. |
| **4 · BRIEF** | Prepare the Executor before entry | Mission JSON (`out/mission_brief.json`) with the target, beacon route (Dijkstra over the beacon graph with a +40 m penalty near confident fire), beacon table, edges and fire beacons to keep clear of. | Same Wi-Fi AP. The Executor downloads the brief before entering. |

## Uplink frames (each ≤ 51 bytes)

| Type | Content | Size |
|---|---|---|
| 1 BEACONS | 2 records × 18 B: id, type, lat/lon (1e-7°), t, severity, confidence, ±dm, next_id | 39 B |
| 2 PATH | 12 points × (east, north) int16 decimetres | 51 B |
| 3 STATUS | Writer state, coverage, battery, number of beacons | 6 B |
| 4 DECISION (downlink) | Target beacon id, policy | 4 B |
| 5 REPORT | Target, success, lat/lon, time, re-routes, dead beacons | 17 B |
| 6 EXEC_PATH | Like PATH, for the Executor | ≤ 51 B |

Nominal run: **23 frames / 844 B, delivered with 30 transmissions** (7 losses recovered). At 1.2 s per frame including
the duty-cycle gap, the full map reaches the command post in under a minute.

## Command post

* Decodes frames, de-duplicates by sequence number, and writes `out/live_map.html`. This is a Leaflet/OpenStreetMap page
  that refreshes every 3 s and shows events coloured by type, confidence, age, ± circles, the Writer and Executor paths,
  and the mission report.
* **Commander decision** (automatic in the simulation, a human in reality):
  `score = confidence_now / (1 + route_length/30) × (0.3 if the route passes within 3 m of a confident fire)`.
* Only the decision is sent back, and it goes to the ONA.

## Writer lost

If the Writer is not back after 15 min, the ONA declares it lost and sends the Executor in **recovery mode**. The
Executor explores, reads the Writer's beacons and goes to the first victim beacon it hears. When it comes out, the
ONA uplinks the beacons the Executor read. The Writer's memory survives the Writer.
