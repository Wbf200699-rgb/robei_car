# UWB gateway server

This service is the network boundary between the ESP32-S3 attached to LinkPG
main anchor A and the mobile application. It keeps the existing FPGA `@S` and
`@L` lines intact, reassembles the additive six-part `@R` lidar scan, adds UWB
position events, and estimates one rectangular-car pose from four tag
coordinates.

For an off-site presentation without the Raspberry Pi, use the cross-platform
laptop deployment and S3 flashing scripts in `tools/uwb/README.md`. They keep
the public Raspberry Pi endpoint as the Android default and expose a temporary
LAN endpoint on port 18765.

## Local setup

```powershell
python -m venv server/.venv
server/.venv/Scripts/python.exe -m pip install -r server/requirements-dev.txt
Copy-Item server/.env.example server/.env
./server/run_server.ps1 -ListenAddress 0.0.0.0 -Port 8765
```

Change both tokens in `server/.env` before exposing the service to another
machine. The defaults are development placeholders.

## Interfaces

- `GET /health`: unauthenticated process/gateway health.
- `GET /api/v1/state`: current tag fixes, fused pose, gateway counters, and
  latest vehicle UART lines. It also returns `lidar_scan`,
  `lidar_scan_age_ms`, and `lidar_scan_fresh`. Send the mobile token as
  `X-API-Token`.
- `POST /api/v1/command`: `{"command":"W","tag_id":1}`. Direct USB register
  probing confirmed that the physical module labelled tag 1 uses protocol ID 1.
- `POST /api/v1/uwb/run`: `{"running":true}` starts transient automatic LinkPG
  output; `false` stops it.
- `POST /api/v1/uwb/read`: `{"address":59,"count":4}` performs a read-only
  diagnostic register query. The CRC-checked response is retained in
  `gateway_raw_frames` within `/api/v1/state`.
- `POST /api/v1/uwb/calibrate`: stops normal positioning and asks main anchor A
  to measure the six pair distances among the four anchors A/B/C/D.
- `POST /api/v1/uwb/calibrate/cancel`: aborts the measurement and resumes normal
  positioning.
- `/ws/mobile?token=...`: JSON events and low-latency commands.
- `/ws/gateway/<id>?token=...`: ESP32 gateway line protocol.

Mobile WebSocket commands are:

```json
{"type":"command","command":"W","tag_id":1}
{"type":"uwb_run","running":true}
{"type":"anchor_calibrate"}
{"type":"anchor_calibrate_cancel"}
{"type":"ping","nonce":"test-1"}
```

The calibration pairs are `AB`, `AC`, `AD`, `BC`, `BD`, and `CD`; these are six
distances between four anchors, not six anchors. The server trilaterates a
temporary A/B/C/D layout and reports the independent `BD` residual. It does not
write coordinates back to LinkPG automatically. Keep all four anchors fixed,
review the residual, then explicitly confirm any persistent configuration.

Valid per-tag coordinates are passed through an independent five-sample median
filter before pose estimation and mobile broadcast. The latest raw four-anchor
distances, validity flags, sequence number, and timestamps remain unfiltered so
hardware faults and multipath can still be diagnosed.

After all six CRC-valid `@R` parts with matching sequence/tick fields arrive,
mobile clients receive a `lidar_scan` event containing 72 ordered 5-degree
points. Invalid points have `distance_mm: null`; clients must not convert them
to zero-distance obstacles. Incomplete or mixed scans are discarded.

For held manual movement, send a new direction command about every 100 ms and
send `C` on release, matching the existing FPGA 300 ms manual watchdog.

The server intentionally reports a four-tag pose as `unconfigured` until
`UWB_VEHICLE_LENGTH_CM` and `UWB_VEHICLE_WIDTH_CM` are measured and set. This
prevents the current zero-coordinate bench setup from being presented as a
valid map position.

Four fixes must also be individually fresh and mutually synchronized. The
default maximum per-fix age is 0.6 seconds and the maximum oldest-to-newest skew
is 0.15 seconds. A complete but time-skewed set is reported as
`unsynchronized`, not as a valid navigation pose. `/api/v1/state` recomputes
freshness when queried, so an old pose cannot remain marked valid just because
no new frame arrived.

Command diagnostics distinguish `sent_to_gateway`, `gateway_queued`, vehicle
ACK results, and a 3-second `timeout`. Completed results are retained in the
bounded `recent_commands` list. `gateway_data_fresh`, `gateway_status_age_ms`,
per-tag `age_ms`, and `vehicle_line_age_ms` allow the mobile client to tell a
connected WebSocket from a live data stream.

Full serial and envelope details are in `docs/UWB_GATEWAY_PROTOCOL.md`.

## Raspberry Pi Docker deployment

The production container listens on Raspberry Pi port `18765` and keeps all
runtime state in memory:

```bash
cd /home/zyx/little-car-uwb-server
docker compose up -d --build
curl http://127.0.0.1:18765/health
```

Keep `.env` mode `0600`. The ESP32-S3 connects directly to the Raspberry Pi's
LAN address, while remote mobile clients should use the TLS reverse proxy.
