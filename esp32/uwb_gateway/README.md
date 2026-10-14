# ESP32-S3 LinkPG gateway

This Arduino sketch connects LinkPG main anchor A to the UWB server. It parses
CRC-checked `CA DA` position frames and `ED DA` data-pass frames, and sends
server commands to a selected vehicle tag with `AT+DataSend`.

## Wiring for the first gateway test

The current build targets an ESP32-S3 N16R8 module. GPIO13/GPIO14 are kept as
the LinkPG UART pins so the existing gateway harness does not change.

| ESP32-S3 | LinkPG main anchor A | Purpose |
| --- | --- | --- |
| 5V | 5V supply rail | power; use a supply sized for both boards |
| GND | GND | common reference |
| GPIO13 / RX2 | TXD | LinkPG to ESP32 |
| GPIO14 / TX2 | RXD | ESP32 to LinkPG |

Do not connect a LinkPG TX signal to the ESP32 or FPGA until its idle-high
voltage has been measured and confirmed as 3.3 V compatible. Do not use the
LinkPG USB serial and TTL serial simultaneously during this test.

## Build

```powershell
Copy-Item esp32/uwb_gateway/gateway_secrets.example.h esp32/uwb_gateway/gateway_secrets.h
# Edit only the ignored gateway_secrets.h with Wi-Fi/server values.
arduino-cli compile --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=opi,FlashMode=qio,USBMode=hwcdc,CDCOnBoot=default,UploadMode=default" esp32/uwb_gateway
```

Required Arduino library: `WebSockets` 2.7.x. The current machine has version
2.7.2 installed.

The default firmware waits three seconds for the main anchor to boot, writes a
transient stop to clear a stale run state, then starts transient automatic
output by writing register 59 value 4. It verifies the register-59 ACK and uses
a bounded retry/recovery state machine; it does not continuously flood the
anchor with start writes after a successful ACK. The UART parser uses sliding
CRC resynchronization so a partial power-on frame cannot hide all subsequent
valid frames. The firmware does not write anchor coordinates, roles, tag lists,
channel settings, or persistent boot mode. Set
`AUTO_START_POSITIONING` false in `gateway_config.h` when configuration work is
being performed from the vendor PC tool.

The server may request a temporary four-anchor calibration. The gateway stops
normal positioning by writing register 59 value `9` as required when the main
anchor is in power-on automatic mode, enables LinkPG anchor mutual ranging,
measures exactly six
pairs (`AB`, `AC`, `AD`, `BC`, `BD`, `CD`), reports each CRC-valid result as a
`CALPAIR` line, exits ranging mode, and restores register 59 value `8` so
power-on automatic positioning remains enabled. It never writes calculated
anchor coordinates into persistent LinkPG registers.

Firmware 0.4.0 accepts either the official write ACK or a valid `CA DA` frame as
progress. Missing ACKs cause at most three closely spaced start attempts; a
longer recovery interval then repeats the stop/start handshake. A server
`UWB_STOP` changes the desired state to stopped, so the watchdog does not
immediately restart an intentional stop. USB diagnostics include stop/start
request counts, ACK counts, and the recovery-state number. The existing server
`STATUS` message remains wire-compatible.

The camera is intentionally not initialized in this gateway firmware.

Protocol details are in `docs/UWB_GATEWAY_PROTOCOL.md`.

For presentation use, `tools/uwb/flash_s3_windows.ps1` and
`tools/uwb/flash_s3_macos.command` compile this same source with a temporary
laptop LAN endpoint, upload it, and restore the ignored secrets header. See
`tools/uwb/README.md`; do not flash the stale `build_esp32s3` directory.
