# Portable UWB relay for presentations

These scripts switch the ESP32-S3 gateway from the Raspberry Pi relay to a
laptop on the same Wi-Fi network. The FPGA firmware does not change.

## Required local secrets

The scripts first read process environment variables, then the ignored
`server/.env`, and finally the ignored ESP32 `gateway_secrets.h`. They never
print tokens or passwords. On a newly cloned Mac/Windows checkout, set:

- `UWB_GATEWAY_TOKEN`: must match the token used by the Android build.
- `UWB_MOBILE_TOKEN`: must match the token used by the Android build.
- `UWB_WIFI_SSID`: presentation Wi-Fi SSID.
- `UWB_WIFI_PASSWORD`: presentation Wi-Fi password.

Do not commit these values. The S3 build uses a temporary secrets header and
restores or deletes it after compilation, including on failure.

## Windows

Start the relay in the background:

```powershell
.\tools\uwb\deploy_server_windows.ps1
```

Compile and flash the connected ESP32-S3, automatically selecting its USB port:

```powershell
.\tools\uwb\flash_s3_windows.ps1
```

Restore the normal Raspberry Pi/public-server firmware configuration:

```powershell
.\tools\uwb\restore_s3_windows.ps1
```

Explicit presentation network/port example:

```powershell
.\tools\uwb\deploy_server_windows.ps1 -ServerHost 192.168.50.20
.\tools\uwb\flash_s3_windows.ps1 -ServerHost 192.168.50.20 -DevicePort COM3
```

For a TLS relay, also pass `-Tls` and the TLS listener port. The laptop mode
normally stays on private-LAN cleartext port 18765.

Stop the background relay:

```powershell
.\tools\uwb\stop_server_windows.ps1
```

## macOS

Install Python 3, then double-click the `.command` files or run:

```bash
./tools/uwb/deploy_server_macos.command
./tools/uwb/flash_s3_macos.command
./tools/uwb/stop_server_macos.command
```

The first S3 build downloads the official Arduino CLI 1.5.1 into the ignored
`tools/.cache/` directory, verifies its SHA-256, and installs the ESP32 core and
WebSockets library. Restore the normal Raspberry Pi/public-server target with:

```bash
./tools/uwb/restore_s3_macos.command
```

Override automatic LAN/serial detection with environment variables:

```bash
UWB_SERVER_HOST=192.168.50.20 UWB_DEVICE_PORT=/dev/cu.usbmodem101 \
  ./tools/uwb/flash_s3_macos.command
```

## Presentation sequence

1. Put the laptop, phone, and S3 on the same Wi-Fi network. Avoid guest Wi-Fi
   with client isolation.
2. Start the relay. Record the printed mobile endpoint, for example
   `ws://192.168.50.20:18765/ws/mobile`.
3. Flash the S3 using the same IP and port.
4. In the Android settings page, select the printed endpoint and reconnect.
5. Confirm `http://127.0.0.1:18765/health`, then confirm gateway online and tag
   updates in the app.
6. Windows Defender Firewall or the macOS firewall may ask for local-network
   permission. Allow inbound TCP 18765 on the private presentation network.

The public Raspberry Pi default remains:
`wss://relay.example.invalid/ws/mobile`.
Both restore scripts target `relay.example.invalid:443` with TLS; they reuse the
same ignored Wi-Fi/token values and do not embed credentials in the scripts.

## Dry runs

Compile without writing the S3:

```powershell
.\tools\uwb\flash_s3_windows.ps1 -CompileOnly
```

Check the relay without exposing any credentials:

```powershell
python tools\uwb_portable.py server-status
```
