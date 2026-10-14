# Android UWB relay address setting handoff

## Required behavior

Add an editable UWB relay WebSocket address to the existing settings page.

- Preference key: `uwb_server_url`
- Default: `wss://relay.example.invalid/ws/mobile`
- Portable presentation example: `ws://192.168.50.20:18765/ws/mobile`
- Persist with `SharedPreferences`; do not persist or display the mobile token.
- Show the effective address and a `Test connection` action.
- Provide `Restore Raspberry Pi default` as an explicit command.

When the address is saved:

1. Validate scheme `wss://` or `ws://`, host, optional port, and exact path
   `/ws/mobile`.
2. Permit cleartext `ws://` only for loopback, RFC1918 IPv4, or local hostnames;
   require `wss://` for public hosts.
3. Shut down the old `UwbServerClient` so its reconnect worker cannot survive.
4. Construct one new client with the saved endpoint and existing
   `BuildConfig.UWB_MOBILE_TOKEN`, then connect immediately.
5. Keep Bluetooth and vehicle control state untouched during UWB reconnect.

Android currently targets API 36 and does not opt into cleartext traffic.
Add the minimum network-security/manifest change needed for private-LAN `ws://`
connections. The app-side URL validator must still reject public cleartext
addresses.

## Acceptance tests

- A clean install uses the Raspberry Pi `wss://` default.
- Saving a private `ws://IP:18765/ws/mobile` reconnects without app restart.
- Returning to the default reconnects to the public relay.
- Invalid schemes, missing host, and wrong paths are rejected.
- Rotating the phone or reopening the settings page does not duplicate clients.
- Repeated address changes leave only one active WebSocket/reconnect worker.
- Token values never appear in logs, UI, saved preferences, screenshots, or
  exported backups.

The laptop scripts and operational sequence are documented in
`tools/uwb/README.md`.
