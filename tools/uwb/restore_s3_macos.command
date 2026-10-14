#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DEVICE_PORT=${UWB_DEVICE_PORT:-auto}
exec "$SCRIPT_DIR/flash_s3_macos.command" --server-host relay.example.invalid \
  --server-port 443 --device-port "$DEVICE_PORT" --tls "$@"
