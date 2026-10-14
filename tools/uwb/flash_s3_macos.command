#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUNNER=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)/uwb_portable.py
SERVER_HOST=${UWB_SERVER_HOST:-auto}
SERVER_PORT=${UWB_SERVER_PORT:-18765}
DEVICE_PORT=${UWB_DEVICE_PORT:-auto}
exec python3 "$RUNNER" flash-s3 --server-host "$SERVER_HOST" \
  --server-port "$SERVER_PORT" --device-port "$DEVICE_PORT" "$@"
