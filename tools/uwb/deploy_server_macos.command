#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUNNER=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)/uwb_portable.py
SERVER_HOST=${UWB_SERVER_HOST:-auto}
SERVER_PORT=${UWB_SERVER_PORT:-18765}
exec python3 "$RUNNER" server-start --server-host "$SERVER_HOST" --port "$SERVER_PORT" "$@"
