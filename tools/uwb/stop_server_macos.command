#!/bin/sh
set -eu
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RUNNER=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)/uwb_portable.py
exec python3 "$RUNNER" server-stop "$@"
