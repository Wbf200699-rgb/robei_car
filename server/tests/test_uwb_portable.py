from __future__ import annotations

import importlib.util
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "uwb_portable", ROOT / "tools" / "uwb_portable.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_parse_env_file(tmp_path: Path) -> None:
    path = tmp_path / ".env"
    path.write_text("# comment\nA=one\nB='two'\nC=\"three\"\n", encoding="utf-8")
    assert MODULE.parse_env_file(path) == {"A": "one", "B": "two", "C": "three"}


def test_render_gateway_secrets_contains_only_requested_endpoint() -> None:
    rendered = MODULE.render_gateway_secrets(
        ssid="test-net", password="test-pass", host="192.168.50.20",
        port=18765, use_tls=False, token="test-token",
    )
    assert 'SERVER_HOST[] = "192.168.50.20"' in rendered
    assert "SERVER_PORT = 18765" in rendered
    assert "SERVER_USE_TLS = false" in rendered


def test_parse_secrets_header_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "gateway_secrets.h"
    path.write_text(
        MODULE.render_gateway_secrets(
            ssid="ssid", password="password", host="10.0.0.2",
            port=18765, use_tls=False, token="token",
        ),
        encoding="utf-8",
    )
    parsed = MODULE.parse_secrets_header(path)
    assert parsed["WIFI_SSID"] == "ssid"
    assert parsed["SERVER_HOST"] == "10.0.0.2"
    assert parsed["SERVER_PORT"] == "18765"
    assert parsed["SERVER_USE_TLS"] == "false"


def test_cpp_string_escapes_quotes_and_backslashes() -> None:
    assert MODULE.cpp_string('a\\b"c') == 'a\\\\b\\"c'


def test_arduino_cli_asset_for_apple_silicon() -> None:
    with patch.object(MODULE.platform, "system", return_value="Darwin"), \
         patch.object(MODULE.platform, "machine", return_value="arm64"):
        asset, digest = MODULE.arduino_cli_asset()
    assert asset == "arduino-cli_1.5.1_macOS_ARM64.tar.gz"
    assert len(digest) == 64


def test_arduino_cli_asset_for_windows_x64() -> None:
    with patch.object(MODULE.platform, "system", return_value="Windows"), \
         patch.object(MODULE.platform, "machine", return_value="AMD64"):
        asset, digest = MODULE.arduino_cli_asset()
    assert asset == "arduino-cli_1.5.1_Windows_64bit.zip"
    assert len(digest) == 64
