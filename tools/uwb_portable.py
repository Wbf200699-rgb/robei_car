#!/usr/bin/env python3
"""Portable UWB relay deployment and ESP32-S3 gateway flashing.

Secrets are loaded into process memory from environment variables, the ignored
server/.env, or the ignored gateway_secrets.h. The flash command writes a
temporary gateway_secrets.h only for the duration of the build and restores the
previous file in a finally block.
"""

from __future__ import annotations

import argparse
import getpass
import hashlib
import json
import os
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.request
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SERVER_DIR = ROOT / "server"
GATEWAY_DIR = ROOT / "esp32" / "uwb_gateway"
RUNTIME_DIR = SERVER_DIR / "data" / "runtime"
PID_FILE = RUNTIME_DIR / "portable_server.pid"
SERVER_LOG = RUNTIME_DIR / "portable_server.log"
SECRETS_HEADER = GATEWAY_DIR / "gateway_secrets.h"
ARDUINO_CLI_VERSION = "1.5.1"
ARDUINO_CLI_DIR = ROOT / "tools" / ".cache" / "arduino-cli"
ARDUINO_CLI_ASSETS = {
    ("Windows", "AMD64"): (
        "arduino-cli_1.5.1_Windows_64bit.zip",
        "fabe42e0eb04d00e776a66178299ff95a46c623dbc260f997e58fd514853dd40",
    ),
    ("Darwin", "x86_64"): (
        "arduino-cli_1.5.1_macOS_64bit.tar.gz",
        "c982e940027996bea9901050e95fae99c59c1dcfee54beedecaf28141e7bf2e7",
    ),
    ("Darwin", "arm64"): (
        "arduino-cli_1.5.1_macOS_ARM64.tar.gz",
        "cb952e8c1621c95ef5f1d17831c945e3d0ec5973f89c557a7ec8feb9c4f7d4c9",
    ),
}
FQBN = (
    "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,"
    "PSRAM=opi,FlashMode=qio,USBMode=hwcdc,CDCOnBoot=default,UploadMode=default"
)
PLACEHOLDERS = {
    "", "CHANGE_ME", "change-me-gateway", "change-me-mobile",
    "your-wifi-name", "your-wifi-password",
}


class PortableError(RuntimeError):
    pass


def parse_env_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        name = name.strip()
        value = value.strip().strip('"').strip("'")
        if name:
            values[name] = value
    return values


def parse_secrets_header(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    text = path.read_text(encoding="utf-8", errors="strict")
    for name, value in re.findall(
        r'constexpr\s+char\s+([A-Z][A-Z0-9_]*)\[\]\s*=\s*"((?:\\.|[^"\\])*)"\s*;',
        text,
    ):
        values[name] = json.loads(f'"{value}"')
    for name, value in re.findall(
        r"constexpr\s+uint16_t\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;",
        text,
    ):
        values[name] = value
    for name, value in re.findall(
        r"constexpr\s+bool\s+([A-Z][A-Z0-9_]*)\s*=\s*(true|false)\s*;",
        text,
    ):
        values[name] = value
    return values


def merged_runtime_env() -> dict[str, str]:
    values = parse_env_file(SERVER_DIR / ".env")
    values.update({key: value for key, value in os.environ.items() if value})
    return values


def required_secret(values: dict[str, str], name: str, prompt: str,
                    *, hidden: bool = True) -> str:
    value = values.get(name, "").strip()
    if value in PLACEHOLDERS and sys.stdin.isatty():
        value = (getpass.getpass(prompt) if hidden else input(prompt)).strip()
    if value in PLACEHOLDERS:
        raise PortableError(
            f"missing {name}; set it as an environment variable before running"
        )
    return value


def detect_lan_ip() -> str:
    candidates: list[str] = []
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("8.8.8.8", 80))
            candidates.append(probe.getsockname()[0])
    except OSError:
        pass
    try:
        candidates.extend(socket.gethostbyname_ex(socket.gethostname())[2])
    except OSError:
        pass
    for address in candidates:
        if address and not address.startswith("127.") and not address.startswith("169.254."):
            return address
    raise PortableError("cannot detect a LAN IPv4 address; pass --server-host explicitly")


def endpoint_host(requested: str) -> str:
    return detect_lan_ip() if requested.lower() == "auto" else requested.strip()


def health_ok(port: int) -> bool:
    try:
        with urllib.request.urlopen(
            f"http://127.0.0.1:{port}/health", timeout=1.0
        ) as response:
            return response.status == 200
    except (OSError, urllib.error.URLError):
        return False


def venv_python() -> Path:
    if os.name == "nt":
        return SERVER_DIR / ".venv" / "Scripts" / "python.exe"
    return SERVER_DIR / ".venv" / "bin" / "python"


def ensure_server_environment(skip_install: bool) -> Path:
    python = venv_python()
    if not python.is_file():
        subprocess.run(
            [sys.executable, "-m", "venv", str(SERVER_DIR / ".venv")],
            check=True,
        )
    probe = subprocess.run(
        [str(python), "-c", "import fastapi,uvicorn"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if probe.returncode != 0:
        if skip_install:
            raise PortableError("server dependencies missing and --skip-install was used")
        subprocess.run(
            [str(python), "-m", "pip", "install", "-r",
             str(SERVER_DIR / "requirements.txt")],
            check=True,
        )
    return python


def read_pid() -> int | None:
    try:
        return int(PID_FILE.read_text(encoding="ascii").strip())
    except (OSError, ValueError):
        return None


def process_exists(pid: int) -> bool:
    if os.name == "nt":
        import ctypes

        process_query_limited_information = 0x1000
        handle = ctypes.windll.kernel32.OpenProcess(
            process_query_limited_information, False, pid
        )
        if not handle:
            return False
        ctypes.windll.kernel32.CloseHandle(handle)
        return True
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def server_start(args: argparse.Namespace) -> None:
    if not 1 <= args.port <= 65535:
        raise PortableError("--port must be between 1 and 65535")
    host = endpoint_host(args.server_host)
    values = merged_runtime_env()
    gateway_token = required_secret(
        values, "UWB_GATEWAY_TOKEN", "UWB gateway token: "
    )
    mobile_token = required_secret(
        values, "UWB_MOBILE_TOKEN", "UWB mobile token: "
    )
    if health_ok(args.port):
        print(f"UWB relay already healthy on port {args.port}.")
        print(f"Mobile endpoint: ws://{host}:{args.port}/ws/mobile")
        print(f"S3 endpoint: {host}:{args.port} (TLS off)")
        return

    old_pid = read_pid()
    if old_pid and process_exists(old_pid):
        raise PortableError(
            f"portable server PID {old_pid} exists but health check failed; run server-stop"
        )
    python = ensure_server_environment(args.skip_install)
    runtime_env = os.environ.copy()
    runtime_env.update(values)
    runtime_env["UWB_GATEWAY_TOKEN"] = gateway_token
    runtime_env["UWB_MOBILE_TOKEN"] = mobile_token
    command = [
        str(python), "-m", "uvicorn", "app.main:app",
        "--host", "0.0.0.0", "--port", str(args.port),
        "--no-access-log", "--log-level", "warning",
    ]
    if args.foreground:
        print(f"Mobile endpoint: ws://{host}:{args.port}/ws/mobile")
        print("Press Ctrl+C to stop the relay.")
        completed = subprocess.run(command, cwd=SERVER_DIR, env=runtime_env)
        if completed.returncode:
            raise PortableError(f"server exited with code {completed.returncode}")
        return

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    log_stream = SERVER_LOG.open("ab", buffering=0)
    kwargs: dict[str, object] = {
        "cwd": SERVER_DIR,
        "env": runtime_env,
        "stdin": subprocess.DEVNULL,
        "stdout": log_stream,
        "stderr": subprocess.STDOUT,
        "close_fds": True,
    }
    if os.name == "nt":
        kwargs["creationflags"] = 0x00000008 | 0x00000200
    else:
        kwargs["start_new_session"] = True
    process = subprocess.Popen(command, **kwargs)
    log_stream.close()
    PID_FILE.write_text(str(process.pid), encoding="ascii")
    for _ in range(60):
        if health_ok(args.port):
            print(f"UWB relay started (PID {process.pid}).")
            print(f"Mobile endpoint: ws://{host}:{args.port}/ws/mobile")
            print(f"S3 endpoint: {host}:{args.port} (TLS off)")
            print(f"Local health: http://127.0.0.1:{args.port}/health")
            print(f"Log: {SERVER_LOG}")
            return
        if process.poll() is not None:
            break
        time.sleep(0.25)
    if process.poll() is None:
        process.terminate()
    PID_FILE.unlink(missing_ok=True)
    raise PortableError(f"server failed health check; inspect {SERVER_LOG}")


def server_stop(_: argparse.Namespace) -> None:
    pid = read_pid()
    if not pid:
        print("No portable UWB server PID file exists.")
        return
    if process_exists(pid):
        os.kill(pid, signal.SIGTERM)
        for _ in range(40):
            if not process_exists(pid):
                break
            time.sleep(0.1)
    PID_FILE.unlink(missing_ok=True)
    print(f"Portable UWB server stop requested for PID {pid}.")


def server_status(args: argparse.Namespace) -> None:
    pid = read_pid()
    print(f"PID: {pid if pid else 'none'}")
    print(f"Process: {'running' if pid and process_exists(pid) else 'stopped'}")
    print(f"Health: {'OK' if health_ok(args.port) else 'unavailable'}")


def cpp_string(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def render_gateway_secrets(*, ssid: str, password: str, host: str, port: int,
                           use_tls: bool, token: str) -> str:
    return (
        "#pragma once\n\n"
        f'constexpr char WIFI_SSID[] = "{cpp_string(ssid)}";\n'
        f'constexpr char WIFI_PASSWORD[] = "{cpp_string(password)}";\n'
        f'constexpr char SERVER_HOST[] = "{cpp_string(host)}";\n'
        f"constexpr uint16_t SERVER_PORT = {port};\n"
        f"constexpr bool SERVER_USE_TLS = {'true' if use_tls else 'false'};\n"
        f'constexpr char SERVER_GATEWAY_TOKEN[] = "{cpp_string(token)}";\n'
    )


def arduino_cli_asset() -> tuple[str, str]:
    key = (platform.system(), platform.machine())
    if key not in ARDUINO_CLI_ASSETS:
        raise PortableError(
            f"automatic arduino-cli install does not support {key[0]} {key[1]}"
        )
    return ARDUINO_CLI_ASSETS[key]


def install_arduino_cli() -> Path:
    asset, expected_sha256 = arduino_cli_asset()
    executable = ARDUINO_CLI_DIR / (
        "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    )
    url = (
        "https://github.com/arduino/arduino-cli/releases/download/"
        f"v{ARDUINO_CLI_VERSION}/{asset}"
    )
    ARDUINO_CLI_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Installing official Arduino CLI {ARDUINO_CLI_VERSION} locally...")
    with tempfile.TemporaryDirectory(prefix="arduino-cli-") as temporary:
        archive = Path(temporary) / asset
        urllib.request.urlretrieve(url, archive)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        if digest != expected_sha256:
            raise PortableError("arduino-cli archive SHA-256 verification failed")
        member_name = "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
        if asset.endswith(".zip"):
            with zipfile.ZipFile(archive) as bundle:
                source = next(
                    (name for name in bundle.namelist()
                     if Path(name).name == member_name), None
                )
                if source is None:
                    raise PortableError("arduino-cli executable missing from archive")
                with bundle.open(source) as input_stream, executable.open("wb") as output:
                    shutil.copyfileobj(input_stream, output)
        else:
            with tarfile.open(archive, "r:gz") as bundle:
                source = next(
                    (member for member in bundle.getmembers()
                     if Path(member.name).name == member_name and member.isfile()), None
                )
                if source is None:
                    raise PortableError("arduino-cli executable missing from archive")
                input_stream = bundle.extractfile(source)
                if input_stream is None:
                    raise PortableError("cannot read arduino-cli executable from archive")
                with input_stream, executable.open("wb") as output:
                    shutil.copyfileobj(input_stream, output)
    if os.name != "nt":
        executable.chmod(0o755)
    return executable


def find_arduino_cli(skip_install: bool) -> str:
    executable = shutil.which("arduino-cli")
    if executable:
        return executable
    cached = ARDUINO_CLI_DIR / (
        "arduino-cli.exe" if os.name == "nt" else "arduino-cli"
    )
    if cached.is_file():
        return str(cached)
    if skip_install:
        raise PortableError(
            "arduino-cli is missing and --skip-install was used"
        )
    return str(install_arduino_cli())


def ensure_arduino_dependencies(cli: str, skip_install: bool) -> None:
    core = subprocess.run(
        [cli, "core", "list"], capture_output=True, text=True, check=True
    ).stdout
    if "esp32:esp32" not in core:
        if skip_install:
            raise PortableError("Arduino ESP32 core missing and --skip-install was used")
        subprocess.run([cli, "core", "update-index"], check=True)
        subprocess.run([cli, "core", "install", "esp32:esp32"], check=True)
    libraries = subprocess.run(
        [cli, "lib", "list"], capture_output=True, text=True, check=True
    ).stdout
    if not re.search(r"(?mi)^WebSockets\s+", libraries):
        if skip_install:
            raise PortableError("Arduino WebSockets library missing and --skip-install was used")
        subprocess.run([cli, "lib", "install", "WebSockets@2.7.2"], check=True)


def detect_esp32_port(cli: str) -> str:
    result = subprocess.run(
        [cli, "board", "list", "--format", "json"],
        capture_output=True, text=True, check=True,
    )
    candidates: list[str] = []
    for detected in json.loads(result.stdout).get("detected_ports", []):
        boards = detected.get("matching_boards", [])
        port = detected.get("port", {}).get("address", "")
        properties = detected.get("port", {}).get("properties", {})
        is_esp32 = any(
            board.get("fqbn", "").startswith("esp32:esp32:") for board in boards
        ) or str(properties.get("vid", "")).lower() == "0x303a"
        if port and is_esp32:
            candidates.append(port)
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise PortableError("no ESP32-S3 serial port detected; pass --device-port")
    raise PortableError(
        "multiple ESP32 ports detected; pass --device-port (candidates: "
        + ", ".join(candidates) + ")"
    )


def flash_s3(args: argparse.Namespace) -> None:
    if not 1 <= args.server_port <= 65535:
        raise PortableError("--server-port must be between 1 and 65535")
    host = endpoint_host(args.server_host)
    runtime = merged_runtime_env()
    existing = parse_secrets_header(SECRETS_HEADER)
    values = {**existing, **runtime}
    values.setdefault("UWB_WIFI_SSID", values.get("WIFI_SSID", ""))
    values.setdefault("UWB_WIFI_PASSWORD", values.get("WIFI_PASSWORD", ""))
    values.setdefault(
        "UWB_GATEWAY_TOKEN", values.get("SERVER_GATEWAY_TOKEN", "")
    )
    ssid = required_secret(values, "UWB_WIFI_SSID", "Wi-Fi SSID: ", hidden=False)
    password_value = values.get("UWB_WIFI_PASSWORD", "")
    if password_value in PLACEHOLDERS:
        password_value = required_secret(
            values, "UWB_WIFI_PASSWORD", "Wi-Fi password: "
        )
    token = required_secret(
        values, "UWB_GATEWAY_TOKEN", "UWB gateway token: "
    )
    cli = find_arduino_cli(args.skip_install)
    ensure_arduino_dependencies(cli, args.skip_install)
    device_port = None
    if not args.compile_only:
        device_port = (
            detect_esp32_port(cli)
            if args.device_port.lower() == "auto"
            else args.device_port
        )

    original = SECRETS_HEADER.read_bytes() if SECRETS_HEADER.exists() else None
    build_path = GATEWAY_DIR / "build_portable"
    build_path.mkdir(parents=True, exist_ok=True)
    try:
        SECRETS_HEADER.write_text(
            render_gateway_secrets(
                ssid=ssid, password=password_value, host=host,
                port=args.server_port, use_tls=args.tls, token=token,
            ),
            encoding="utf-8", newline="\n",
        )
        print(f"Compiling ESP32-S3 gateway for {host}:{args.server_port}...")
        subprocess.run(
            [cli, "compile", "--clean", "--fqbn", FQBN,
             "--build-path", str(build_path), str(GATEWAY_DIR)],
            check=True,
        )
        if args.compile_only:
            print(f"Compile completed. Build path: {build_path}")
            return
        print(f"Uploading ESP32-S3 gateway on {device_port}...")
        subprocess.run(
            [cli, "upload", "--port", str(device_port), "--fqbn", FQBN,
             "--build-path", str(build_path), str(GATEWAY_DIR)],
            check=True,
        )
        print(f"S3 upload completed on {device_port}.")
        scheme = "wss" if args.tls else "ws"
        print(f"Gateway target: {scheme}://{host}:{args.server_port}/ws/gateway/anchor-a")
    finally:
        if original is None:
            SECRETS_HEADER.unlink(missing_ok=True)
        else:
            SECRETS_HEADER.write_bytes(original)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    start = sub.add_parser("server-start", help="deploy the local relay")
    start.add_argument("--server-host", default="auto",
                       help="LAN IPv4 advertised to phone/S3, or auto")
    start.add_argument("--port", type=int, default=18765)
    start.add_argument("--foreground", action="store_true")
    start.add_argument("--skip-install", action="store_true")
    start.set_defaults(function=server_start)

    stop = sub.add_parser("server-stop", help="stop the background relay")
    stop.set_defaults(function=server_stop)

    status = sub.add_parser("server-status", help="check relay process/health")
    status.add_argument("--port", type=int, default=18765)
    status.set_defaults(function=server_status)

    flash = sub.add_parser("flash-s3", help="compile and upload the S3 gateway")
    flash.add_argument("--server-host", default="auto")
    flash.add_argument("--server-port", type=int, default=18765)
    flash.add_argument("--device-port", default="auto")
    flash.add_argument("--tls", action="store_true")
    flash.add_argument("--compile-only", action="store_true")
    flash.add_argument("--skip-install", action="store_true")
    flash.set_defaults(function=flash_s3)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        args.function(args)
    except (PortableError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
