from __future__ import annotations

import asyncio
from dataclasses import dataclass, field
import time
from typing import Any

from fastapi import WebSocket

from .lidar import LidarScan, LidarScanAssembler, parse_lidar_chunk
from .pose import PoseEstimator
from .protocol import (
    AckEnvelope,
    FragmentEnvelope,
    FragmentReassembler,
    ProtocolError,
    build_command,
    parse_envelope,
    parse_position_message,
)
from .anchor_calibration import solve_anchor_coordinates


ALLOWED_COMMANDS = frozenset("Pp?WUSDA LR C MHOEQVT12345Z+-NBG".replace(" ", ""))
COMMAND_TIMEOUT_S = 3.0
RECENT_COMMAND_LIMIT = 32


@dataclass
class PendingCommand:
    request_id: str
    sequence: int
    command: str
    tag_id: int
    created_at: float
    transport_result: int | None = None
    gateway_queue_result: str | None = None
    status: str = "created"
    updated_at: float | None = None
    completed_at: float | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "request_id": self.request_id,
            "sequence": self.sequence,
            "command": self.command,
            "tag_id": self.tag_id,
            "created_at": self.created_at,
            "transport_result": self.transport_result,
            "gateway_queue_result": self.gateway_queue_result,
            "status": self.status,
            "updated_at": self.updated_at,
            "completed_at": self.completed_at,
        }


@dataclass
class GatewayConnection:
    gateway_id: str
    websocket: WebSocket
    connected_at: float = field(default_factory=time.time)
    send_lock: asyncio.Lock = field(default_factory=asyncio.Lock)


class VehicleRuntime:
    def __init__(self, pose_estimator: PoseEstimator) -> None:
        self.pose_estimator = pose_estimator
        self.reassembler = FragmentReassembler()
        self.gateway: GatewayConnection | None = None
        self.mobile_clients: dict[int, WebSocket] = {}
        self.pending_commands: dict[int, PendingCommand] = {}
        self.recent_commands: list[dict[str, Any]] = []
        self.latest_vehicle_lines: dict[str, str] = {}
        self.vehicle_line_received_at: dict[str, float] = {}
        self.lidar_assembler = LidarScanAssembler()
        self.latest_lidar_scan: LidarScan | None = None
        self.latest_gateway_status: dict[str, Any] = {}
        self.latest_gateway_raw_frames: list[dict[str, Any]] = []
        self.latest_pose = self.pose_estimator.estimate()
        self.protocol_errors = 0
        self.anchor_calibration: dict[str, Any] = {
            "state": "idle", "pairs_cm": {}, "result": None
        }
        self._command_sequence = 0
        self._request_sequence = 0
        self._state_lock = asyncio.Lock()

    def _complete_command(self, pending: PendingCommand, status: str, now: float | None = None) -> None:
        timestamp = time.time() if now is None else now
        pending.status = status
        pending.updated_at = timestamp
        pending.completed_at = timestamp
        self.pending_commands.pop(pending.sequence, None)
        self.recent_commands.append(pending.as_dict())
        del self.recent_commands[:-RECENT_COMMAND_LIMIT]

    def _expire_pending_commands(self, now: float | None = None) -> None:
        timestamp = time.time() if now is None else now
        expired = [
            pending
            for pending in self.pending_commands.values()
            if timestamp - pending.created_at > COMMAND_TIMEOUT_S
        ]
        for pending in expired:
            self._complete_command(pending, "timeout", timestamp)

    async def attach_gateway(self, gateway_id: str, websocket: WebSocket) -> None:
        async with self._state_lock:
            previous = self.gateway
            self.gateway = GatewayConnection(gateway_id=gateway_id, websocket=websocket)
        if previous is not None and previous.websocket is not websocket:
            await previous.websocket.close(code=1012, reason="replaced by a newer gateway session")
        await self.broadcast({"type": "gateway", "connected": True, "gateway_id": gateway_id})

    async def detach_gateway(self, websocket: WebSocket) -> None:
        gateway_id: str | None = None
        async with self._state_lock:
            if self.gateway is not None and self.gateway.websocket is websocket:
                gateway_id = self.gateway.gateway_id
                self.gateway = None
        if gateway_id is not None:
            await self.broadcast({"type": "gateway", "connected": False, "gateway_id": gateway_id})

    async def attach_mobile(self, websocket: WebSocket) -> None:
        self.mobile_clients[id(websocket)] = websocket
        await websocket.send_json({"type": "snapshot", "data": self.snapshot()})

    def detach_mobile(self, websocket: WebSocket) -> None:
        self.mobile_clients.pop(id(websocket), None)

    async def broadcast(self, event: dict[str, Any]) -> None:
        stale: list[int] = []
        for client_id, websocket in list(self.mobile_clients.items()):
            try:
                await websocket.send_json(event)
            except Exception:
                stale.append(client_id)
        for client_id in stale:
            self.mobile_clients.pop(client_id, None)

    async def _send_gateway(self, message: str) -> None:
        gateway = self.gateway
        if gateway is None:
            raise RuntimeError("UWB gateway is not connected")
        async with gateway.send_lock:
            await gateway.websocket.send_text(message)

    async def send_vehicle_command(self, command: str, tag_id: int) -> PendingCommand:
        if len(command) != 1 or command not in ALLOWED_COMMANDS:
            raise ProtocolError(f"unsupported vehicle command: {command!r}")

        async with self._state_lock:
            self._expire_pending_commands()
            for _ in range(256):
                sequence = self._command_sequence
                self._command_sequence = (self._command_sequence + 1) & 0xFF
                if sequence not in self.pending_commands:
                    break
            else:
                raise RuntimeError("all UWB command sequence numbers are pending")
            request_id = f"{self._request_sequence:08X}"
            self._request_sequence = (self._request_sequence + 1) & 0xFFFFFFFF
            pending = PendingCommand(
                request_id=request_id,
                sequence=sequence,
                command=command,
                tag_id=tag_id,
                created_at=time.time(),
                updated_at=time.time(),
            )
            self.pending_commands[sequence] = pending

        payload = build_command(sequence, command)
        try:
            await self._send_gateway(f"SEND,{request_id},{tag_id},{payload.hex().upper()}")
        except Exception:
            self.pending_commands.pop(sequence, None)
            raise
        pending.status = "sent_to_gateway"
        pending.updated_at = time.time()
        await self.broadcast({"type": "command_queued", "command": pending.as_dict()})
        return pending

    async def set_uwb_running(self, running: bool) -> None:
        await self._send_gateway("UWB_START" if running else "UWB_STOP")

    async def start_anchor_calibration(self) -> None:
        self.anchor_calibration = {"state": "requested", "pairs_cm": {}, "result": None}
        await self._send_gateway("UWB_CALIBRATE")

    async def cancel_anchor_calibration(self) -> None:
        await self._send_gateway("UWB_CALIBRATE_CANCEL")

    async def read_uwb_registers(self, address: int, count: int) -> str:
        if not 0 <= address <= 0xFFFF:
            raise ProtocolError("UWB register address is out of range")
        if not 1 <= count <= 60:
            raise ProtocolError("UWB register count is out of range")
        async with self._state_lock:
            request_id = f"{self._request_sequence:08X}"
            self._request_sequence = (self._request_sequence + 1) & 0xFFFFFFFF
        await self._send_gateway(f"UWB_READ,{request_id},{address},{count}")
        return request_id

    async def handle_gateway_message(self, message: str) -> None:
        try:
            if message.startswith("POS,"):
                report = parse_position_message(message)
                self.latest_pose = self.pose_estimator.update(report)
                display_report = self.pose_estimator.latest_report(report.tag_id) or report
                await self.broadcast(
                    {
                        "type": "uwb_position",
                        "tag": {
                            "id": display_report.tag_id,
                            "flags": f"{display_report.flags:08X}",
                            "position_valid": display_report.position_valid,
                            "x_cm": display_report.x_cm,
                            "y_cm": display_report.y_cm,
                            "z_cm": display_report.z_cm,
                            "distances_cm": list(display_report.distances_cm),
                        },
                        "pose": self.latest_pose.as_dict(),
                    }
                )
                return

            if message.startswith("PASS,"):
                await self._handle_pass(message)
                return

            if message.startswith("STATUS,"):
                fields = message.split(",")
                if len(fields) not in (7, 13, 16):
                    raise ProtocolError("invalid STATUS message")
                status = {
                    "uptime_ms": int(fields[1]),
                    "rssi": int(fields[2]),
                    "good_frames": int(fields[3]),
                    "crc_errors": int(fields[4]),
                    "uwb_tx": int(fields[5]),
                    "queue_drops": int(fields[6]),
                    "received_at": time.time(),
                }
                if len(fields) == 13:
                    status.update(
                        {
                            "rx_bytes": int(fields[7]),
                            "framing_errors": int(fields[8]),
                            "position_frames": int(fields[9]),
                            "pass_frames": int(fields[10]),
                            "raw_frames": int(fields[11]),
                            "rx_idle_ms": int(fields[12]),
                        }
                    )
                elif len(fields) == 16:
                    status.update(
                        {
                            "rx_bytes": int(fields[7]),
                            "framing_errors": int(fields[8]),
                            "position_frames": int(fields[9]),
                            "pass_frames": int(fields[10]),
                            "raw_frames": int(fields[11]),
                            "rx_idle_ms": int(fields[12]),
                            "positioning_wanted": bool(int(fields[13])),
                            "position_idle_ms": int(fields[14]),
                            "position_start_requests": int(fields[15]),
                        }
                    )
                self.latest_gateway_status.update(status)
                await self.broadcast({"type": "gateway_status", "data": self.latest_gateway_status})
                return

            if message.startswith("HELLO,"):
                fields = message.split(",")
                if len(fields) >= 4:
                    self.latest_gateway_status.update(
                        {"protocol": fields[1], "gateway_id": fields[2], "firmware": fields[3]}
                    )
                return

            if message.startswith("RAW,"):
                fields = message.split(",", 2)
                if len(fields) != 3:
                    raise ProtocolError("invalid RAW message")
                try:
                    frame = bytes.fromhex(fields[2])
                except ValueError as exc:
                    raise ProtocolError("RAW frame is not hexadecimal") from exc
                item = {
                    "gateway_sequence": int(fields[1]),
                    "frame_hex": frame.hex().upper(),
                    "received_at": time.time(),
                }
                self.latest_gateway_raw_frames.append(item)
                del self.latest_gateway_raw_frames[:-16]
                await self.broadcast({"type": "uwb_raw", "data": item})
                return

            if message.startswith("SENDQ,"):
                fields = message.split(",", 2)
                if len(fields) != 3:
                    raise ProtocolError("invalid SENDQ message")
                request_id, result = fields[1], fields[2]
                pending = next(
                    (item for item in self.pending_commands.values() if item.request_id == request_id),
                    None,
                )
                if pending is not None:
                    pending.gateway_queue_result = result
                    pending.updated_at = time.time()
                    if result == "0":
                        pending.status = "gateway_queued"
                    else:
                        self._complete_command(pending, "gateway_rejected")
                await self.broadcast(
                    {
                        "type": "gateway_queue_result",
                        "request_id": request_id,
                        "result": result,
                        "command": None if pending is None else pending.as_dict(),
                    }
                )
                return

            if message.startswith("PONG,"):
                await self.broadcast({"type": "gateway_message", "line": message})
                return

            if message.startswith("CALSTATE,"):
                fields = message.split(",", 2)
                if len(fields) != 3:
                    raise ProtocolError("invalid calibration state")
                self.anchor_calibration["state"] = fields[1].lower()
                self.anchor_calibration["detail"] = fields[2]
                await self.broadcast({"type": "anchor_calibration", "data": self.anchor_calibration})
                return

            if message.startswith("CALPAIR,"):
                fields = message.split(",")
                if len(fields) != 6:
                    raise ProtocolError("invalid calibration pair")
                anchor_names = {255: "A", 240: "B", 241: "C", 242: "D"}
                active, passive = int(fields[2]), int(fields[3])
                if active not in anchor_names or passive not in anchor_names:
                    raise ProtocolError("calibration pair uses unsupported anchor")
                pair = anchor_names[active] + anchor_names[passive]
                success, distance = bool(int(fields[4])), int(fields[5])
                if not success or distance <= 0:
                    self.anchor_calibration["state"] = "failed"
                    self.anchor_calibration["detail"] = f"{pair} measurement failed"
                else:
                    self.anchor_calibration.setdefault("pairs_cm", {})[pair] = distance
                    self.anchor_calibration["state"] = "measuring"
                    if len(self.anchor_calibration["pairs_cm"]) == 6:
                        self.anchor_calibration["result"] = solve_anchor_coordinates(
                            self.anchor_calibration["pairs_cm"]
                        )
                await self.broadcast({"type": "anchor_calibration", "data": self.anchor_calibration})
                return

            raise ProtocolError("unknown gateway message")
        except (ProtocolError, ValueError):
            self.protocol_errors += 1
            raise

    async def _handle_pass(self, message: str) -> None:
        fields = message.split(",", 3)
        if len(fields) != 4:
            raise ProtocolError("invalid PASS message")
        gateway_sequence = int(fields[1])
        sender_id = int(fields[2])
        try:
            payload = bytes.fromhex(fields[3])
        except ValueError as exc:
            raise ProtocolError("PASS payload is not hexadecimal") from exc

        envelope = parse_envelope(payload)
        if isinstance(envelope, AckEnvelope):
            pending = self.pending_commands.get(envelope.sequence)
            if pending is not None and pending.command == envelope.command:
                pending.transport_result = envelope.result
                ack_status = {
                    0: "vehicle_accepted",
                    1: "vehicle_duplicate",
                    2: "vehicle_queue_rejected",
                    3: "vehicle_injection_failed",
                }[envelope.result]
                self._complete_command(pending, ack_status)
            await self.broadcast(
                {
                    "type": "uwb_command_ack",
                    "gateway_sequence": gateway_sequence,
                    "sender_id": sender_id,
                    "sequence": envelope.sequence,
                    "command": envelope.command,
                    "result": envelope.result,
                }
            )
            return

        if isinstance(envelope, FragmentEnvelope):
            line = self.reassembler.add(sender_id, envelope)
            if line is None:
                return
            try:
                decoded = line.decode("ascii")
            except UnicodeDecodeError as exc:
                raise ProtocolError("vehicle telemetry is not ASCII") from exc
            if not decoded.startswith("@"):
                raise ProtocolError("vehicle telemetry does not start with @")
            prefix = decoded[:2]
            received_at = time.time()
            complete_lidar_scan: LidarScan | None = None
            if prefix == "@R":
                complete_lidar_scan = self.lidar_assembler.add(
                    sender_id, parse_lidar_chunk(decoded), received_at
                )
                if complete_lidar_scan is not None:
                    self.latest_lidar_scan = complete_lidar_scan
            self.latest_vehicle_lines[prefix] = decoded
            self.vehicle_line_received_at[prefix] = received_at
            await self.broadcast(
                {
                    "type": "vehicle_uart",
                    "sender_id": sender_id,
                    "line": decoded,
                }
            )
            if complete_lidar_scan is not None:
                await self.broadcast(
                    {
                        "type": "lidar_scan",
                        "sender_id": sender_id,
                        "scan": complete_lidar_scan.as_dict(),
                    }
                )
            return

        raise ProtocolError("server received a command envelope from vehicle")

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        self._expire_pending_commands(now)
        self.latest_pose = self.pose_estimator.estimate(now)
        gateway_status = dict(self.latest_gateway_status)
        status_received_at = gateway_status.get("received_at")
        gateway_status_age_ms = (
            None if status_received_at is None else max(0.0, (now - float(status_received_at)) * 1000.0)
        )
        vehicle_line_age_ms = {
            prefix: max(0.0, (now - received_at) * 1000.0)
            for prefix, received_at in self.vehicle_line_received_at.items()
        }
        lidar_scan_age_ms = (
            None
            if self.latest_lidar_scan is None
            else max(0.0, (now - self.latest_lidar_scan.received_at) * 1000.0)
        )
        return {
            "gateway_connected": self.gateway is not None,
            "gateway_id": None if self.gateway is None else self.gateway.gateway_id,
            "gateway_status": gateway_status,
            "gateway_status_age_ms": gateway_status_age_ms,
            "gateway_data_fresh": gateway_status_age_ms is not None and gateway_status_age_ms <= 3000.0,
            "gateway_raw_frames": self.latest_gateway_raw_frames,
            "tag_fixes": self.pose_estimator.fixes_as_dict(now),
            "pose": self.latest_pose.as_dict(),
            "vehicle_lines": self.latest_vehicle_lines,
            "vehicle_line_age_ms": vehicle_line_age_ms,
            "lidar_scan": None if self.latest_lidar_scan is None else self.latest_lidar_scan.as_dict(),
            "lidar_scan_age_ms": lidar_scan_age_ms,
            "lidar_scan_fresh": lidar_scan_age_ms is not None and lidar_scan_age_ms <= 1500.0,
            "pending_commands": [item.as_dict() for item in self.pending_commands.values()],
            "recent_commands": self.recent_commands,
            "protocol_errors": self.protocol_errors,
            "anchor_calibration": self.anchor_calibration,
        }
