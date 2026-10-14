from __future__ import annotations

import hmac
from typing import Any, Optional

from fastapi import FastAPI, Header, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

from .config import Settings
from .pose import PoseEstimator
from .protocol import ProtocolError
from .runtime import VehicleRuntime


settings = Settings.from_env()
runtime = VehicleRuntime(PoseEstimator(settings.pose))
app = FastAPI(title="Little Car UWB Gateway", version="0.1.0")


class CommandRequest(BaseModel):
    command: str = Field(min_length=1, max_length=1)
    tag_id: Optional[int] = Field(default=None, ge=0, le=99)


class UwbRunRequest(BaseModel):
    running: bool


class UwbReadRequest(BaseModel):
    address: int = Field(ge=0, le=0xFFFF)
    count: int = Field(ge=1, le=60)


def _valid_token(actual: str, expected: str) -> bool:
    return bool(actual) and hmac.compare_digest(actual, expected)


def _require_mobile_token(token: Optional[str]) -> None:
    if token is None or not _valid_token(token, settings.mobile_token):
        raise HTTPException(status_code=401, detail="invalid mobile token")


@app.get("/health")
async def health() -> dict[str, Any]:
    return {"ok": True, "gateway_connected": runtime.gateway is not None}


@app.get("/api/v1/state")
async def state(x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    return runtime.snapshot()


@app.post("/api/v1/command")
async def command(request: CommandRequest, x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    tag_id = settings.command_tag_id if request.tag_id is None else request.tag_id
    try:
        pending = await runtime.send_vehicle_command(request.command, tag_id)
    except ProtocolError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"accepted": True, "command": pending.as_dict()}


@app.post("/api/v1/uwb/run")
async def uwb_run(request: UwbRunRequest, x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    try:
        await runtime.set_uwb_running(request.running)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"accepted": True, "running": request.running}


@app.post("/api/v1/uwb/read")
async def uwb_read(request: UwbReadRequest, x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    try:
        request_id = await runtime.read_uwb_registers(request.address, request.count)
    except ProtocolError as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"accepted": True, "request_id": request_id}


@app.post("/api/v1/uwb/calibrate")
async def uwb_calibrate(x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    try:
        await runtime.start_anchor_calibration()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"accepted": True}


@app.post("/api/v1/uwb/calibrate/cancel")
async def uwb_calibrate_cancel(x_api_token: Optional[str] = Header(default=None)) -> dict[str, Any]:
    _require_mobile_token(x_api_token)
    try:
        await runtime.cancel_anchor_calibration()
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc
    return {"accepted": True}


@app.websocket("/ws/gateway/{gateway_id}")
async def gateway_socket(websocket: WebSocket, gateway_id: str, token: str = Query(default="")) -> None:
    if not _valid_token(token, settings.gateway_token):
        await websocket.close(code=1008, reason="invalid gateway token")
        return
    await websocket.accept()
    await runtime.attach_gateway(gateway_id, websocket)
    try:
        while True:
            message = await websocket.receive_text()
            try:
                await runtime.handle_gateway_message(message)
            except (ProtocolError, ValueError):
                await websocket.send_text("ERROR,PROTOCOL")
    except WebSocketDisconnect:
        pass
    finally:
        await runtime.detach_gateway(websocket)


@app.websocket("/ws/mobile")
async def mobile_socket(websocket: WebSocket, token: str = Query(default="")) -> None:
    if not _valid_token(token, settings.mobile_token):
        await websocket.close(code=1008, reason="invalid mobile token")
        return
    await websocket.accept()
    await runtime.attach_mobile(websocket)
    try:
        while True:
            message = await websocket.receive_json()
            try:
                message_type = message.get("type")
                if message_type == "command":
                    tag_id = int(message.get("tag_id", settings.command_tag_id))
                    pending = await runtime.send_vehicle_command(str(message.get("command", "")), tag_id)
                    await websocket.send_json({"type": "command_accepted", "command": pending.as_dict()})
                elif message_type == "uwb_run":
                    running = bool(message.get("running"))
                    await runtime.set_uwb_running(running)
                    await websocket.send_json({"type": "uwb_run_accepted", "running": running})
                elif message_type == "uwb_read":
                    address = int(message.get("address"))
                    count = int(message.get("count"))
                    request_id = await runtime.read_uwb_registers(address, count)
                    await websocket.send_json({"type": "uwb_read_accepted", "request_id": request_id})
                elif message_type == "anchor_calibrate":
                    await runtime.start_anchor_calibration()
                    await websocket.send_json({"type": "anchor_calibration_accepted"})
                elif message_type == "anchor_calibrate_cancel":
                    await runtime.cancel_anchor_calibration()
                elif message_type == "ping":
                    await websocket.send_json({"type": "pong", "nonce": message.get("nonce")})
                else:
                    await websocket.send_json({"type": "error", "detail": "unknown message type"})
            except (ProtocolError, ValueError) as exc:
                await websocket.send_json({"type": "error", "detail": str(exc)})
            except RuntimeError as exc:
                await websocket.send_json({"type": "unavailable", "detail": str(exc)})
    except WebSocketDisconnect:
        pass
    finally:
        runtime.detach_mobile(websocket)
