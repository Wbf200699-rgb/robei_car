import asyncio

from app.pose import PoseEstimator
from app.protocol import build_ack
from app.runtime import VehicleRuntime


class FakeGatewaySocket:
    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send_text(self, message: str) -> None:
        self.sent.append(message)

    async def close(self, code: int, reason: str) -> None:
        return None


def test_status_preserves_hello_metadata() -> None:
    runtime = VehicleRuntime(PoseEstimator())

    async def exercise() -> None:
        await runtime.handle_gateway_message("HELLO,1,anchor-a,0.1.0")
        await runtime.handle_gateway_message("STATUS,1234,-42,9,1,7,0")

    asyncio.run(exercise())
    assert runtime.latest_gateway_status["protocol"] == "1"
    assert runtime.latest_gateway_status["gateway_id"] == "anchor-a"
    assert runtime.latest_gateway_status["firmware"] == "0.1.0"
    assert runtime.latest_gateway_status["good_frames"] == 9
    assert runtime.latest_gateway_status["queue_drops"] == 0


def test_extended_status_and_raw_frame_are_preserved() -> None:
    runtime = VehicleRuntime(PoseEstimator())

    async def exercise() -> None:
        await runtime.handle_gateway_message("STATUS,1234,-42,9,1,7,0,200,3,4,5,6,70")
        await runtime.handle_gateway_message("RAW,18,0103021234B533")

    asyncio.run(exercise())
    assert runtime.latest_gateway_status["rx_bytes"] == 200
    assert runtime.latest_gateway_status["framing_errors"] == 3
    assert runtime.latest_gateway_status["position_frames"] == 4
    assert runtime.latest_gateway_status["rx_idle_ms"] == 70
    assert runtime.latest_gateway_raw_frames[-1]["gateway_sequence"] == 18
    assert runtime.latest_gateway_raw_frames[-1]["frame_hex"] == "0103021234B533"


def test_watchdog_status_fields_are_preserved() -> None:
    runtime = VehicleRuntime(PoseEstimator())

    async def exercise() -> None:
        await runtime.handle_gateway_message(
            "STATUS,1234,-42,9,1,7,0,200,3,4,5,6,70,1,80,2"
        )

    asyncio.run(exercise())
    assert runtime.latest_gateway_status["positioning_wanted"] is True
    assert runtime.latest_gateway_status["position_idle_ms"] == 80
    assert runtime.latest_gateway_status["position_start_requests"] == 2


def test_command_tracks_gateway_queue_and_vehicle_ack() -> None:
    runtime = VehicleRuntime(PoseEstimator())
    gateway = FakeGatewaySocket()

    async def exercise() -> None:
        await runtime.attach_gateway("anchor-a", gateway)
        pending = await runtime.send_vehicle_command("W", 1)
        await runtime.handle_gateway_message(f"SENDQ,{pending.request_id},0")
        assert pending.status == "gateway_queued"
        ack = build_ack(pending.sequence, "W", 0).hex().upper()
        await runtime.handle_gateway_message(f"PASS,1,1,{ack}")

    asyncio.run(exercise())
    assert not runtime.pending_commands
    assert runtime.recent_commands[-1]["status"] == "vehicle_accepted"
    assert runtime.recent_commands[-1]["gateway_queue_result"] == "0"


def test_unacknowledged_command_expires() -> None:
    runtime = VehicleRuntime(PoseEstimator())
    gateway = FakeGatewaySocket()

    async def exercise() -> None:
        await runtime.attach_gateway("anchor-a", gateway)
        pending = await runtime.send_vehicle_command("?", 1)
        pending.created_at -= 4.0

    asyncio.run(exercise())
    snapshot = runtime.snapshot()
    assert snapshot["pending_commands"] == []
    assert snapshot["recent_commands"][-1]["status"] == "timeout"
