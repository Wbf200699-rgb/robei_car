from __future__ import annotations

from dataclasses import dataclass
import os

from .pose import PoseConfig
from .vehicle_geometry_generated import (
    FRONT_LEFT_TAG,
    FRONT_RIGHT_TAG,
    MAXIMUM_FIX_AGE_S,
    MAXIMUM_FIX_SKEW_S,
    REAR_LEFT_TAG,
    REAR_RIGHT_TAG,
    VEHICLE_DIMENSION_TOLERANCE_M,
    VEHICLE_TAG_LENGTH_M,
    VEHICLE_TAG_WIDTH_M,
)


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    return default if value is None else int(value)


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    return default if value is None else float(value)


@dataclass(frozen=True)
class Settings:
    gateway_token: str
    mobile_token: str
    command_tag_id: int
    pose: PoseConfig

    @classmethod
    def from_env(cls) -> "Settings":
        return cls(
            gateway_token=os.getenv("UWB_GATEWAY_TOKEN", "change-me-gateway"),
            mobile_token=os.getenv("UWB_MOBILE_TOKEN", "change-me-mobile"),
            command_tag_id=_env_int("UWB_COMMAND_TAG_ID", 1),
            pose=PoseConfig(
                front_left_tag=_env_int("UWB_TAG_FRONT_LEFT", FRONT_LEFT_TAG),
                front_right_tag=_env_int("UWB_TAG_FRONT_RIGHT", FRONT_RIGHT_TAG),
                rear_left_tag=_env_int("UWB_TAG_REAR_LEFT", REAR_LEFT_TAG),
                rear_right_tag=_env_int("UWB_TAG_REAR_RIGHT", REAR_RIGHT_TAG),
                vehicle_length_cm=_env_float("UWB_VEHICLE_LENGTH_CM", VEHICLE_TAG_LENGTH_M * 100.0),
                vehicle_width_cm=_env_float("UWB_VEHICLE_WIDTH_CM", VEHICLE_TAG_WIDTH_M * 100.0),
                dimension_tolerance_cm=_env_float(
                    "UWB_DIMENSION_TOLERANCE_CM", VEHICLE_DIMENSION_TOLERANCE_M * 100.0
                ),
                maximum_fix_age_s=_env_float("UWB_MAX_FIX_AGE_S", MAXIMUM_FIX_AGE_S),
                maximum_fix_skew_s=_env_float("UWB_MAX_FIX_SKEW_S", MAXIMUM_FIX_SKEW_S),
            ),
        )
