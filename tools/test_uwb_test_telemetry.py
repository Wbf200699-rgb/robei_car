#!/usr/bin/env python3
"""Unit tests for the test-only UWB capture and analysis tools."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from analyze_uwb_test_telemetry import analyze_records, render_report  # noqa: E402
from capture_uwb_test_telemetry import (  # noqa: E402
    KinematicsEstimator,
    compact_state,
    normalized_angle_delta,
)


def valid_state(x_cm: float, y_cm: float, heading_deg: float) -> dict[str, object]:
    return {
        "gateway_connected": True,
        "gateway_data_fresh": True,
        "tag_fixes": {
            str(tag_id): {
                "fresh": True,
                "position_valid": True,
                "x_cm": x_cm + tag_id,
                "y_cm": y_cm + tag_id,
            }
            for tag_id in range(4)
        },
        "pose": {
            "quality": "unconfigured",
            "x_cm": x_cm,
            "y_cm": y_cm,
            "heading_deg": heading_deg,
        },
    }


class CaptureTests(unittest.TestCase):
    def test_angle_delta_wraps_through_180(self) -> None:
        self.assertAlmostEqual(normalized_angle_delta(-170.0, 170.0), 20.0)
        self.assertAlmostEqual(normalized_angle_delta(170.0, -170.0), -20.0)

    def test_windowed_kinematics(self) -> None:
        estimator = KinematicsEstimator(window_s=0.8)
        first = estimator.update(100.0, valid_state(0.0, 0.0, 170.0))
        second = estimator.update(100.4, valid_state(12.0, 16.0, -170.0))
        self.assertTrue(first["valid"])
        self.assertIsNone(first["speed_cm_s"])
        self.assertAlmostEqual(second["vx_cm_s"], 30.0)
        self.assertAlmostEqual(second["vy_cm_s"], 40.0)
        self.assertAlmostEqual(second["speed_cm_s"], 50.0)
        self.assertAlmostEqual(second["yaw_rate_deg_s"], 50.0)

    def test_invalid_sample_breaks_kinematics_window(self) -> None:
        estimator = KinematicsEstimator(window_s=0.8)
        estimator.update(100.0, valid_state(0.0, 0.0, 0.0))
        invalid = valid_state(1.0, 0.0, 0.0)
        invalid["gateway_data_fresh"] = False
        self.assertFalse(estimator.update(100.4, invalid)["valid"])
        restarted = estimator.update(100.8, valid_state(2.0, 0.0, 0.0))
        self.assertIsNone(restarted["speed_cm_s"])

    def test_compact_state_excludes_control_and_lidar_fields(self) -> None:
        state = valid_state(1.0, 2.0, 3.0)
        state.update(
            {
                "gateway_id": "gateway-test",
                "gateway_status": {"position_frames": 12},
                "gateway_status_age_ms": 10.0,
                "vehicle_lines": {"@S": "control"},
                "lidar_scan": {"points": [1, 2, 3]},
                "pending_commands": [{"command": "forward"}],
            }
        )
        compact = compact_state(state)
        self.assertIn("tag_fixes", compact)
        self.assertIn("pose", compact)
        self.assertNotIn("vehicle_lines", compact)
        self.assertNotIn("lidar_scan", compact)
        self.assertNotIn("pending_commands", compact)


class AnalysisTests(unittest.TestCase):
    def synthetic_samples(self) -> list[dict[str, object]]:
        samples: list[dict[str, object]] = []
        for index in range(3):
            state = valid_state(index * 10.0, 0.0, index * 5.0)
            state["gateway_status"] = {
                "position_frames": 100 + index * 10,
                "crc_errors": 2,
                "queue_drops": 0,
            }
            state["pose"].update(
                {
                    "front_width_cm": 40.0,
                    "rear_width_cm": 40.0,
                    "left_length_cm": 60.0,
                    "right_length_cm": 60.0,
                    "fix_skew_ms": 8.0,
                }
            )
            for tag_id, fix in state["tag_fixes"].items():
                fix.update(
                    {
                        "age_ms": 10.0,
                        "z_cm": 0.0,
                        "distances_cm": [100 + int(tag_id), 200, 300, 400],
                    }
                )
            samples.append(
                {
                    "record_type": "sample",
                    "captured_at_unix": 100.0 + index,
                    "state": state,
                    "derived": {
                        "valid": True,
                        "fresh_tag_count": 4,
                        "speed_cm_s": 10.0,
                        "yaw_rate_deg_s": 5.0,
                    },
                }
            )
        return samples

    def test_analysis_summarizes_motion_and_counters(self) -> None:
        metadata = {
            "schema": "little_car_uwb_test_v1",
            "test_only": True,
            "competition_design_input": False,
        }
        summary = analyze_records(metadata, self.synthetic_samples(), [], "unit-test")
        sample = summary["sample_summary"]
        self.assertEqual(sample["samples"], 3)
        self.assertEqual(sample["duration_s"], 2.0)
        self.assertEqual(sample["four_fresh_valid_tags_fraction"], 1.0)
        self.assertEqual(summary["vehicle_pose"]["path_length_cm"], 20.0)
        self.assertEqual(summary["vehicle_pose"]["displacement_cm"], 20.0)
        self.assertEqual(summary["gateway_counter_deltas"]["position_frames"]["delta"], 20)
        self.assertEqual(summary["tags"]["0"]["median_position_cm"]["x"], 10.0)
        self.assertTrue(summary["metadata_boundary_valid"])

    def test_report_contains_test_only_boundary(self) -> None:
        metadata = {
            "schema": "little_car_uwb_test_v1",
            "test_only": True,
            "competition_design_input": False,
        }
        report = render_report(analyze_records(metadata, self.synthetic_samples(), [], "unit-test"))
        self.assertIn("TEST ONLY", report)
        self.assertIn("not part of the competition architecture", report)


if __name__ == "__main__":
    unittest.main()
