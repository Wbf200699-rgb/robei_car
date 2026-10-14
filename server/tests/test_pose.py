from app.pose import PoseConfig, PoseEstimator
from app.protocol import PositionReport


def fix(tag_id: int, x_cm: int, y_cm: int) -> PositionReport:
    return PositionReport(
        gateway_sequence=tag_id,
        tag_id=tag_id,
        flags=1 << 16,
        x_cm=x_cm,
        y_cm=y_cm,
        z_cm=0,
        distances_cm=(0, 0, 0, 0),
    )


def test_rectangular_pose_and_heading() -> None:
    estimator = PoseEstimator(
        PoseConfig(
            vehicle_length_cm=60,
            vehicle_width_cm=40,
            dimension_tolerance_cm=1,
            maximum_fix_age_s=1,
        )
    )
    timestamp = 100.0
    estimator.update(fix(1, 60, 20), timestamp)   # front-left
    estimator.update(fix(2, 60, -20), timestamp)  # front-right
    estimator.update(fix(0, 0, 20), timestamp)    # rear-left
    pose = estimator.update(fix(3, 0, -20), timestamp)  # rear-right

    assert pose.quality == "full"
    assert pose.rigid_rectangle is True
    assert pose.x_cm == 30
    assert pose.y_cm == 0
    assert pose.heading_deg == 0


def test_project_geometry_defaults_and_tag_roles() -> None:
    estimator = PoseEstimator()
    timestamp = 100.0

    # Vehicle center is (80, 90) cm and heading is +90 degrees. Rotating the
    # configured tag offsets by +90 degrees maps (x, y) to (-y, x).
    estimator.update(fix(1, 70, 101), timestamp)  # front-left
    estimator.update(fix(2, 90, 101), timestamp)  # front-right
    estimator.update(fix(0, 70, 79), timestamp)   # rear-left
    pose = estimator.update(fix(3, 90, 79), timestamp)  # rear-right

    assert pose.quality == "full"
    assert pose.valid_tags == (0, 1, 2, 3)
    assert abs(pose.x_cm - 80.0) <= 0.5
    assert abs(pose.y_cm - 90.0) <= 0.5
    assert abs(pose.heading_deg - 90.0) < 0.001
    assert pose.rigid_rectangle is True


def test_unconfigured_dimensions_do_not_claim_full_validation() -> None:
    estimator = PoseEstimator(PoseConfig(
        vehicle_length_cm=0,
        vehicle_width_cm=0,
        maximum_fix_age_s=1,
    ))
    for tag_id, x_cm, y_cm in ((2, 60, 20), (3, 60, -20), (1, 0, 20)):
        estimator.update(fix(tag_id, x_cm, y_cm), 100.0)
    pose = estimator.update(fix(0, 0, -20), 100.0)
    assert pose.quality == "unconfigured"
    assert pose.rigid_rectangle is None


def test_four_tag_pose_rejects_unsynchronized_samples() -> None:
    estimator = PoseEstimator(
        PoseConfig(
            vehicle_length_cm=60,
            vehicle_width_cm=40,
            dimension_tolerance_cm=1,
            maximum_fix_age_s=1,
            maximum_fix_skew_s=0.1,
        )
    )
    estimator.update(fix(2, 60, 20), 100.00)
    estimator.update(fix(3, 60, -20), 100.04)
    estimator.update(fix(1, 0, 20), 100.08)
    pose = estimator.update(fix(0, 0, -20), 100.20)
    assert pose.quality == "unsynchronized"
    assert pose.fix_skew_ms is not None
    assert abs(pose.fix_skew_ms - 200) < 0.001


def test_pose_expires_when_no_new_fix_arrives() -> None:
    estimator = PoseEstimator(PoseConfig(maximum_fix_age_s=0.5))
    estimator.update(fix(1, 60, 20), 100.0)
    assert estimator.estimate(100.1).quality == "partial"
    assert estimator.estimate(100.6).quality == "none"


def test_three_tags_fit_known_rigid_rectangle() -> None:
    estimator = PoseEstimator(PoseConfig(
        vehicle_length_cm=22,
        vehicle_width_cm=20,
        dimension_tolerance_cm=2,
        maximum_fix_age_s=1,
    ))
    estimator.update(fix(2, 90, 111), 100.00)  # front-right at +90 degrees
    estimator.update(fix(0, 70, 89), 100.02)   # rear-left
    pose = estimator.update(fix(3, 90, 89), 100.04)  # rear-right

    assert pose.quality == "partial_rigid"
    assert pose.valid_tags == (0, 2, 3)
    assert abs(pose.x_cm - 80.0) <= 0.5
    assert abs(pose.y_cm - 100.0) <= 0.5
    assert abs(pose.heading_deg - 90.0) <= 0.5
    assert pose.rigid_rectangle is True


def test_invalid_position_flag_is_excluded() -> None:
    estimator = PoseEstimator(PoseConfig(maximum_fix_age_s=1))
    invalid = PositionReport(1, 1, 0x0F, 0, 0, 0, (1, 1, 1, 1))
    pose = estimator.update(invalid, 100.0)
    assert pose.quality == "none"


def test_five_sample_median_rejects_single_coordinate_outlier() -> None:
    estimator = PoseEstimator(PoseConfig(maximum_fix_age_s=1))
    stable = [(50, 80), (51, 79), (49, 81), (50, 80)]
    for sequence, (x_cm, y_cm) in enumerate(stable, start=1):
        estimator.update(PositionReport(
            sequence, 1, 1 << 16, x_cm, y_cm, 0, (10, 20, 30, 40)
        ), 100.0 + sequence * 0.01)

    estimator.update(PositionReport(
        5, 1, 1 << 16, 95, 20, 0, (11, 21, 31, 41)
    ), 100.05)
    filtered = estimator.latest_report(1)

    assert filtered is not None
    assert filtered.gateway_sequence == 5
    assert filtered.x_cm == 50
    assert filtered.y_cm == 80
    assert filtered.distances_cm == (11, 21, 31, 41)
