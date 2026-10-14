#!/usr/bin/env python3
"""Analyze PMW3901/BMI088 calibration captures without external packages."""

import argparse
import csv
import json
import statistics
from pathlib import Path


GYRO_COUNTS_PER_DPS = 32768.0 / 250.0
TELEMETRY_PERIOD_S = 0.1
REQUIRED_STATUS_MASK = 0x1F
ERROR_STATUS_MASK = 0x60
MIN_SQUAL = 20


def load_records(path: Path):
    with path.open('r', encoding='utf-8-sig', newline='') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f'capture has no records: {path}')

    integer_fields = {
        'sequence', 'pmw_online', 'bmi0_online', 'bmi1_online',
        'calibration_done', 'fused_ready', 'imu_degraded',
        'imu_disagreement', 'flow_seen', 'squal', 'flow_dx', 'flow_dy',
        'flow_accum_x', 'flow_accum_y', 'accel_x', 'accel_y', 'accel_z',
        'gyro_x', 'gyro_y', 'gyro_z', 'bmi0_raw_gyro_z',
        'bmi1_raw_gyro_z',
    }
    records = []
    for row in rows:
        record = dict(row)
        for field in integer_fields:
            record[field] = int(record[field])
        record['status'] = int(record['status_hex'], 16)
        records.append(record)
    return records


def sample_stats(values):
    values = list(values)
    return {
        'min': min(values),
        'mean': statistics.fmean(values),
        'max': max(values),
        'stddev': statistics.pstdev(values),
    }


def wrapped_delta_32(last, first):
    return ((last - first + (1 << 31)) % (1 << 32)) - (1 << 31)


def sequence_gaps(records):
    return sum(
        1
        for previous, current in zip(records, records[1:])
        if (current['sequence'] - previous['sequence']) % 65536 != 1
    )


def integrate_gyro_axis(records, field):
    total_counts_seconds = 0.0
    for previous, current in zip(records, records[1:]):
        periods = (current['sequence'] - previous['sequence']) % 65536
        if periods == 0 or periods > 10:
            continue
        dt = periods * TELEMETRY_PERIOD_S
        total_counts_seconds += (previous[field] + current[field]) * 0.5 * dt
    return total_counts_seconds / GYRO_COUNTS_PER_DPS


def analyze_common(records):
    statuses = sorted({record['status'] for record in records})
    good_status_count = sum(
        1 for record in records
        if (record['status'] & REQUIRED_STATUS_MASK) == REQUIRED_STATUS_MASK
        and (record['status'] & ERROR_STATUS_MASK) == 0
    )
    good_squal_count = sum(record['squal'] >= MIN_SQUAL for record in records)
    return {
        'frames': len(records),
        'first_sequence': records[0]['sequence'],
        'last_sequence': records[-1]['sequence'],
        'sequence_gaps': sequence_gaps(records),
        'status_values': [f'{value:02X}' for value in statuses],
        'healthy_status_fraction': good_status_count / len(records),
        'usable_squal_fraction': good_squal_count / len(records),
        'squal': sample_stats(record['squal'] for record in records),
        'flow_dx': sample_stats(record['flow_dx'] for record in records),
        'flow_dy': sample_stats(record['flow_dy'] for record in records),
        'gyro_x': sample_stats(record['gyro_x'] for record in records),
        'gyro_y': sample_stats(record['gyro_y'] for record in records),
        'gyro_z': sample_stats(record['gyro_z'] for record in records),
        'raw_gyro_z_difference': sample_stats(
            record['bmi0_raw_gyro_z'] - record['bmi1_raw_gyro_z']
            for record in records
        ),
    }


def analyze_stationary(records):
    result = analyze_common(records)
    gyro_stddev_max = max(result[axis]['stddev'] for axis in ('gyro_x', 'gyro_y', 'gyro_z'))
    flow_delta_peak = max(
        abs(result[axis][bound])
        for axis in ('flow_dx', 'flow_dy')
        for bound in ('min', 'max')
    )
    checks = {
        'minimum_100_frames': result['frames'] >= 100,
        'no_sequence_gaps': result['sequence_gaps'] == 0,
        'all_sensors_healthy': result['healthy_status_fraction'] == 1.0,
        'optical_surface_valid': result['usable_squal_fraction'] >= 0.90,
        'gyro_stationary_noise': gyro_stddev_max <= 100.0,
        'flow_stationary_peak': flow_delta_peak <= 10,
    }
    result.update({
        'mode': 'stationary',
        'gyro_stddev_max': gyro_stddev_max,
        'flow_delta_peak': flow_delta_peak,
        'checks': checks,
        'pass': all(checks.values()),
    })
    return result


def analyze_linear(records, distance_mm):
    if distance_mm <= 0:
        raise ValueError('distance_mm must be positive')
    result = analyze_common(records)
    delta_x = wrapped_delta_32(records[-1]['flow_accum_x'], records[0]['flow_accum_x'])
    delta_y = wrapped_delta_32(records[-1]['flow_accum_y'], records[0]['flow_accum_y'])
    dominant_axis = 'x' if abs(delta_x) >= abs(delta_y) else 'y'
    dominant = delta_x if dominant_axis == 'x' else delta_y
    cross = delta_y if dominant_axis == 'x' else delta_x
    cross_ratio = abs(cross) / abs(dominant) if dominant else None
    checks = {
        'minimum_20_frames': result['frames'] >= 20,
        'no_sequence_gaps': result['sequence_gaps'] == 0,
        'all_sensors_healthy': result['healthy_status_fraction'] == 1.0,
        'optical_surface_valid': result['usable_squal_fraction'] >= 0.90,
        'measurable_motion': abs(dominant) >= 100,
        'cross_axis_below_25_percent': (
            cross_ratio is not None and cross_ratio <= 0.25
        ),
    }
    result.update({
        'mode': 'linear',
        'distance_mm': distance_mm,
        'flow_accum_delta_x': delta_x,
        'flow_accum_delta_y': delta_y,
        'dominant_axis': dominant_axis,
        'dominant_sign': 1 if dominant > 0 else -1 if dominant < 0 else 0,
        'counts_per_mm': abs(dominant) / distance_mm,
        'cross_axis_ratio': cross_ratio,
        'checks': checks,
        'pass': all(checks.values()),
    })
    return result


def analyze_rotation(records, angle_deg):
    if angle_deg <= 0:
        raise ValueError('angle_deg must be positive')
    result = analyze_common(records)
    integrated = {
        axis: integrate_gyro_axis(records, f'gyro_{axis}')
        for axis in ('x', 'y', 'z')
    }
    dominant_axis = max(integrated, key=lambda axis: abs(integrated[axis]))
    dominant_angle = integrated[dominant_axis]
    relative_error = abs(abs(dominant_angle) - angle_deg) / angle_deg
    checks = {
        'minimum_20_frames': result['frames'] >= 20,
        'no_sequence_gaps': result['sequence_gaps'] == 0,
        'all_sensors_healthy': result['healthy_status_fraction'] == 1.0,
        'dominant_rotation_detected': abs(dominant_angle) >= angle_deg * 0.50,
        'angle_error_below_35_percent': relative_error <= 0.35,
    }
    result.update({
        'mode': 'rotation',
        'commanded_angle_deg': angle_deg,
        'integrated_angle_deg': integrated,
        'dominant_axis': dominant_axis,
        'dominant_sign': 1 if dominant_angle > 0 else -1 if dominant_angle < 0 else 0,
        'relative_angle_error': relative_error,
        'flow_accum_delta_x': wrapped_delta_32(
            records[-1]['flow_accum_x'], records[0]['flow_accum_x']
        ),
        'flow_accum_delta_y': wrapped_delta_32(
            records[-1]['flow_accum_y'], records[0]['flow_accum_y']
        ),
        'checks': checks,
        'pass': all(checks.values()),
    })
    return result


def format_markdown(source, result):
    lines = [
        '# Motion calibration analysis',
        '',
        f'- Source: `{source}`',
        f'- Mode: `{result["mode"]}`',
        f'- Result: `{"PASS" if result["pass"] else "FAIL"}`',
        f'- Frames: {result["frames"]}',
        f'- Sequence gaps: {result["sequence_gaps"]}',
        f'- Status values: {", ".join(result["status_values"])}',
        f'- SQUAL mean/min/max: {result["squal"]["mean"]:.2f} / '
        f'{result["squal"]["min"]} / {result["squal"]["max"]}',
        '',
        '## Checks',
        '',
    ]
    for name, passed in result['checks'].items():
        lines.append(f'- `{"PASS" if passed else "FAIL"}` {name}')
    lines.extend(['', '## Result data', '', '```json', json.dumps(result, indent=2), '```', ''])
    return '\n'.join(lines)


def self_test():
    records = []
    for index in range(120):
        records.append({
            'sequence': index,
            'status': 0x9F,
            'squal': 60,
            'flow_dx': 0,
            'flow_dy': 0,
            'flow_accum_x': 0,
            'flow_accum_y': 0,
            'gyro_x': 1,
            'gyro_y': -2,
            'gyro_z': 3,
            'bmi0_raw_gyro_z': 10,
            'bmi1_raw_gyro_z': 12,
        })
    result = analyze_stationary(records)
    if not result['pass']:
        raise AssertionError(result)

    linear_records = [dict(record) for record in records]
    for index, record in enumerate(linear_records):
        record['flow_accum_x'] = index * 5
    linear_result = analyze_linear(linear_records, 500.0)
    if not linear_result['pass'] or linear_result['dominant_axis'] != 'x':
        raise AssertionError(linear_result)

    rotation_records = [dict(record) for record in records]
    rotation_rate_counts = round((360.0 / 11.9) * GYRO_COUNTS_PER_DPS)
    for record in rotation_records:
        record['gyro_z'] = rotation_rate_counts
    rotation_result = analyze_rotation(rotation_records, 360.0)
    if not rotation_result['pass'] or rotation_result['dominant_axis'] != 'z':
        raise AssertionError(rotation_result)
    print('RESULT TEST_PASS motion_calibration_analyzer')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', type=Path, help='decoded capture CSV')
    parser.add_argument('--mode', choices=('stationary', 'linear', 'rotation'))
    parser.add_argument('--distance-mm', type=float)
    parser.add_argument('--angle-deg', type=float)
    parser.add_argument('--output', type=Path, help='write Markdown evidence')
    parser.add_argument('--json-output', type=Path, help='write machine-readable result')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return
    if args.csv is None or args.mode is None:
        parser.error('csv and --mode are required unless --self-test is used')

    records = load_records(args.csv)
    if args.mode == 'stationary':
        result = analyze_stationary(records)
    elif args.mode == 'linear':
        if args.distance_mm is None:
            parser.error('--distance-mm is required for linear mode')
        result = analyze_linear(records, args.distance_mm)
    else:
        if args.angle_deg is None:
            parser.error('--angle-deg is required for rotation mode')
        result = analyze_rotation(records, args.angle_deg)

    markdown = format_markdown(args.csv, result)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(markdown, encoding='utf-8')
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(markdown)


if __name__ == '__main__':
    main()
