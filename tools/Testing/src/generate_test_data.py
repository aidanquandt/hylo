#!/usr/bin/env python3
"""
Generate fake CSV test data for dynamic_accuracy_test.py and
dynamic_accuracy_orientation_test.py.

Simulates the actual sensor fusion cadence:
  - EKF prediction at 200 Hz (IMU accel + gyro, accumulates drift)
  - EKF update at 200 Hz interleaved: every 4th sample is a TWR range
    to one of 4 anchors (= 50 Hz full TWR cycle)

    t=0ms   IMU predict + TWR update (anchor 0)
    t=5ms   IMU predict + TWR update (anchor 1)
    t=10ms  IMU predict + TWR update (anchor 2)
    t=15ms  IMU predict + TWR update (anchor 3)
    t=20ms  IMU predict + TWR update (anchor 0)  ... repeats

Creates three CSVs with varying noise/drift levels:
  - test_data_good.csv    : low noise, low drift  (should PASS)
  - test_data_medium.csv  : moderate noise/drift   (borderline)
  - test_data_bad.csv     : high noise, high drift (should FAIL)

Usage:
    python generate_test_data.py
    python generate_test_data.py --side-length 4 --laps 3
"""

import argparse
import os
import numpy as np


def build_ground_truth(l, straight_speed_kmh, turn_speed_kmh, num_laps,
                       points_per_meter=50):
    """Build ground-truth rounded-square path with heading."""
    r = l / 4.0
    v_s = straight_speed_kmh / 3.6
    v_t = turn_speed_kmh / 3.6
    half = l / 2.0

    segments = [
        ('line', 0, 0, half - r, 0),
        ('arc',  half - r, r, r, -np.pi / 2, 0),
        ('line', half, r, half, l - r),
        ('arc',  half - r, l - r, r, 0, np.pi / 2),
        ('line', half - r, l, -half + r, l),
        ('arc',  -half + r, l - r, r, np.pi / 2, np.pi),
        ('line', -half, l - r, -half, r),
        ('arc',  -half + r, r, r, np.pi, 3 * np.pi / 2),
        ('line', -half + r, 0, 0, 0),
    ]

    all_times = []
    all_points = []
    all_headings = []
    current_time = 0.0

    for seg in segments:
        kind = seg[0]
        if kind == 'line':
            _, x0, y0, x1, y1 = seg
            length = np.hypot(x1 - x0, y1 - y0)
            n = max(int(length * points_per_meter), 2)
            t_seg = length / v_s
            ts = np.linspace(0, t_seg, n)
            xs = np.linspace(x0, x1, n)
            ys = np.linspace(y0, y1, n)
            heading = np.degrees(np.arctan2(y1 - y0, x1 - x0))
            hs = np.full(n, heading)
        else:
            _, cx, cy, radius, a0, a1 = seg
            arc_len = abs(a1 - a0) * radius
            n = max(int(arc_len * points_per_meter), 2)
            t_seg = arc_len / v_t
            ts = np.linspace(0, t_seg, n)
            angles = np.linspace(a0, a1, n)
            xs = cx + radius * np.cos(angles)
            ys = cy + radius * np.sin(angles)
            direction = 1.0 if a1 > a0 else -1.0
            hs = np.degrees(angles + direction * np.pi / 2)

        if len(all_points) > 0:
            ts = ts[1:]
            xs = xs[1:]
            ys = ys[1:]
            hs = hs[1:]

        all_times.extend((ts + current_time).tolist())
        all_points.extend(zip(xs.tolist(), ys.tolist()))
        all_headings.extend(hs.tolist())
        current_time += t_seg

    one_lap_time = current_time
    one_lap_times = np.array(all_times)
    one_lap_positions = np.array(all_points)
    one_lap_headings = np.array(all_headings)

    gt_times_list = [one_lap_times]
    gt_pos_list = [one_lap_positions]
    gt_hdg_list = [one_lap_headings]

    for lap in range(1, num_laps):
        gt_times_list.append(one_lap_times[1:] + lap * one_lap_time)
        gt_pos_list.append(one_lap_positions[1:])
        gt_hdg_list.append(one_lap_headings[1:])

    gt_headings = np.concatenate(gt_hdg_list)
    gt_headings = (gt_headings + 180) % 360 - 180

    return (np.concatenate(gt_times_list),
            np.vstack(gt_pos_list),
            gt_headings,
            one_lap_time)


def interpolate_gt(sample_times, gt_times, gt_positions, gt_headings):
    """Interpolate ground truth at sample times."""
    x = np.interp(sample_times, gt_times, gt_positions[:, 0])
    y = np.interp(sample_times, gt_times, gt_positions[:, 1])
    hdg_rad = np.radians(gt_headings)
    s = np.interp(sample_times, gt_times, np.sin(hdg_rad))
    c = np.interp(sample_times, gt_times, np.cos(hdg_rad))
    hdg = np.degrees(np.arctan2(s, c))
    return x, y, hdg


def generate_csv(filepath, gt_times, gt_positions, gt_headings,
                 pos_noise, pos_drift, hdg_noise, hdg_drift,
                 anchor_positions, twr_range_std=0.20):
    """
    Generate a fake CSV simulating the EKF fusion cadence.

    The EKF runs at 200 Hz. Each 5ms tick:
      1. IMU prediction: propagates position using velocity + accel,
         accumulating process noise (random walk).
      2. TWR update (one anchor per tick, cycling 0-1-2-3 at 50 Hz):
         pulls the estimated position toward ground truth, weighted by
         the TWR measurement noise vs accumulated IMU drift.

    This produces realistic correlated errors: position drifts between
    TWR corrections and snaps back when ranging updates arrive.
    """
    IMU_HZ = 200.0
    NUM_ANCHORS = 4
    dt = 1.0 / IMU_HZ

    sample_times = np.arange(0, gt_times[-1], dt)
    n = len(sample_times)

    # Ground truth at each sample
    x_gt, y_gt, hdg_gt = interpolate_gt(
        sample_times, gt_times, gt_positions, gt_headings)

    # Ground truth velocity (body-frame forward, since robot faces forward)
    vx_gt = np.gradient(x_gt, dt)
    vy_gt = np.gradient(y_gt, dt)

    # ── Simulate EKF state propagation ───────────────────────────────────
    x_est = np.zeros(n)
    y_est = np.zeros(n)
    vx_est = np.zeros(n)
    vy_est = np.zeros(n)

    # Start at ground truth
    x_est[0] = x_gt[0]
    y_est[0] = y_gt[0]
    vx_est[0] = vx_gt[0]
    vy_est[0] = vy_gt[0]

    # IMU process noise per step (accel noise → velocity → position)
    imu_accel_noise = pos_noise * 40.0   # maps to ~pos_noise after integration
    # TWR correction gain: how strongly a range pulls the state back
    # Higher = better correction, lower = more IMU drift visible
    twr_gain = 0.3

    for i in range(1, n):
        # ── Step 1: IMU Prediction ───────────────────────────────────
        # Use GT velocity + noise as the IMU-predicted velocity
        ax_noise = np.random.normal(0, imu_accel_noise)
        ay_noise = np.random.normal(0, imu_accel_noise)

        vx_est[i] = vx_gt[i] + ax_noise * dt + pos_drift
        vy_est[i] = vy_gt[i] + ay_noise * dt + pos_drift * 0.5

        # Propagate position with noisy velocity
        x_est[i] = x_est[i - 1] + vx_est[i] * dt
        y_est[i] = y_est[i - 1] + vy_est[i] * dt

        # ── Step 2: TWR Update (one anchor per 5ms tick) ────────────
        anchor_idx = i % NUM_ANCHORS
        ax_pos = anchor_positions[anchor_idx]

        # Simulated measured range (true distance + noise)
        true_dist = np.sqrt((x_gt[i] - ax_pos[0])**2 +
                            (y_gt[i] - ax_pos[1])**2)
        meas_dist = true_dist + np.random.normal(0, twr_range_std)

        # Expected distance from current estimate
        est_dist = np.sqrt((x_est[i] - ax_pos[0])**2 +
                           (y_est[i] - ax_pos[1])**2)

        # Scalar Kalman-like correction along the anchor direction
        if est_dist > 0.01:
            innovation = meas_dist - est_dist
            # Unit vector from estimate to anchor
            hx = (x_est[i] - ax_pos[0]) / est_dist
            hy = (y_est[i] - ax_pos[1]) / est_dist
            # Apply correction
            x_est[i] += twr_gain * innovation * hx
            y_est[i] += twr_gain * innovation * hy

    # ── Heading: IMU-integrated yaw with noise + drift ───────────────
    yaw_est = hdg_gt + np.random.normal(0, hdg_noise, n)
    yaw_est += hdg_drift * sample_times
    yaw_est = (yaw_est + 180) % 360 - 180

    # ── Velocity from estimated positions ────────────────────────────
    vx_out = np.gradient(x_est, dt)
    vy_out = np.gradient(y_est, dt)
    vz_out = np.zeros(n)

    # z is flat (no vertical motion)
    z_est = np.random.normal(0, 0.005, n)

    # Attitude
    roll = np.random.normal(0, 0.5, n)
    pitch = np.random.normal(0, 0.5, n)

    # Timestamps in ms
    timestamps_ms = (sample_times * 1000).astype(int)

    # Write CSV
    with open(filepath, 'w') as f:
        f.write("timestamp_ms,x,y,z,vx,vy,vz,roll,pitch,yaw\n")
        for i in range(n):
            f.write(f"{timestamps_ms[i]},{x_est[i]:.6f},{y_est[i]:.6f},"
                    f"{z_est[i]:.6f},"
                    f"{vx_out[i]:.6f},{vy_out[i]:.6f},{vz_out[i]:.6f},"
                    f"{roll[i]:.4f},{pitch[i]:.4f},{yaw_est[i]:.4f}\n")

    # Stats
    pos_err = np.sqrt((x_est - x_gt)**2 + (y_est - y_gt)**2)
    cep50 = np.median(pos_err)
    hdg_err = np.abs((yaw_est - hdg_gt + 180) % 360 - 180)
    return cep50, np.median(hdg_err), n


def main():
    parser = argparse.ArgumentParser(
        description='Generate fake test CSV data for dynamic accuracy tests')
    parser.add_argument('--side-length', type=float, default=5.0)
    parser.add_argument('--straight-speed', type=float, default=5.0)
    parser.add_argument('--turn-speed', type=float, default=2.0)
    parser.add_argument('--laps', type=int, default=5)
    parser.add_argument('--twr-std', type=float, default=0.20,
                        help='TWR ranging std dev in metres (default: 0.20)')
    parser.add_argument('--output-dir', type=str,
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             '..', 'dummy_data'))
    args = parser.parse_args()

    gt_times, gt_positions, gt_headings, one_lap_time = build_ground_truth(
        args.side_length, args.straight_speed, args.turn_speed, args.laps)

    print()
    print("=" * 60)
    print(f"  Generating test CSVs for {args.side_length}m square, "
          f"{args.laps} laps")
    print(f"  EKF cadence: 200Hz IMU predict, 50Hz TWR (4 anchors)")
    print(f"  TWR range std: {args.twr_std:.2f} m")
    print(f"  Output dir: {args.output_dir}")
    print("=" * 60)
    print()

    os.makedirs(args.output_dir, exist_ok=True)

    # Anchor positions: 4 corners of the test grid (matching test spec figure)
    half = args.side_length / 2.0
    margin = args.side_length * 0.2
    anchor_positions = [
        (-half - margin, -margin),
        ( half + margin, -margin),
        ( half + margin,  args.side_length + margin),
        (-half - margin,  args.side_length + margin),
    ]

    configs = [
        {
            'name': 'test_data_good',
            'desc': 'Low noise, low drift (should PASS)',
            'pos_noise': 0.01,
            'pos_drift': 0.0001,
            'hdg_noise': 1.0,
            'hdg_drift': 0.005,
        },
        {
            'name': 'test_data_medium',
            'desc': 'Moderate noise/drift (borderline)',
            'pos_noise': 0.025,
            'pos_drift': 0.0005,
            'hdg_noise': 3.0,
            'hdg_drift': 0.03,
        },
        {
            'name': 'test_data_bad',
            'desc': 'High noise, high drift (should FAIL)',
            'pos_noise': 0.06,
            'pos_drift': 0.002,
            'hdg_noise': 8.0,
            'hdg_drift': 0.1,
        },
    ]

    for cfg in configs:
        filepath = os.path.join(args.output_dir, f"{cfg['name']}.csv")
        cep50, hdg_med, n_samples = generate_csv(
            filepath, gt_times, gt_positions, gt_headings,
            cfg['pos_noise'], cfg['pos_drift'],
            cfg['hdg_noise'], cfg['hdg_drift'],
            anchor_positions, args.twr_std)

        status = "PASS" if cep50 <= 0.05 else "FAIL"
        print(f"  {cfg['name']}.csv")
        print(f"    {cfg['desc']}")
        print(f"    Samples: {n_samples}  |  CEP50: {cep50*100:.2f} cm  "
              f"|  Hdg median: {hdg_med:.1f} deg  |  {status}")
        print()

    print("  Done! Use with:")
    print(f"    python dynamic_accuracy_test.py "
          f"--data test_data_good.csv --side-length {args.side_length}")
    print(f"    python dynamic_accuracy_orientation_test.py "
          f"--data test_data_good.csv --side-length {args.side_length}")
    print()


if __name__ == '__main__':
    main()
