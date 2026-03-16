#!/usr/bin/env python3
"""
Dynamic Accuracy Test - Square-Path Traversal (Section 4.1)

Compares algorithm-estimated positions against a ground-truth rounded-square
path. Computes CEP50 and animates algorithm vs ground truth in real time.

Usage:
    # With real data:
    python dynamic_accuracy_test.py --data log.csv --side-length 5

    # Demo mode (synthetic data):
    python dynamic_accuracy_test.py --demo --side-length 5

    # Ground truth only:
    python dynamic_accuracy_test.py --ground-truth-only --side-length 5

CSV format expected (header row required):
    timestamp_ms, x, y, z [, vx, vy, vz, roll, pitch, yaw]
    Positions in meters, timestamp in milliseconds.
"""

import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ─── Ground-Truth Path Construction ──────────────────────────────────────────

def build_ground_truth(l, straight_speed_kmh, turn_speed_kmh, num_laps,
                       points_per_meter=50):
    """
    Build a time-parameterised ground-truth rounded-square path.

    The path origin (0, 0) is at the midpoint of the bottom edge.
    Direction: clockwise (right -> up -> left -> down).
    Turn radius: l / 4.

    Returns
    -------
    gt_times : ndarray, shape (N,)
        Timestamps in seconds.
    gt_positions : ndarray, shape (N, 2)
        (x, y) positions in metres.
    one_lap_time : float
        Duration of a single lap in seconds.
    """
    r = l / 4.0
    v_s = straight_speed_kmh / 3.6
    v_t = turn_speed_kmh / 3.6
    half = l / 2.0

    # Each segment: ('line', x0, y0, x1, y1) or ('arc', cx, cy, r, a0, a1)
    segments = [
        ('line', 0, 0, half - r, 0),                       # bottom right-half
        ('arc',  half - r, r, r, -np.pi / 2, 0),           # bottom-right corner
        ('line', half, r, half, l - r),                     # right side up
        ('arc',  half - r, l - r, r, 0, np.pi / 2),        # top-right corner
        ('line', half - r, l, -half + r, l),                # top left
        ('arc',  -half + r, l - r, r, np.pi / 2, np.pi),   # top-left corner
        ('line', -half, l - r, -half, r),                   # left side down
        ('arc',  -half + r, r, r, np.pi, 3 * np.pi / 2),   # bottom-left corner
        ('line', -half + r, 0, 0, 0),                       # bottom left-half
    ]

    all_times = []
    all_points = []
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

        else:  # arc
            _, cx, cy, radius, a0, a1 = seg
            arc_len = abs(a1 - a0) * radius
            n = max(int(arc_len * points_per_meter), 2)
            t_seg = arc_len / v_t
            ts = np.linspace(0, t_seg, n)
            angles = np.linspace(a0, a1, n)
            xs = cx + radius * np.cos(angles)
            ys = cy + radius * np.sin(angles)

        # Avoid duplicate junction points
        if len(all_points) > 0:
            ts = ts[1:]
            xs = xs[1:]
            ys = ys[1:]

        all_times.extend((ts + current_time).tolist())
        all_points.extend(zip(xs.tolist(), ys.tolist()))
        current_time += t_seg

    one_lap_time = current_time
    one_lap_times = np.array(all_times)
    one_lap_positions = np.array(all_points)

    # Stack multiple laps
    gt_times_list = [one_lap_times]
    gt_pos_list = [one_lap_positions]

    for lap in range(1, num_laps):
        gt_times_list.append(one_lap_times[1:] + lap * one_lap_time)
        gt_pos_list.append(one_lap_positions[1:])

    return (np.concatenate(gt_times_list),
            np.vstack(gt_pos_list),
            one_lap_time)


# ─── Interpolation & Metrics ────────────────────────────────────────────────

def interpolate_ground_truth(query_times, gt_times, gt_positions):
    """Linearly interpolate ground-truth (x, y) at arbitrary query times."""
    x_interp = np.interp(query_times, gt_times, gt_positions[:, 0])
    y_interp = np.interp(query_times, gt_times, gt_positions[:, 1])
    return np.column_stack([x_interp, y_interp])


def compute_cep50(errors_2d):
    """
    Compute CEP50: the median of 2-D radial position errors.

    Returns
    -------
    cep50 : float
    radial_errors : ndarray
    """
    radial = np.hypot(errors_2d[:, 0], errors_2d[:, 1])
    return float(np.median(radial)), radial


# ─── Data I/O ───────────────────────────────────────────────────────────────

def load_csv_data(filepath):
    """
    Load algorithm output CSV.

    Expected columns: timestamp_ms, x, y, z [, ...]
    Returns timestamps in seconds (zero-based) and 3-D positions.
    """
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    timestamps_s = (data[:, 0] - data[0, 0]) / 1000.0
    positions = data[:, 1:4]
    return timestamps_s, positions


def generate_demo_data(gt_times, gt_positions,
                       noise_std=0.02, drift_rate=0.0004, sample_hz=50):
    """Generate synthetic algorithm output with noise + slow drift."""
    dt = 1.0 / sample_hz
    sample_times = np.arange(0, gt_times[-1], dt)
    gt_interp = interpolate_ground_truth(sample_times, gt_times, gt_positions)

    noise = np.random.normal(0, noise_std, gt_interp.shape)
    drift = (drift_rate * sample_times[:, np.newaxis]
             * np.array([1.0, 0.5]))

    algo_xy = gt_interp + noise + drift
    z = np.random.normal(0, 0.005, len(sample_times))

    return sample_times, np.column_stack([algo_xy, z])


# ─── Visualisation ──────────────────────────────────────────────────────────

def create_visualisation(l, gt_times, gt_positions,
                         algo_times, algo_xy, gt_at_algo,
                         radial_errors, cep50, passed, args):
    """Animated + static dashboard."""

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3, width_ratios=[2, 1, 1],
                          hspace=0.35, wspace=0.3)

    # ── Main path plot ───────────────────────────────────────────────────
    ax_path = fig.add_subplot(gs[:, 0])
    ax_path.set_aspect('equal')
    ax_path.set_xlabel('x-position [m]')
    ax_path.set_ylabel('y-position [m]')
    ax_path.set_title(f'Dynamic Accuracy Test \u2014 {l}m Square Path')
    ax_path.grid(True, alpha=0.3)

    # Full ground-truth outline (faint)
    ax_path.plot(gt_positions[:, 0], gt_positions[:, 1],
                 'g-', alpha=0.25, linewidth=2, label='Ground Truth Path')
    ax_path.plot(0, 0, '*', color='green', markersize=15,
                 zorder=5, label='Origin')

    # Animated lines
    line_algo, = ax_path.plot([], [], 'r-', linewidth=1.5, alpha=0.8,
                              label='Algorithm')
    dot_algo, = ax_path.plot([], [], 'ro', markersize=6, zorder=5)
    line_gt_anim, = ax_path.plot([], [], 'g-', linewidth=1.5, alpha=0.8)
    dot_gt, = ax_path.plot([], [], 'go', markersize=6, zorder=5)
    line_error, = ax_path.plot([], [], 'k--', linewidth=0.8, alpha=0.5)

    margin = l * 0.15
    ax_path.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax_path.set_ylim(-margin, l + margin)
    ax_path.legend(loc='upper left', fontsize=8)

    time_text = ax_path.text(
        0.02, 0.98, '', transform=ax_path.transAxes, fontsize=9,
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # ── Error vs time ────────────────────────────────────────────────────
    ax_err = fig.add_subplot(gs[0, 1:])
    ax_err.set_xlabel('Time [s]')
    ax_err.set_ylabel('Position Error [m]')
    ax_err.set_title('2D Radial Error vs Time')
    ax_err.grid(True, alpha=0.3)
    ax_err.axhline(y=0.05, color='r', linestyle='--', alpha=0.5,
                   label='CEP50 threshold (5 cm)')
    ax_err.axhline(y=cep50, color='orange', linestyle='-', alpha=0.7,
                   label=f'CEP50 = {cep50 * 100:.2f} cm')
    line_err, = ax_err.plot([], [], 'b-', linewidth=0.8, alpha=0.8)
    ax_err.set_xlim(0, algo_times[-1])
    ax_err.set_ylim(0, max(np.max(radial_errors) * 1.2, 0.1))
    ax_err.legend(fontsize=8)

    # ── Error histogram ──────────────────────────────────────────────────
    ax_hist = fig.add_subplot(gs[1, 1])
    ax_hist.hist(radial_errors * 100, bins=40,
                 color='steelblue', alpha=0.7, edgecolor='white')
    ax_hist.axvline(x=cep50 * 100, color='orange', linewidth=2,
                    label=f'CEP50 = {cep50 * 100:.2f} cm')
    ax_hist.axvline(x=5.0, color='r', linewidth=1.5, linestyle='--',
                    label='Threshold = 5 cm')
    ax_hist.set_xlabel('Error [cm]')
    ax_hist.set_ylabel('Count')
    ax_hist.set_title('Error Distribution')
    ax_hist.legend(fontsize=7)

    # ── Stats panel ──────────────────────────────────────────────────────
    ax_stats = fig.add_subplot(gs[1, 2])
    ax_stats.axis('off')

    status_text = 'PASS' if passed else 'FAIL'
    status_colour = 'green' if passed else 'red'

    r_turn = l / 4.0
    total_straight = 4 * (l - 2 * r_turn)
    total_arc = 2 * np.pi * r_turn
    lap_dist = total_straight + total_arc

    stats = (
        f"Test Results\n"
        f"{'─' * 28}\n"
        f"Side length:   {l} m\n"
        f"Turn radius:   {r_turn:.2f} m\n"
        f"Lap distance:  {lap_dist:.2f} m\n"
        f"Laps:          {args.laps}\n"
        f"Samples:       {len(algo_times)}\n"
        f"{'─' * 28}\n"
        f"CEP50:         {cep50 * 100:.2f} cm\n"
        f"Mean error:    {np.mean(radial_errors) * 100:.2f} cm\n"
        f"Max error:     {np.max(radial_errors) * 100:.2f} cm\n"
        f"Std error:     {np.std(radial_errors) * 100:.2f} cm\n"
        f"{'─' * 28}\n"
    )
    ax_stats.text(0.05, 0.95, stats, transform=ax_stats.transAxes,
                  fontsize=9, verticalalignment='top', fontfamily='monospace')
    ax_stats.text(0.05, 0.08, f"  {status_text}",
                  transform=ax_stats.transAxes, fontsize=22,
                  fontweight='bold', color=status_colour, fontfamily='monospace')

    # ── Animation ────────────────────────────────────────────────────────
    max_frames = 600
    frame_step = max(1, len(algo_times) // max_frames)
    total_frames = len(algo_times) // frame_step

    def init():
        line_algo.set_data([], [])
        dot_algo.set_data([], [])
        line_gt_anim.set_data([], [])
        dot_gt.set_data([], [])
        line_error.set_data([], [])
        line_err.set_data([], [])
        time_text.set_text('')
        return (line_algo, dot_algo, line_gt_anim, dot_gt,
                line_error, line_err, time_text)

    def animate(frame):
        idx = min(frame * frame_step, len(algo_times) - 1)

        line_algo.set_data(algo_xy[:idx + 1, 0], algo_xy[:idx + 1, 1])
        dot_algo.set_data([algo_xy[idx, 0]], [algo_xy[idx, 1]])

        line_gt_anim.set_data(gt_at_algo[:idx + 1, 0],
                              gt_at_algo[:idx + 1, 1])
        dot_gt.set_data([gt_at_algo[idx, 0]], [gt_at_algo[idx, 1]])

        line_error.set_data(
            [algo_xy[idx, 0], gt_at_algo[idx, 0]],
            [algo_xy[idx, 1], gt_at_algo[idx, 1]])

        line_err.set_data(algo_times[:idx + 1], radial_errors[:idx + 1])

        t = algo_times[idx]
        err = radial_errors[idx]
        time_text.set_text(f't = {t:.1f} s  |  err = {err * 100:.1f} cm')

        return (line_algo, dot_algo, line_gt_anim, dot_gt,
                line_error, line_err, time_text)

    anim = animation.FuncAnimation(                                        # noqa: F841
        fig, animate, init_func=init,
        frames=total_frames, interval=20, blit=True)

    plt.tight_layout()

    if args.save:
        animate(total_frames - 1)
        fig.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"  Saved static plot to {args.save}")

    plt.show()


# ─── Lap-1 Static Plot ───────────────────────────────────────────────────────

def save_first_lap_plot(l, one_lap_time,
                        algo_times, algo_xy, gt_at_algo,
                        radial_errors, cep50, passed, args):
    """
    Save a clean static PNG showing lap 1 only:
    ground-truth path vs algorithm output, with error stats.
    Filename is derived from the data file (or 'demo' if in demo mode).
    """
    # Filename: same directory as data file, stem + _lap1_comparison.png
    if args.demo:
        out_path = 'demo_lap1_comparison.png'
    else:
        import os
        base = os.path.splitext(os.path.abspath(args.data))[0]
        out_path = base + '_lap1_comparison.png'

    # Mask to first lap only
    lap1_mask = algo_times <= one_lap_time
    if not np.any(lap1_mask):
        print("  [WARN] No samples in first lap — skipping lap-1 plot.")
        return

    t1 = algo_times[lap1_mask]
    xy1 = algo_xy[lap1_mask]
    gt1 = gt_at_algo[lap1_mask]
    err1 = radial_errors[lap1_mask]
    cep50_lap1 = np.median(np.sqrt((xy1[:, 0] - gt1[:, 0])**2 +
                                   (xy1[:, 1] - gt1[:, 1])**2))
    passed_lap1 = cep50_lap1 <= 0.05

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle(f'Lap 1 Path Comparison — {l}m × {l}m Square', fontsize=13)

    # ── Left: path comparison ────────────────────────────────────────────
    ax = axes[0]
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Ground Truth vs Algorithm (Lap 1)')
    ax.grid(True, alpha=0.3)

    ax.plot(gt1[:, 0], gt1[:, 1], 'g-', linewidth=2.5, label='Ground Truth')
    ax.plot(xy1[:, 0], xy1[:, 1], 'r-', linewidth=1.5, alpha=0.8,
            label='Algorithm')

    # Error lines sampled every ~2% of path
    step = max(1, len(t1) // 50)
    for i in range(0, len(t1), step):
        ax.plot([xy1[i, 0], gt1[i, 0]], [xy1[i, 1], gt1[i, 1]],
                'k-', linewidth=0.5, alpha=0.3)

    ax.plot(gt1[0, 0], gt1[0, 1], '*', color='green', markersize=14,
            zorder=5, label='Start')
    ax.plot(gt1[-1, 0], gt1[-1, 1], 's', color='green', markersize=9,
            zorder=5, label='End (GT)')
    ax.plot(xy1[-1, 0], xy1[-1, 1], 's', color='red', markersize=9,
            zorder=5, label='End (Algo)')

    margin = l * 0.15
    ax.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax.set_ylim(-margin, l + margin)
    ax.legend(loc='upper left', fontsize=8)

    # ── Right: error over time + stats ───────────────────────────────────
    ax2 = axes[1]
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('2D Position Error [m]')
    ax2.set_title('Radial Error — Lap 1')
    ax2.grid(True, alpha=0.3)
    ax2.plot(t1, err1, 'b-', linewidth=1, alpha=0.8, label='Error')
    ax2.axhline(y=0.05, color='r', linestyle='--', alpha=0.6,
                label='CEP50 threshold (5 cm)')
    ax2.axhline(y=cep50_lap1, color='orange', linestyle='-', alpha=0.8,
                label=f'Lap-1 CEP50 = {cep50_lap1 * 100:.2f} cm')
    ax2.set_xlim(0, t1[-1])
    ax2.set_ylim(0, max(np.max(err1) * 1.2, 0.1))
    ax2.legend(fontsize=8)

    status = 'PASS' if passed_lap1 else 'FAIL'
    colour = 'green' if passed_lap1 else 'red'
    info = (f"Lap 1 Results\n"
            f"{'─'*26}\n"
            f"Samples:     {len(t1)}\n"
            f"Duration:    {t1[-1]:.1f} s\n"
            f"{'─'*26}\n"
            f"CEP50:       {cep50_lap1*100:.2f} cm\n"
            f"Mean err:    {np.mean(err1)*100:.2f} cm\n"
            f"Max err:     {np.max(err1)*100:.2f} cm\n"
            f"Std err:     {np.std(err1)*100:.2f} cm\n"
            f"{'─'*26}\n"
            f"Overall CEP50: {cep50*100:.2f} cm")
    ax2.text(0.98, 0.97, info, transform=ax2.transAxes,
             fontsize=8.5, verticalalignment='top', horizontalalignment='right',
             fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))
    ax2.text(0.98, 0.05, f'  {status}', transform=ax2.transAxes,
             fontsize=18, fontweight='bold', color=colour,
             horizontalalignment='right', fontfamily='monospace')

    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved lap-1 comparison plot to: {out_path}")


# ─── Main ───────────────────────────────────────────────────────────────────

def run_test(args):
    """Execute the full test pipeline."""
    l = args.side_length
    r = l / 4.0
    v_s = args.straight_speed
    v_t = args.turn_speed
    num_laps = args.laps

    print()
    print("=" * 55)
    print(f"  Dynamic Accuracy Test - {l}m x {l}m Square Path")
    print("=" * 55)
    print(f"  Turn radius:    {r:.2f} m")
    print(f"  Straight speed: {v_s} km/h ({v_s / 3.6:.3f} m/s)")
    print(f"  Turn speed:     {v_t} km/h ({v_t / 3.6:.3f} m/s)")
    print(f"  Laps:           {num_laps}")

    # Build ground truth
    gt_times, gt_positions, one_lap_time = build_ground_truth(
        l, v_s, v_t, num_laps)

    total_straight = 4 * (l - 2 * r)
    total_arc = 2 * np.pi * r
    lap_dist = total_straight + total_arc

    print(f"  Lap distance:   {lap_dist:.2f} m")
    print(f"  Lap time:       {one_lap_time:.2f} s")
    print(f"  Total time:     {one_lap_time * num_laps:.2f} s")
    print()

    # Load / generate algorithm data
    if args.demo:
        print("  [DEMO MODE] Generating synthetic algorithm data ...")
        algo_times, algo_pos_3d = generate_demo_data(gt_times, gt_positions)
    else:
        print(f"  Loading data from: {args.data}")
        algo_times, algo_pos_3d = load_csv_data(args.data)

    algo_xy = algo_pos_3d[:, :2]

    # Clip algorithm times to ground-truth range
    mask = algo_times <= gt_times[-1]
    algo_times = algo_times[mask]
    algo_xy = algo_xy[mask]

    # Interpolate ground truth at algorithm sample times
    gt_at_algo = interpolate_ground_truth(algo_times, gt_times, gt_positions)

    # Compute errors
    errors_2d = algo_xy - gt_at_algo
    cep50, radial_errors = compute_cep50(errors_2d)

    # Pass / fail
    CEP_THRESHOLD = 0.05
    passed = cep50 <= CEP_THRESHOLD
    tag = "PASS" if passed else "FAIL"

    print("-" * 55)
    print(f"  CEP50:          {cep50 * 100:.2f} cm  ({cep50:.4f} m)")
    print(f"  Mean error:     {np.mean(radial_errors) * 100:.2f} cm")
    print(f"  Max error:      {np.max(radial_errors) * 100:.2f} cm")
    print(f"  Std error:      {np.std(radial_errors) * 100:.2f} cm")
    print(f"  Samples:        {len(algo_times)}")
    print(f"  Result:         {tag}  (threshold: {CEP_THRESHOLD * 100:.0f} cm)")
    print("-" * 55)
    print()

    # Save first-lap static comparison plot
    save_first_lap_plot(l, one_lap_time,
                        algo_times, algo_xy, gt_at_algo,
                        radial_errors, cep50, passed, args)

    # Visualise
    create_visualisation(l, gt_times, gt_positions,
                         algo_times, algo_xy, gt_at_algo,
                         radial_errors, cep50, passed, args)


def run_ground_truth_only(args):
    """Show just the ground-truth path with an animated dot tracing it."""
    l = args.side_length
    r = l / 4.0
    v_s = args.straight_speed
    v_t = args.turn_speed
    num_laps = args.laps

    print()
    print("=" * 55)
    print(f"  Ground Truth Path - {l}m x {l}m Square")
    print("=" * 55)
    print(f"  Turn radius:    {r:.2f} m")
    print(f"  Straight speed: {v_s} km/h ({v_s / 3.6:.3f} m/s)")
    print(f"  Turn speed:     {v_t} km/h ({v_t / 3.6:.3f} m/s)")
    print(f"  Laps:           {num_laps}")

    gt_times, gt_positions, one_lap_time = build_ground_truth(
        l, v_s, v_t, num_laps)

    total_straight = 4 * (l - 2 * r)
    total_arc = 2 * np.pi * r
    lap_dist = total_straight + total_arc

    print(f"  Lap distance:   {lap_dist:.2f} m")
    print(f"  Lap time:       {one_lap_time:.2f} s")
    print(f"  Total time:     {one_lap_time * num_laps:.2f} s")
    print()

    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    ax.set_aspect('equal')
    ax.set_xlabel('x-position [m]')
    ax.set_ylabel('y-position [m]')
    ax.set_title(f'Ground Truth Path - {l}m x {l}m  '
                 f'(r={r:.2f}m, {num_laps} laps)')
    ax.grid(True, alpha=0.3)

    # Full path outline
    ax.plot(gt_positions[:, 0], gt_positions[:, 1],
            'g-', linewidth=2, alpha=0.35, label='Full path')

    # One-lap highlight
    one_lap_n = int(len(gt_positions) / num_laps) + 1
    ax.plot(gt_positions[:one_lap_n, 0], gt_positions[:one_lap_n, 1],
            'g-', linewidth=2.5, alpha=0.8, label='Single lap')

    # Origin
    ax.plot(0, 0, '*', color='green', markersize=18, zorder=5,
            label='Origin (0, 0)')

    # Corner centres
    half = l / 2.0
    corners = [(half - r, r), (half - r, l - r),
               (-half + r, l - r), (-half + r, r)]
    for i, (cx, cy) in enumerate(corners):
        ax.plot(cx, cy, 'x', color='gray', markersize=8, zorder=4)
        ax.add_patch(plt.Circle((cx, cy), r, fill=False,
                                linestyle=':', color='gray', alpha=0.4))

    # Animated dot tracing the path
    dot, = ax.plot([], [], 'go', markersize=10, zorder=6)
    trail, = ax.plot([], [], 'g-', linewidth=2, alpha=0.7)

    margin = l * 0.15
    ax.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax.set_ylim(-margin, l + margin)
    ax.legend(loc='upper left', fontsize=9)

    time_text = ax.text(
        0.02, 0.98, '', transform=ax.transAxes, fontsize=10,
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # Dimensions annotation
    ax.annotate('', xy=(half, -margin * 0.5), xytext=(-half, -margin * 0.5),
                arrowprops=dict(arrowstyle='<->', color='dimgray', lw=1.5))
    ax.text(0, -margin * 0.5 - margin * 0.15, f'{l} m', ha='center',
            fontsize=9, color='dimgray')

    max_frames = 500
    frame_step = max(1, len(gt_positions) // max_frames)
    total_frames = len(gt_positions) // frame_step

    def init():
        dot.set_data([], [])
        trail.set_data([], [])
        time_text.set_text('')
        return dot, trail, time_text

    def animate(frame):
        idx = min(frame * frame_step, len(gt_positions) - 1)
        dot.set_data([gt_positions[idx, 0]], [gt_positions[idx, 1]])
        trail.set_data(gt_positions[:idx + 1, 0], gt_positions[:idx + 1, 1])
        t = gt_times[idx]
        lap_num = int(t // one_lap_time) + 1
        time_text.set_text(f't = {t:.1f} s  |  lap {lap_num}/{num_laps}')
        return dot, trail, time_text

    anim = animation.FuncAnimation(                                        # noqa: F841
        fig, animate, init_func=init,
        frames=total_frames, interval=20, blit=True)

    plt.tight_layout()

    if args.save:
        animate(total_frames - 1)
        fig.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"  Saved plot to {args.save}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Dynamic Accuracy Test - Square-Path Traversal '
                    '(Section 4.1)')

    parser.add_argument('--data', type=str,
                        help='Path to algorithm output CSV file')
    parser.add_argument('--side-length', type=float, required=True,
                        help='Square side length in metres (3, 4, or 5)')
    parser.add_argument('--straight-speed', type=float, default=5.0,
                        help='Straight-segment speed in km/h (default: 5)')
    parser.add_argument('--turn-speed', type=float, default=2.0,
                        help='Turn-segment speed in km/h (default: 2)')
    parser.add_argument('--laps', type=int, default=5,
                        help='Number of laps (default: 5)')
    parser.add_argument('--demo', action='store_true',
                        help='Run with synthetic demo data')
    parser.add_argument('--ground-truth-only', action='store_true',
                        help='Show only the ground-truth path (no data needed)')
    parser.add_argument('--save', type=str, default=None,
                        help='Save final plot to file (e.g. results.png)')

    args = parser.parse_args()

    if not args.demo and not args.data and not args.ground_truth_only:
        parser.error('One of --data <csv_file>, --demo, or --ground-truth-only is required')

    if args.ground_truth_only:
        run_ground_truth_only(args)
    else:
        run_test(args)


if __name__ == '__main__':
    main()
