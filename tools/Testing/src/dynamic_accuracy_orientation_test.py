#!/usr/bin/env python3
"""
Dynamic Accuracy Test with Orientation - Square-Path Traversal (Section 4.1)

Same as dynamic_accuracy_test.py but also computes and visualises heading
(yaw) accuracy. Since the robot always moves forward, the ground-truth
heading is derived from the path tangent direction at each point.

Usage:
    # With real data (CSV must include roll, pitch, yaw columns):
    python dynamic_accuracy_orientation_test.py --data log.csv --side-length 5

    # Demo mode:
    python dynamic_accuracy_orientation_test.py --demo --side-length 5

    # Ground truth only (shows path + heading arrows):
    python dynamic_accuracy_orientation_test.py --ground-truth-only --side-length 5

CSV format expected (header row required):
    timestamp_ms, x, y, z, vx, vy, vz, roll, pitch, yaw
    Positions in meters, angles in degrees, timestamp in milliseconds.
"""

import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch


# ─── Ground-Truth Path + Heading Construction ────────────────────────────────

def build_ground_truth(l, straight_speed_kmh, turn_speed_kmh, num_laps,
                       points_per_meter=50):
    """
    Build a time-parameterised ground-truth rounded-square path with heading.

    The robot always faces forward, so heading = atan2(dy, dx) of the path
    tangent. Returns heading in degrees, wrapped to [-180, 180].

    Returns
    -------
    gt_times : ndarray (N,)
    gt_positions : ndarray (N, 2)
    gt_headings : ndarray (N,) in degrees
    one_lap_time : float
    """
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

        else:  # arc
            _, cx, cy, radius, a0, a1 = seg
            arc_len = abs(a1 - a0) * radius
            n = max(int(arc_len * points_per_meter), 2)
            t_seg = arc_len / v_t
            ts = np.linspace(0, t_seg, n)
            angles = np.linspace(a0, a1, n)
            xs = cx + radius * np.cos(angles)
            ys = cy + radius * np.sin(angles)
            # Heading is tangent to circle: perpendicular to radius
            # For CCW arc (a1 > a0), tangent = angle + pi/2
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
    # Wrap to [-180, 180]
    gt_headings = (gt_headings + 180) % 360 - 180

    return (np.concatenate(gt_times_list),
            np.vstack(gt_pos_list),
            gt_headings,
            one_lap_time)


# ─── Interpolation & Metrics ────────────────────────────────────────────────

def interpolate_ground_truth(query_times, gt_times, gt_positions, gt_headings):
    """Interpolate position and heading. Heading uses circular interpolation."""
    x_interp = np.interp(query_times, gt_times, gt_positions[:, 0])
    y_interp = np.interp(query_times, gt_times, gt_positions[:, 1])

    # Circular interpolation for heading via sin/cos decomposition
    hdg_rad = np.radians(gt_headings)
    sin_interp = np.interp(query_times, gt_times, np.sin(hdg_rad))
    cos_interp = np.interp(query_times, gt_times, np.cos(hdg_rad))
    hdg_interp = np.degrees(np.arctan2(sin_interp, cos_interp))

    return np.column_stack([x_interp, y_interp]), hdg_interp


def wrap_angle_deg(angle):
    """Wrap angle to [-180, 180] degrees."""
    return (angle + 180) % 360 - 180


def compute_cep50(errors_2d):
    """CEP50: median of 2D radial position errors."""
    radial = np.hypot(errors_2d[:, 0], errors_2d[:, 1])
    return float(np.median(radial)), radial


def compute_heading_error_stats(heading_errors_deg):
    """Compute heading error statistics from wrapped errors in degrees."""
    abs_err = np.abs(heading_errors_deg)
    return {
        'median': float(np.median(abs_err)),
        'mean': float(np.mean(abs_err)),
        'max': float(np.max(abs_err)),
        'std': float(np.std(abs_err)),
    }


# ─── Data I/O ───────────────────────────────────────────────────────────────

def load_csv_data(filepath):
    """
    Load algorithm output CSV.

    Expected: timestamp_ms, x, y, z, vx, vy, vz, roll, pitch, yaw
    Returns timestamps (s), positions (m), and yaw (degrees).
    """
    data = np.genfromtxt(filepath, delimiter=',', skip_header=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    timestamps_s = (data[:, 0] - data[0, 0]) / 1000.0
    positions = data[:, 1:4]

    if data.shape[1] >= 10:
        yaw = data[:, 9]
    else:
        print("  WARNING: No yaw column found, defaulting to 0")
        yaw = np.zeros(len(timestamps_s))

    return timestamps_s, positions, yaw


def generate_demo_data(gt_times, gt_positions, gt_headings,
                       pos_noise_std=0.02, pos_drift_rate=0.0004,
                       hdg_noise_std=3.0, hdg_drift_rate=0.02,
                       sample_hz=50):
    """Generate synthetic algorithm output with noise + drift on pos & heading."""
    dt = 1.0 / sample_hz
    sample_times = np.arange(0, gt_times[-1], dt)
    gt_pos_interp, gt_hdg_interp = interpolate_ground_truth(
        sample_times, gt_times, gt_positions, gt_headings)

    # Position: noise + drift
    noise = np.random.normal(0, pos_noise_std, gt_pos_interp.shape)
    drift = pos_drift_rate * sample_times[:, np.newaxis] * np.array([1.0, 0.5])
    algo_xy = gt_pos_interp + noise + drift
    z = np.random.normal(0, 0.005, len(sample_times))

    # Heading: noise + slow drift
    hdg_noise = np.random.normal(0, hdg_noise_std, len(sample_times))
    hdg_drift = hdg_drift_rate * sample_times
    algo_yaw = wrap_angle_deg(gt_hdg_interp + hdg_noise + hdg_drift)

    return sample_times, np.column_stack([algo_xy, z]), algo_yaw


# ─── Visualisation ──────────────────────────────────────────────────────────

def draw_heading_arrow(ax, x, y, heading_deg, color, length=0.15):
    """Draw a heading arrow at (x, y)."""
    dx = length * np.cos(np.radians(heading_deg))
    dy = length * np.sin(np.radians(heading_deg))
    ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=1.5))


def create_visualisation(l, gt_times, gt_positions, gt_headings,
                         algo_times, algo_xy, algo_yaw,
                         gt_at_algo, gt_hdg_at_algo,
                         radial_errors, heading_errors,
                         cep50, hdg_stats, passed, args):
    """Animated dashboard with position and heading comparison."""

    fig = plt.figure(figsize=(18, 10))
    gs = fig.add_gridspec(3, 3, width_ratios=[2, 1, 1],
                          hspace=0.4, wspace=0.35)

    # ── Main path plot ───────────────────────────────────────────────────
    ax_path = fig.add_subplot(gs[:2, 0])
    ax_path.set_aspect('equal')
    ax_path.set_xlabel('x-position [m]')
    ax_path.set_ylabel('y-position [m]')
    ax_path.set_title(f'Dynamic Accuracy + Orientation \u2014 {l}m Square Path')
    ax_path.grid(True, alpha=0.3)

    ax_path.plot(gt_positions[:, 0], gt_positions[:, 1],
                 'g-', alpha=0.25, linewidth=2, label='Ground Truth Path')
    ax_path.plot(0, 0, '*', color='green', markersize=15,
                 zorder=5, label='Origin')

    line_algo, = ax_path.plot([], [], 'r-', linewidth=1.5, alpha=0.8,
                              label='Algorithm')
    dot_algo, = ax_path.plot([], [], 'ro', markersize=6, zorder=5)
    line_gt_anim, = ax_path.plot([], [], 'g-', linewidth=1.5, alpha=0.8)
    dot_gt, = ax_path.plot([], [], 'go', markersize=6, zorder=5)
    line_error, = ax_path.plot([], [], 'k--', linewidth=0.8, alpha=0.5)

    # Heading arrows (updated per frame)
    arrow_len = l * 0.04
    arrow_gt = ax_path.annotate(
        '', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='green', lw=2))
    arrow_algo = ax_path.annotate(
        '', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='red', lw=2))

    margin = l * 0.15
    ax_path.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax_path.set_ylim(-margin, l + margin)
    ax_path.legend(loc='upper left', fontsize=8)

    time_text = ax_path.text(
        0.02, 0.98, '', transform=ax_path.transAxes, fontsize=9,
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # ── Position error vs time ───────────────────────────────────────────
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

    # ── Heading error vs time ────────────────────────────────────────────
    ax_hdg = fig.add_subplot(gs[1, 1:])
    ax_hdg.set_xlabel('Time [s]')
    ax_hdg.set_ylabel('Heading Error [\u00b0]')
    ax_hdg.set_title('Heading Error vs Time')
    ax_hdg.grid(True, alpha=0.3)
    ax_hdg.axhline(y=hdg_stats['median'], color='orange', linestyle='-',
                   alpha=0.7,
                   label=f'Median |err| = {hdg_stats["median"]:.1f}\u00b0')
    line_hdg, = ax_hdg.plot([], [], 'm-', linewidth=0.8, alpha=0.8)
    ax_hdg.set_xlim(0, algo_times[-1])
    max_hdg_err = max(np.max(np.abs(heading_errors)) * 1.2, 10)
    ax_hdg.set_ylim(-max_hdg_err, max_hdg_err)
    ax_hdg.legend(fontsize=8)

    # ── Heading comparison (polar-ish) ───────────────────────────────────
    ax_yaw = fig.add_subplot(gs[2, 0])
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.set_ylabel('Heading [\u00b0]')
    ax_yaw.set_title('Heading: Ground Truth vs Algorithm')
    ax_yaw.grid(True, alpha=0.3)
    line_yaw_gt, = ax_yaw.plot([], [], 'g-', linewidth=1, alpha=0.8,
                               label='GT Heading')
    line_yaw_algo, = ax_yaw.plot([], [], 'r-', linewidth=1, alpha=0.8,
                                 label='Algo Heading')
    ax_yaw.set_xlim(0, algo_times[-1])
    ax_yaw.set_ylim(-200, 200)
    ax_yaw.legend(fontsize=8)

    # ── Error histograms ─────────────────────────────────────────────────
    ax_hist_pos = fig.add_subplot(gs[2, 1])
    ax_hist_pos.hist(radial_errors * 100, bins=40,
                     color='steelblue', alpha=0.7, edgecolor='white')
    ax_hist_pos.axvline(x=cep50 * 100, color='orange', linewidth=2,
                        label=f'CEP50 = {cep50 * 100:.2f} cm')
    ax_hist_pos.axvline(x=5.0, color='r', linewidth=1.5, linestyle='--',
                        label='Threshold = 5 cm')
    ax_hist_pos.set_xlabel('Position Error [cm]')
    ax_hist_pos.set_ylabel('Count')
    ax_hist_pos.set_title('Position Error Distribution')
    ax_hist_pos.legend(fontsize=7)

    # ── Stats panel ──────────────────────────────────────────────────────
    ax_stats = fig.add_subplot(gs[2, 2])
    ax_stats.axis('off')

    status_text = 'PASS' if passed else 'FAIL'
    status_colour = 'green' if passed else 'red'

    r_turn = l / 4.0
    total_straight = 4 * (l - 2 * r_turn)
    total_arc = 2 * np.pi * r_turn
    lap_dist = total_straight + total_arc

    stats = (
        f"Test Results\n"
        f"{'─' * 32}\n"
        f"Side length:     {l} m\n"
        f"Turn radius:     {r_turn:.2f} m\n"
        f"Lap distance:    {lap_dist:.2f} m\n"
        f"Laps:            {args.laps}\n"
        f"Samples:         {len(algo_times)}\n"
        f"{'─' * 32}\n"
        f"Position:\n"
        f"  CEP50:         {cep50 * 100:.2f} cm\n"
        f"  Mean error:    {np.mean(radial_errors) * 100:.2f} cm\n"
        f"  Max error:     {np.max(radial_errors) * 100:.2f} cm\n"
        f"{'─' * 32}\n"
        f"Heading:\n"
        f"  Median |err|:  {hdg_stats['median']:.2f}\u00b0\n"
        f"  Mean |err|:    {hdg_stats['mean']:.2f}\u00b0\n"
        f"  Max |err|:     {hdg_stats['max']:.2f}\u00b0\n"
        f"  Std |err|:     {hdg_stats['std']:.2f}\u00b0\n"
        f"{'─' * 32}\n"
    )
    ax_stats.text(0.05, 0.95, stats, transform=ax_stats.transAxes,
                  fontsize=8.5, verticalalignment='top', fontfamily='monospace')
    ax_stats.text(0.05, 0.02, f"  {status_text}",
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
        line_hdg.set_data([], [])
        line_yaw_gt.set_data([], [])
        line_yaw_algo.set_data([], [])
        time_text.set_text('')
        return (line_algo, dot_algo, line_gt_anim, dot_gt,
                line_error, line_err, line_hdg, line_yaw_gt,
                line_yaw_algo, time_text)

    def animate(frame):
        idx = min(frame * frame_step, len(algo_times) - 1)

        # Position trails
        line_algo.set_data(algo_xy[:idx + 1, 0], algo_xy[:idx + 1, 1])
        dot_algo.set_data([algo_xy[idx, 0]], [algo_xy[idx, 1]])
        line_gt_anim.set_data(gt_at_algo[:idx + 1, 0],
                              gt_at_algo[:idx + 1, 1])
        dot_gt.set_data([gt_at_algo[idx, 0]], [gt_at_algo[idx, 1]])
        line_error.set_data(
            [algo_xy[idx, 0], gt_at_algo[idx, 0]],
            [algo_xy[idx, 1], gt_at_algo[idx, 1]])

        # Heading arrows
        for a in [arrow_gt, arrow_algo]:
            a.set_visible(False)

        gx, gy = gt_at_algo[idx]
        ax_, ay = algo_xy[idx]
        gt_h = gt_hdg_at_algo[idx]
        al_h = algo_yaw[idx]

        # Re-draw arrows by updating position
        arrow_gt.xy = (gx + arrow_len * np.cos(np.radians(gt_h)),
                       gy + arrow_len * np.sin(np.radians(gt_h)))
        arrow_gt.set_position((gx, gy))
        arrow_gt.set_visible(True)

        arrow_algo.xy = (ax_ + arrow_len * np.cos(np.radians(al_h)),
                         ay + arrow_len * np.sin(np.radians(al_h)))
        arrow_algo.set_position((ax_, ay))
        arrow_algo.set_visible(True)

        # Error plots
        line_err.set_data(algo_times[:idx + 1], radial_errors[:idx + 1])
        line_hdg.set_data(algo_times[:idx + 1], heading_errors[:idx + 1])
        line_yaw_gt.set_data(algo_times[:idx + 1], gt_hdg_at_algo[:idx + 1])
        line_yaw_algo.set_data(algo_times[:idx + 1], algo_yaw[:idx + 1])

        t = algo_times[idx]
        err = radial_errors[idx]
        herr = heading_errors[idx]
        time_text.set_text(
            f't = {t:.1f} s  |  pos err = {err * 100:.1f} cm  '
            f'|  hdg err = {herr:.1f}\u00b0')

        return (line_algo, dot_algo, line_gt_anim, dot_gt,
                line_error, line_err, line_hdg, line_yaw_gt,
                line_yaw_algo, time_text)

    anim = animation.FuncAnimation(                                        # noqa: F841
        fig, animate, init_func=init,
        frames=total_frames, interval=20, blit=False)

    plt.tight_layout()

    if args.save:
        animate(total_frames - 1)
        fig.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"  Saved static plot to {args.save}")

    plt.show()


# ─── Ground-Truth Only Visualisation ────────────────────────────────────────

def create_gt_only_visualisation(l, gt_times, gt_positions, gt_headings,
                                 one_lap_time, args):
    """Animated ground-truth path with heading arrows."""
    r = l / 4.0
    half = l / 2.0
    num_laps = args.laps

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.set_aspect('equal')
    ax.set_xlabel('x-position [m]')
    ax.set_ylabel('y-position [m]')
    ax.set_title(f'Ground Truth Path + Heading \u2014 {l}m x {l}m  '
                 f'(r={r:.2f}m, {num_laps} laps)')
    ax.grid(True, alpha=0.3)

    ax.plot(gt_positions[:, 0], gt_positions[:, 1],
            'g-', linewidth=2, alpha=0.3, label='Full path')

    one_lap_n = int(len(gt_positions) / num_laps) + 1
    ax.plot(gt_positions[:one_lap_n, 0], gt_positions[:one_lap_n, 1],
            'g-', linewidth=2.5, alpha=0.8, label='Single lap')

    ax.plot(0, 0, '*', color='green', markersize=18, zorder=5,
            label='Origin (0, 0)')

    # Static heading arrows sampled along path
    arrow_step = max(1, one_lap_n // 20)
    arrow_len_static = l * 0.03
    for i in range(0, one_lap_n, arrow_step):
        x, y = gt_positions[i]
        h = gt_headings[i]
        dx = arrow_len_static * np.cos(np.radians(h))
        dy = arrow_len_static * np.sin(np.radians(h))
        ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                    arrowprops=dict(arrowstyle='->', color='darkgreen',
                                    lw=1.2, alpha=0.6))

    corners = [(half - r, r), (half - r, l - r),
               (-half + r, l - r), (-half + r, r)]
    for cx, cy in corners:
        ax.plot(cx, cy, 'x', color='gray', markersize=8, zorder=4)
        ax.add_patch(plt.Circle((cx, cy), r, fill=False,
                                linestyle=':', color='gray', alpha=0.4))

    dot, = ax.plot([], [], 'go', markersize=10, zorder=6)
    trail, = ax.plot([], [], 'g-', linewidth=2, alpha=0.7)

    arrow_len_anim = l * 0.06
    arrow_anim = ax.annotate(
        '', xy=(0, 0), xytext=(0, 0),
        arrowprops=dict(arrowstyle='->', color='darkgreen', lw=2.5))

    margin = l * 0.15
    ax.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax.set_ylim(-margin, l + margin)
    ax.legend(loc='upper left', fontsize=9)

    time_text = ax.text(
        0.02, 0.98, '', transform=ax.transAxes, fontsize=10,
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

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
        px, py = gt_positions[idx]
        h = gt_headings[idx]

        dot.set_data([px], [py])
        trail.set_data(gt_positions[:idx + 1, 0], gt_positions[:idx + 1, 1])

        arrow_anim.xy = (px + arrow_len_anim * np.cos(np.radians(h)),
                         py + arrow_len_anim * np.sin(np.radians(h)))
        arrow_anim.set_position((px, py))

        t = gt_times[idx]
        lap_num = int(t // one_lap_time) + 1
        time_text.set_text(
            f't = {t:.1f} s  |  lap {lap_num}/{num_laps}  '
            f'|  hdg = {h:.1f}\u00b0')
        return dot, trail, time_text

    anim = animation.FuncAnimation(                                        # noqa: F841
        fig, animate, init_func=init,
        frames=total_frames, interval=20, blit=False)

    plt.tight_layout()

    if args.save:
        animate(total_frames - 1)
        fig.savefig(args.save, dpi=150, bbox_inches='tight')
        print(f"  Saved plot to {args.save}")

    plt.show()


# ─── Main ───────────────────────────────────────────────────────────────────

# ─── Lap-1 Static Plot ───────────────────────────────────────────────────────

def save_first_lap_plot(l, one_lap_time,
                        algo_times, algo_xy, algo_yaw,
                        gt_at_algo, gt_hdg_at_algo,
                        radial_errors, heading_errors,
                        cep50, hdg_stats, passed, args):
    """
    Save a clean static PNG showing lap 1 only:
    ground-truth vs algorithm path + heading, with error stats.
    """
    if args.demo:
        out_path = 'demo_lap1_orientation_comparison.png'
    else:
        import os
        base = os.path.splitext(os.path.abspath(args.data))[0]
        out_path = base + '_lap1_orientation_comparison.png'

    lap1_mask = algo_times <= one_lap_time
    if not np.any(lap1_mask):
        print("  [WARN] No samples in first lap — skipping lap-1 plot.")
        return

    t1 = algo_times[lap1_mask]
    xy1 = algo_xy[lap1_mask]
    yaw1 = algo_yaw[lap1_mask]
    gt1 = gt_at_algo[lap1_mask]
    gth1 = gt_hdg_at_algo[lap1_mask]
    err1 = radial_errors[lap1_mask]
    herr1 = heading_errors[lap1_mask]

    cep50_lap1 = np.median(np.sqrt((xy1[:, 0] - gt1[:, 0])**2 +
                                   (xy1[:, 1] - gt1[:, 1])**2))
    hdg_med_lap1 = np.median(np.abs(herr1))
    passed_lap1 = cep50_lap1 <= 0.05

    fig, axes = plt.subplots(2, 2, figsize=(15, 11))
    fig.suptitle(
        f'Lap 1 Path + Orientation Comparison — {l}m × {l}m Square',
        fontsize=13)

    # ── Top-left: path comparison ─────────────────────────────────────────
    ax = axes[0, 0]
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Path: Ground Truth vs Algorithm (Lap 1)')
    ax.grid(True, alpha=0.3)
    ax.plot(gt1[:, 0], gt1[:, 1], 'g-', linewidth=2.5, label='Ground Truth')
    ax.plot(xy1[:, 0], xy1[:, 1], 'r-', linewidth=1.5, alpha=0.8,
            label='Algorithm')

    # Heading arrows every ~5% of path
    arrow_len = l * 0.04
    step = max(1, len(t1) // 20)
    for i in range(0, len(t1), step):
        for pos, hdg, col in [(gt1[i], gth1[i], 'green'),
                              (xy1[i], yaw1[i], 'red')]:
            dx = arrow_len * np.cos(np.radians(hdg))
            dy = arrow_len * np.sin(np.radians(hdg))
            ax.annotate('', xy=(pos[0] + dx, pos[1] + dy),
                        xytext=(pos[0], pos[1]),
                        arrowprops=dict(arrowstyle='->', color=col,
                                        lw=1.2, alpha=0.5))

    # Error lines
    err_step = max(1, len(t1) // 50)
    for i in range(0, len(t1), err_step):
        ax.plot([xy1[i, 0], gt1[i, 0]], [xy1[i, 1], gt1[i, 1]],
                'k-', linewidth=0.5, alpha=0.25)

    ax.plot(gt1[0, 0], gt1[0, 1], '*', color='green', markersize=14,
            zorder=5, label='Start')
    margin = l * 0.15
    ax.set_xlim(-l / 2 - margin, l / 2 + margin)
    ax.set_ylim(-margin, l + margin)
    ax.legend(loc='upper left', fontsize=8)

    # ── Top-right: position error vs time ────────────────────────────────
    ax2 = axes[0, 1]
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('2D Position Error [m]')
    ax2.set_title('Radial Position Error — Lap 1')
    ax2.grid(True, alpha=0.3)
    ax2.plot(t1, err1, 'b-', linewidth=1, alpha=0.8)
    ax2.axhline(y=0.05, color='r', linestyle='--', alpha=0.6,
                label='Threshold (5 cm)')
    ax2.axhline(y=cep50_lap1, color='orange', linestyle='-', alpha=0.8,
                label=f'CEP50 = {cep50_lap1 * 100:.2f} cm')
    ax2.set_xlim(0, t1[-1])
    ax2.set_ylim(0, max(np.max(err1) * 1.2, 0.1))
    ax2.legend(fontsize=8)

    # ── Bottom-left: heading comparison ──────────────────────────────────
    ax3 = axes[1, 0]
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Heading [°]')
    ax3.set_title('Heading: Ground Truth vs Algorithm — Lap 1')
    ax3.grid(True, alpha=0.3)
    ax3.plot(t1, gth1, 'g-', linewidth=1.5, alpha=0.8, label='GT Heading')
    ax3.plot(t1, yaw1, 'r-', linewidth=1.2, alpha=0.8, label='Algo Heading')
    ax3.set_xlim(0, t1[-1])
    ax3.set_ylim(-200, 200)
    ax3.legend(fontsize=8)

    # ── Bottom-right: heading error + stats ──────────────────────────────
    ax4 = axes[1, 1]
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Heading Error [°]')
    ax4.set_title('Heading Error — Lap 1')
    ax4.grid(True, alpha=0.3)
    ax4.plot(t1, herr1, 'm-', linewidth=1, alpha=0.8)
    ax4.axhline(y=0, color='gray', linestyle='-', alpha=0.4)
    ax4.axhline(y=hdg_med_lap1, color='orange', linestyle='-', alpha=0.8,
                label=f'Median |err| = {hdg_med_lap1:.1f}°')
    ax4.axhline(y=-hdg_med_lap1, color='orange', linestyle='-', alpha=0.8)
    ax4.set_xlim(0, t1[-1])
    max_he = max(np.max(np.abs(herr1)) * 1.2, 10)
    ax4.set_ylim(-max_he, max_he)
    ax4.legend(fontsize=8)

    status = 'PASS' if passed_lap1 else 'FAIL'
    colour = 'green' if passed_lap1 else 'red'
    info = (f"Lap 1 Results\n"
            f"{'─'*30}\n"
            f"Samples:       {len(t1)}\n"
            f"Duration:      {t1[-1]:.1f} s\n"
            f"{'─'*30}\n"
            f"Position:\n"
            f"  CEP50:       {cep50_lap1*100:.2f} cm\n"
            f"  Mean err:    {np.mean(err1)*100:.2f} cm\n"
            f"  Max err:     {np.max(err1)*100:.2f} cm\n"
            f"{'─'*30}\n"
            f"Heading:\n"
            f"  Median |err|:{hdg_med_lap1:.1f}°\n"
            f"  Max |err|:   {np.max(np.abs(herr1)):.1f}°\n"
            f"{'─'*30}\n"
            f"Overall CEP50: {cep50*100:.2f} cm")
    ax4.text(0.98, 0.97, info, transform=ax4.transAxes,
             fontsize=8, verticalalignment='top', horizontalalignment='right',
             fontfamily='monospace',
             bbox=dict(boxstyle='round', facecolor='white', alpha=0.85))
    ax4.text(0.98, 0.03, f'  {status}', transform=ax4.transAxes,
             fontsize=18, fontweight='bold', color=colour,
             horizontalalignment='right', fontfamily='monospace')

    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved lap-1 orientation comparison plot to: {out_path}")


# ─── Main ───────────────────────────────────────────────────────────────────

def run_test(args):
    l = args.side_length
    r = l / 4.0
    v_s = args.straight_speed
    v_t = args.turn_speed
    num_laps = args.laps

    print()
    print("=" * 60)
    print(f"  Dynamic Accuracy + Orientation \u2014 {l}m x {l}m Square Path")
    print("=" * 60)
    print(f"  Turn radius:    {r:.2f} m")
    print(f"  Straight speed: {v_s} km/h ({v_s / 3.6:.3f} m/s)")
    print(f"  Turn speed:     {v_t} km/h ({v_t / 3.6:.3f} m/s)")
    print(f"  Laps:           {num_laps}")

    gt_times, gt_positions, gt_headings, one_lap_time = build_ground_truth(
        l, v_s, v_t, num_laps)

    total_straight = 4 * (l - 2 * r)
    total_arc = 2 * np.pi * r
    lap_dist = total_straight + total_arc

    print(f"  Lap distance:   {lap_dist:.2f} m")
    print(f"  Lap time:       {one_lap_time:.2f} s")
    print(f"  Total time:     {one_lap_time * num_laps:.2f} s")
    print()

    if args.demo:
        print("  [DEMO MODE] Generating synthetic data ...")
        algo_times, algo_pos_3d, algo_yaw = generate_demo_data(
            gt_times, gt_positions, gt_headings)
    else:
        print(f"  Loading data from: {args.data}")
        algo_times, algo_pos_3d, algo_yaw = load_csv_data(args.data)

    algo_xy = algo_pos_3d[:, :2]

    mask = algo_times <= gt_times[-1]
    algo_times = algo_times[mask]
    algo_xy = algo_xy[mask]
    algo_yaw = algo_yaw[mask]

    gt_at_algo, gt_hdg_at_algo = interpolate_ground_truth(
        algo_times, gt_times, gt_positions, gt_headings)

    # Position errors
    errors_2d = algo_xy - gt_at_algo
    cep50, radial_errors = compute_cep50(errors_2d)

    # Heading errors (wrapped)
    heading_errors = wrap_angle_deg(algo_yaw - gt_hdg_at_algo)
    hdg_stats = compute_heading_error_stats(heading_errors)

    CEP_THRESHOLD = 0.05
    passed = cep50 <= CEP_THRESHOLD
    tag = "PASS" if passed else "FAIL"

    print("-" * 60)
    print(f"  POSITION:")
    print(f"    CEP50:        {cep50 * 100:.2f} cm  ({cep50:.4f} m)")
    print(f"    Mean error:   {np.mean(radial_errors) * 100:.2f} cm")
    print(f"    Max error:    {np.max(radial_errors) * 100:.2f} cm")
    print(f"  HEADING:")
    print(f"    Median |err|: {hdg_stats['median']:.2f}\u00b0")
    print(f"    Mean |err|:   {hdg_stats['mean']:.2f}\u00b0")
    print(f"    Max |err|:    {hdg_stats['max']:.2f}\u00b0")
    print(f"  Samples:        {len(algo_times)}")
    print(f"  Result:         {tag}  (pos threshold: {CEP_THRESHOLD * 100:.0f} cm)")
    print("-" * 60)
    print()

    # Save first-lap static comparison plot
    save_first_lap_plot(l, one_lap_time,
                        algo_times, algo_xy, algo_yaw,
                        gt_at_algo, gt_hdg_at_algo,
                        radial_errors, heading_errors,
                        cep50, hdg_stats, passed, args)

    create_visualisation(l, gt_times, gt_positions, gt_headings,
                         algo_times, algo_xy, algo_yaw,
                         gt_at_algo, gt_hdg_at_algo,
                         radial_errors, heading_errors,
                         cep50, hdg_stats, passed, args)


def run_ground_truth_only(args):
    """Show ground-truth path with heading arrows."""
    l = args.side_length
    r = l / 4.0
    v_s = args.straight_speed
    v_t = args.turn_speed
    num_laps = args.laps

    print()
    print("=" * 60)
    print(f"  Ground Truth + Heading \u2014 {l}m x {l}m Square")
    print("=" * 60)
    print(f"  Turn radius:    {r:.2f} m")
    print(f"  Straight speed: {v_s} km/h ({v_s / 3.6:.3f} m/s)")
    print(f"  Turn speed:     {v_t} km/h ({v_t / 3.6:.3f} m/s)")
    print(f"  Laps:           {num_laps}")

    gt_times, gt_positions, gt_headings, one_lap_time = build_ground_truth(
        l, v_s, v_t, num_laps)

    total_straight = 4 * (l - 2 * r)
    total_arc = 2 * np.pi * r
    lap_dist = total_straight + total_arc

    print(f"  Lap distance:   {lap_dist:.2f} m")
    print(f"  Lap time:       {one_lap_time:.2f} s")
    print(f"  Total time:     {one_lap_time * num_laps:.2f} s")
    print()

    create_gt_only_visualisation(l, gt_times, gt_positions, gt_headings,
                                 one_lap_time, args)


def main():
    parser = argparse.ArgumentParser(
        description='Dynamic Accuracy + Orientation Test \u2014 '
                    'Square-Path Traversal (Section 4.1)')

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
                        help='Show only ground-truth path + heading arrows')
    parser.add_argument('--save', type=str, default=None,
                        help='Save final plot to file (e.g. results.png)')

    args = parser.parse_args()

    if not args.demo and not args.data and not args.ground_truth_only:
        parser.error(
            'One of --data <csv_file>, --demo, or --ground-truth-only is required')

    if args.ground_truth_only:
        run_ground_truth_only(args)
    else:
        run_test(args)


if __name__ == '__main__':
    main()
