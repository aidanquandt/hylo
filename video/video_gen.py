"""
Hylo UWB + IMU Sensor Fusion — Manim CE animation.

Scenes:
  TitleScene       — Project title card
  UWBScene         — UWB-only: sparse, noisy position pings at ~2 Hz
  IMUScene         — IMU-only: smooth dead-reckoning that drifts from truth
  FusionScene      — EKF fusion: UWB corrections snap drifting IMU back on track
  SummaryScene     — Side-by-side comparison table
  AllScenes        — All scenes in sequence

Render examples:
  manim video_gen.py TitleScene   -pqh
  manim video_gen.py UWBScene     -pqh
  manim video_gen.py IMUScene     -pqh
  manim video_gen.py FusionScene  -pqh
  manim video_gen.py SummaryScene -pqh
  manim video_gen.py AllScenes    -pqh --fps 60

Install: pip install manim
"""

from manim import *
import math
import random

# ---------------------------------------------------------------------------
# Project color palette (matches host/webapp/backend/static/index.html CSS vars)
# ---------------------------------------------------------------------------
BG          = "#1a1b26"   # --bg
SURFACE     = "#24283b"   # --surface
TEXT_COLOR  = "#c0caf5"   # --text
ACCENT      = "#7aa2f7"   # --accent  (blue)
SUCCESS     = "#9ece6a"   # --success (green)
ERROR       = "#f7768e"   # --error   (red)
MUTED       = "#565f89"   # --muted

IMU_COLOR   = "#e0af68"   # warm amber  -> IMU path
UWB_COLOR   = ACCENT      # blue        -> UWB pings
TRUTH_COLOR = SUCCESS     # green       -> ground truth
FUSED_COLOR = "#bb9af7"   # purple      -> EKF output
TAG_COLOR   = "#ff9e64"   # orange      -> tag dot

# ---------------------------------------------------------------------------
# Tweakable parameters
# ---------------------------------------------------------------------------
UWB_FREQ_HZ    = 2.0    # UWB pings per second (lower = more dramatic gaps)
UWB_NOISE_M    = 0.18   # UWB position noise standard deviation (metres)
IMU_DRIFT_RATE = 0.04   # Metres of position bias added per second
IMU_GYRO_DRIFT = 0.012  # rad/s heading drift (causes path to arc)
EKF_CORRECTION = 0.55   # Kalman gain proxy [0-1]: how aggressively UWB corrects
ROOM_W_M       = 6.0    # Room width  (metres)
ROOM_H_M       = 4.5    # Room height (metres)
PATH_DURATION_S = 10.0  # Seconds of tag motion per scene

# ---------------------------------------------------------------------------
# Coordinate helpers: map room metres -> Manim scene units
# ---------------------------------------------------------------------------
SCALE = 1.15   # Manim units per metre

def m2u(x_m, y_m):
    """Convert room-frame metres to Manim scene coords (centred at origin)."""
    return np.array([
        (x_m - ROOM_W_M / 2) * SCALE,
        (y_m - ROOM_H_M / 2) * SCALE,
        0.0,
    ])

# ---------------------------------------------------------------------------
# Ground-truth path: smooth Lissajous curve inside the room
# ---------------------------------------------------------------------------
def true_position(t):
    cx, cy = ROOM_W_M / 2, ROOM_H_M / 2
    rx, ry = ROOM_W_M * 0.36, ROOM_H_M * 0.34
    phase  = t / PATH_DURATION_S * 2 * math.pi
    return (
        cx + rx * math.sin(phase * 1.3 + 0.4),
        cy + ry * math.sin(phase * 0.9),
    )

def true_path_points(n=300):
    times = [i / n * PATH_DURATION_S for i in range(n + 1)]
    return [m2u(*true_position(t)) for t in times]

# ---------------------------------------------------------------------------
# Noise / drift generators
# ---------------------------------------------------------------------------
def uwb_noisy_position(t, rng):
    tx, ty = true_position(t)
    return (tx + rng.gauss(0, UWB_NOISE_M), ty + rng.gauss(0, UWB_NOISE_M * 0.8))

def imu_dead_reckoning(t):
    """Integrate a drifting velocity -- heading error accumulates over time."""
    cx, cy = ROOM_W_M / 2, ROOM_H_M / 2
    rx, ry = ROOM_W_M * 0.36, ROOM_H_M * 0.34
    phase  = t / PATH_DURATION_S * 2 * math.pi
    x_true = cx + rx * math.sin(phase * 1.3 + 0.4)
    y_true = cy + ry * math.sin(phase * 0.9)
    heading_err = IMU_GYRO_DRIFT * t
    return (
        x_true + IMU_DRIFT_RATE * t * math.cos(heading_err + 0.8),
        y_true + IMU_DRIFT_RATE * t * math.sin(heading_err + 0.8),
    )

# ---------------------------------------------------------------------------
# Shared room-building helper
# ---------------------------------------------------------------------------
def make_room():
    room = Rectangle(
        width=ROOM_W_M * SCALE, height=ROOM_H_M * SCALE,
        stroke_color=MUTED, stroke_width=2,
        fill_color=SURFACE, fill_opacity=0.35,
    )
    anchors = VGroup()
    for (ax, ay), lbl in zip(
        [(0.3, 0.3), (ROOM_W_M-0.3, 0.3), (0.3, ROOM_H_M-0.3), (ROOM_W_M-0.3, ROOM_H_M-0.3)],
        ["B0", "B1", "B2", "B3"],
    ):
        pos = m2u(ax, ay)
        dot = Square(side_length=0.18, color=ACCENT, fill_color=ACCENT, fill_opacity=0.9).move_to(pos)
        label = Text(lbl, font_size=14, color=ACCENT).next_to(dot, UP, buff=0.07)
        anchors.add(dot, label)
    return room, anchors

# ---------------------------------------------------------------------------
# Scene 0 - Title card
# ---------------------------------------------------------------------------
class TitleScene(Scene):
    def construct(self):
        self.camera.background_color = BG
        title    = Text("Hylo Indoor Positioning", font_size=52, color=ACCENT, weight=BOLD)
        subtitle = Text("UWB  +  IMU  Sensor Fusion", font_size=32, color=TEXT_COLOR)
        caption  = Text(
            "Extended Kalman Filter  |  Two-Way Ranging  |  9-State Estimator",
            font_size=18, color=MUTED,
        )
        stack = VGroup(title, subtitle, caption).arrange(DOWN, buff=0.5).move_to(ORIGIN)
        line  = Line(LEFT*4, RIGHT*4, stroke_color=ACCENT, stroke_width=1.5, stroke_opacity=0.5
                     ).next_to(subtitle, DOWN, buff=0.25)
        self.play(FadeIn(title, shift=UP*0.3), run_time=1.0)
        self.play(FadeIn(subtitle, shift=UP*0.2), Create(line), run_time=0.8)
        self.play(FadeIn(caption), run_time=0.6)
        self.wait(2.5)
        self.play(FadeOut(stack), FadeOut(line), run_time=0.6)


# ---------------------------------------------------------------------------
# Scene 1 - UWB only
# ---------------------------------------------------------------------------
class UWBScene(Scene):
    def construct(self):
        self.camera.background_color = BG
        rng = random.Random(42)

        scene_label = Text("UWB Only", font_size=42, color=UWB_COLOR, weight=BOLD)
        desc = Text(
            f"Sparse pings at {UWB_FREQ_HZ:.0f} Hz  |  Noisy  |  Gaps between measurements",
            font_size=22, color=TEXT_COLOR,
        )
        VGroup(scene_label, desc).arrange(DOWN, buff=0.4).move_to(ORIGIN)
        self.play(FadeIn(scene_label, shift=UP*0.2), run_time=0.7)
        self.play(FadeIn(desc), run_time=0.5)
        self.wait(1.2)
        self.play(FadeOut(scene_label), FadeOut(desc), run_time=0.5)

        room, anchors = make_room()
        room_label = Text("6 m x 4.5 m room", font_size=16, color=MUTED).to_corner(DR, buff=0.3)
        self.play(Create(room), FadeIn(anchors), FadeIn(room_label), run_time=0.8)

        # Faint truth path for reference
        pts = true_path_points(250)
        truth_path = VMobject(stroke_color=TRUTH_COLOR, stroke_width=1.5, stroke_opacity=0.25)
        truth_path.set_points_as_corners(pts)
        truth_label = Text("Ground truth (hidden from UWB system)", font_size=14,
                           color=TRUTH_COLOR).to_corner(UL, buff=0.3)
        self.play(Create(truth_path), FadeIn(truth_label), run_time=1.0)

        anno = Text(
            "UWB TWR: tag pings each anchor in turn\n"
            "Trilateration gives a noisy position estimate",
            font_size=16, color=MUTED, line_spacing=1.3,
        ).to_corner(UR, buff=0.35)
        self.play(FadeIn(anno), run_time=0.4)

        ping_interval = 1.0 / UWB_FREQ_HZ
        n_steps       = int(PATH_DURATION_S / ping_interval)
        step_time     = 0.20

        fx, fy  = uwb_noisy_position(0.0, rng)
        est_dot = Dot(m2u(fx, fy), color=UWB_COLOR, radius=0.14)
        est_lbl = Text("UWB estimate", font_size=14, color=UWB_COLOR).next_to(est_dot, DOWN, buff=0.1)
        tag_dot = Dot(m2u(*true_position(0)), color=TRUTH_COLOR, radius=0.09, fill_opacity=0.35)
        self.play(FadeIn(est_dot), FadeIn(est_lbl), run_time=0.3)
        self.add(tag_dot)

        ping_trail = []
        for step in range(1, n_steps + 1):
            t       = step * ping_interval
            tx, ty  = true_position(t)
            nx, ny  = uwb_noisy_position(t, rng)
            new_pos = m2u(nx, ny)

            ring  = Circle(radius=0.08, color=UWB_COLOR, stroke_width=2, fill_opacity=0).move_to(new_pos)
            ghost = Dot(new_pos, color=UWB_COLOR, radius=0.06, fill_opacity=0.6)
            ping_trail.append(ghost)
            if len(ping_trail) > 8:
                self.play(FadeOut(ping_trail.pop(0)), run_time=0.04)

            self.play(
                tag_dot.animate.move_to(m2u(tx, ty)),
                est_dot.animate.move_to(new_pos),
                est_lbl.animate.next_to(new_pos, DOWN, buff=0.1),
                FadeIn(ring, scale=0.3),
                FadeIn(ghost),
                run_time=step_time,
            )
            self.play(FadeOut(ring, scale=2.5, rate_func=rush_from), run_time=0.10)

        conclude = Text(
            "Problem: noisy jumps, no information between pings",
            font_size=19, color=ERROR,
        ).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(conclude, shift=UP*0.1), run_time=0.6)
        self.wait(1.5)
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.7)


# ---------------------------------------------------------------------------
# Scene 2 - IMU only
# ---------------------------------------------------------------------------
class IMUScene(Scene):
    def construct(self):
        self.camera.background_color = BG
        n_pts = 280

        scene_label = Text("IMU Only", font_size=42, color=IMU_COLOR, weight=BOLD)
        desc = Text(
            "Dead-reckoning at high rate  |  Smooth  |  Accumulating drift",
            font_size=22, color=TEXT_COLOR,
        )
        VGroup(scene_label, desc).arrange(DOWN, buff=0.4).move_to(ORIGIN)
        self.play(FadeIn(scene_label, shift=UP*0.2), run_time=0.7)
        self.play(FadeIn(desc), run_time=0.5)
        self.wait(1.2)
        self.play(FadeOut(scene_label), FadeOut(desc), run_time=0.5)

        room, anchors = make_room()
        self.play(Create(room), FadeIn(anchors), run_time=0.7)

        truth_pts = true_path_points(n_pts)
        truth_vmo = VMobject(stroke_color=TRUTH_COLOR, stroke_width=2.0, stroke_opacity=0.5)
        truth_vmo.set_points_as_corners(truth_pts)
        truth_lbl = Text("True path", font_size=15, color=TRUTH_COLOR).to_corner(UL, buff=0.3)
        self.play(Create(truth_vmo), FadeIn(truth_lbl), run_time=1.0)

        times   = [i / n_pts * PATH_DURATION_S for i in range(n_pts + 1)]
        imu_pts = [m2u(*imu_dead_reckoning(t)) for t in times]

        imu_vmo = VMobject(stroke_color=IMU_COLOR, stroke_width=2.2)
        imu_vmo.set_points_as_corners(imu_pts)
        imu_vmo.set_stroke(opacity=0)

        imu_lbl = Text("IMU dead-reckoning", font_size=15, color=IMU_COLOR).to_corner(UR, buff=0.35)

        drift_t     = PATH_DURATION_S * 0.6
        true_mid    = m2u(*true_position(drift_t))
        imu_mid     = m2u(*imu_dead_reckoning(drift_t))
        drift_arrow = Arrow(true_mid, imu_mid, color=ERROR, stroke_width=2, buff=0.08)
        drift_label = Text("Drift", font_size=16, color=ERROR
                           ).next_to((true_mid + imu_mid) / 2, RIGHT, buff=0.15)

        anno = Text(
            "Accelerometer bias + gyro drift\n-> heading error accumulates over time",
            font_size=16, color=MUTED, line_spacing=1.3,
        ).to_corner(DR, buff=0.4)

        tag_dot = Dot(imu_pts[0], color=TAG_COLOR, radius=0.12)
        self.add(tag_dot)
        self.play(FadeIn(imu_lbl), FadeIn(anno), run_time=0.4)

        self.play(
            imu_vmo.animate.set_stroke(opacity=0.9),
            UpdateFromAlphaFunc(
                tag_dot,
                lambda mob, a: mob.move_to(imu_pts[min(int(a * n_pts * 0.5), n_pts)]),
            ),
            run_time=PATH_DURATION_S * 0.5, rate_func=linear,
        )
        self.play(FadeIn(drift_arrow), FadeIn(drift_label), run_time=0.5)
        self.play(
            UpdateFromAlphaFunc(
                tag_dot,
                lambda mob, a: mob.move_to(imu_pts[min(int((0.5 + a*0.5) * n_pts), n_pts)]),
            ),
            run_time=PATH_DURATION_S * 0.5, rate_func=linear,
        )

        conclude = Text(
            "Problem: smooth but wrong - drift grows unbounded",
            font_size=19, color=ERROR,
        ).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(conclude, shift=UP*0.1), run_time=0.6)
        self.wait(1.5)
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.7)


# ---------------------------------------------------------------------------
# Scene 3 - EKF Sensor Fusion
# ---------------------------------------------------------------------------
class FusionScene(Scene):
    def construct(self):
        self.camera.background_color = BG
        rng   = random.Random(99)
        n_pts = 300

        fusion_uwb_freq  = 4.0    # Hz
        fusion_ekf_corr  = 0.35   # Kalman gain — gentle UWB correction
        fusion_drift_deg = 0.006  # rad/s heading error accumulated in EKF predict

        scene_label = Text("EKF Sensor Fusion", font_size=42, color=FUSED_COLOR, weight=BOLD)
        desc = Text(
            "IMU predicts  |  UWB corrects  |  EKF optimal blend",
            font_size=22, color=TEXT_COLOR,
        )
        VGroup(scene_label, desc).arrange(DOWN, buff=0.4).move_to(ORIGIN)
        self.play(FadeIn(scene_label, shift=UP*0.2), run_time=0.7)
        self.play(FadeIn(desc), run_time=0.5)
        self.wait(1.2)
        self.play(FadeOut(scene_label), FadeOut(desc), run_time=0.5)

        room, anchors = make_room()
        self.play(Create(room), FadeIn(anchors), run_time=0.7)

        truth_pts = true_path_points(n_pts)
        truth_vmo = VMobject(stroke_color=TRUTH_COLOR, stroke_width=1.5, stroke_opacity=0.30)
        truth_vmo.set_points_as_corners(truth_pts)
        truth_lbl = Text("True path", font_size=14, color=TRUTH_COLOR).to_corner(UL, buff=0.3)
        self.play(Create(truth_vmo), FadeIn(truth_lbl), run_time=0.8)

        # ---- Pre-compute paths ----
        dt    = PATH_DURATION_S / n_pts
        times = [i * dt for i in range(n_pts + 1)]

        # EKF path: velocity integration with small heading error + UWB corrections
        ex, ey     = true_position(0.0)
        ekf_positions = [(ex, ey)]
        uwb_events    = []
        next_uwb_t    = 1.0 / fusion_uwb_freq

        for i in range(1, n_pts + 1):
            t_prev = times[i - 1]
            t_cur  = times[i]
            # True velocity from path derivative
            tx0, ty0 = true_position(t_prev)
            tx1, ty1 = true_position(t_cur)
            true_vx  = (tx1 - tx0) / dt
            true_vy  = (ty1 - ty0) / dt
            # IMU: rotate velocity by small accumulating heading error
            angle_err = fusion_drift_deg * t_cur
            imu_vx = true_vx * math.cos(angle_err) - true_vy * math.sin(angle_err)
            imu_vy = true_vx * math.sin(angle_err) + true_vy * math.cos(angle_err)
            # Predict: integrate IMU velocity
            ex += imu_vx * dt
            ey += imu_vy * dt
            # Update at UWB pings
            if t_cur >= next_uwb_t:
                mx, my = uwb_noisy_position(t_cur, rng)
                ex += fusion_ekf_corr * (mx - ex)
                ey += fusion_ekf_corr * (my - ey)
                uwb_events.append((i, mx, my))
                next_uwb_t += 1.0 / fusion_uwb_freq
            ekf_positions.append((ex, ey))

        ekf_pts = [m2u(x, y) for x, y in ekf_positions]

        # IMU display path: same velocity integration but larger drift (for visual contrast)
        ix, iy    = true_position(0.0)
        imu_disp  = [(ix, iy)]
        for i in range(1, n_pts + 1):
            t_prev = times[i - 1]
            t_cur  = times[i]
            tx0, ty0 = true_position(t_prev)
            tx1, ty1 = true_position(t_cur)
            true_vx  = (tx1 - tx0) / dt
            true_vy  = (ty1 - ty0) / dt
            angle_err = IMU_GYRO_DRIFT * t_cur
            imu_vx = true_vx * math.cos(angle_err) - true_vy * math.sin(angle_err)
            imu_vy = true_vx * math.sin(angle_err) + true_vy * math.cos(angle_err)
            ix += imu_vx * dt
            iy += imu_vy * dt
            imu_disp.append((ix, iy))

        imu_pts = [m2u(x, y) for x, y in imu_disp]

        # Legend
        ekf_lbl = Text("EKF estimate",   font_size=15, color=FUSED_COLOR).to_corner(UR, buff=0.35)
        imu_lbl = Text("IMU prediction", font_size=14, color=IMU_COLOR).next_to(ekf_lbl, DOWN, buff=0.12)
        uwb_lbl = Text("UWB correction", font_size=14, color=UWB_COLOR).next_to(imu_lbl, DOWN, buff=0.12)
        anno = Text(
            "Predict:  IMU accel + gyro  ->  state propagation\n"
            "Update:   UWB distance  ->  Kalman correction\n"
            "Result:   smooth  +  bounded error",
            font_size=15, color=MUTED, line_spacing=1.4,
        ).to_corner(DR, buff=0.35)
        self.play(FadeIn(ekf_lbl), FadeIn(imu_lbl), FadeIn(uwb_lbl), FadeIn(anno), run_time=0.5)

        tag_dot        = Dot(ekf_pts[0], color=TAG_COLOR, radius=0.13)
        display_speed  = 0.55
        anim_s_per_stp = dt * display_speed
        self.add(tag_dot)

        def make_updater(pts):
            def updater(mob, a):
                mob.move_to(pts[min(int(a * (len(pts) - 1)), len(pts) - 1)])
            return updater

        prev_idx = 0

        for ev_idx, mx, my in uwb_events:
            seg_imu_pts = imu_pts[prev_idx : ev_idx + 1]
            seg_ekf_pts = ekf_pts[prev_idx : ev_idx + 1]
            if len(seg_ekf_pts) < 2:
                prev_idx = ev_idx
                continue

            seg_dur = max(len(seg_ekf_pts) * anim_s_per_stp, 0.12)

            seg_imu = VMobject(stroke_color=IMU_COLOR, stroke_width=1.8, stroke_opacity=0.45)
            seg_imu.set_points_as_corners(seg_imu_pts)
            seg_ekf = VMobject(stroke_color=FUSED_COLOR, stroke_width=2.5)
            seg_ekf.set_points_as_corners(seg_ekf_pts)

            self.play(
                Create(seg_imu),
                Create(seg_ekf),
                UpdateFromAlphaFunc(tag_dot, make_updater(seg_ekf_pts)),
                run_time=seg_dur, rate_func=linear,
            )

            # Subtle UWB ping flash
            uwb_pos  = m2u(mx, my)
            ring     = Circle(radius=0.06, stroke_color=UWB_COLOR, stroke_width=1.5,
                              fill_opacity=0).move_to(uwb_pos)
            ping_dot = Dot(uwb_pos, color=UWB_COLOR, radius=0.07, fill_opacity=0.75)
            self.play(FadeIn(ping_dot, scale=0.3), FadeIn(ring, scale=0.3), run_time=0.07)
            self.play(FadeOut(ring, scale=2.2), FadeOut(ping_dot), run_time=0.09)

            prev_idx = ev_idx

        # Final tail segment
        if prev_idx < n_pts:
            tail_imu = imu_pts[prev_idx:]
            tail_ekf = ekf_pts[prev_idx:]
            if len(tail_ekf) >= 2:
                t_imu = VMobject(stroke_color=IMU_COLOR, stroke_width=1.8, stroke_opacity=0.45)
                t_imu.set_points_as_corners(tail_imu)
                t_ekf = VMobject(stroke_color=FUSED_COLOR, stroke_width=2.5)
                t_ekf.set_points_as_corners(tail_ekf)
                self.play(Create(t_imu), Create(t_ekf),
                          UpdateFromAlphaFunc(tag_dot, make_updater(tail_ekf)),
                          run_time=0.5, rate_func=linear)

        conclude = Text(
            "EKF: IMU provides continuity  +  UWB bounds the error",
            font_size=19, color=FUSED_COLOR,
        ).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(conclude, shift=UP*0.1), run_time=0.6)
        self.wait(2.0)
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.7)


# ---------------------------------------------------------------------------
# Scene 4 - Side-by-side summary
# ---------------------------------------------------------------------------
class SummaryScene(Scene):
    def construct(self):
        self.camera.background_color = BG

        title = Text("Why Sensor Fusion?", font_size=38, color=ACCENT, weight=BOLD)
        title.to_edge(UP, buff=0.5)
        self.play(FadeIn(title), run_time=0.6)

        cols = [
            ("UWB alone",  UWB_COLOR,   ["+ Accurate when available",  "- Sparse: ~4 Hz per anchor", "- Noisy (multipath / NLOS)"]),
            ("IMU alone",  IMU_COLOR,   ["+ High rate (100-1000 Hz)",   "+ Smooth trajectory",        "- Unbounded drift"]),
            ("EKF Fusion", FUSED_COLOR, ["+ Smooth  (IMU rate)",        "+ Bounded error (UWB)",      "+ Robust to dropouts"]),
        ]

        col_groups = VGroup()
        for heading, color, points in cols:
            hdr = Text(heading, font_size=24, color=color, weight=BOLD)
            items = VGroup(*[
                Text(p, font_size=17, color=TEXT_COLOR if p.startswith("+") else ERROR)
                for p in points
            ]).arrange(DOWN, aligned_edge=LEFT, buff=0.22)
            col_groups.add(VGroup(hdr, items).arrange(DOWN, aligned_edge=LEFT, buff=0.30))

        col_groups.arrange(RIGHT, buff=0.9).next_to(title, DOWN, buff=0.7)

        for i in range(1, len(cols)):
            x = col_groups[i].get_left()[0] - 0.45
            self.add(Line([x, col_groups.get_top()[1] - 0.1, 0],
                          [x, col_groups.get_bottom()[1] + 0.1, 0],
                          stroke_color=MUTED, stroke_width=1))

        self.play(LaggedStartMap(FadeIn, col_groups, shift=UP*0.15, lag_ratio=0.25), run_time=1.2)

        box  = SurroundingRectangle(col_groups[2], color=FUSED_COLOR, stroke_width=2, buff=0.18)
        best = Text("Best of both worlds", font_size=16, color=FUSED_COLOR).next_to(box, DOWN, buff=0.2)
        self.play(Create(box), FadeIn(best), run_time=0.7)

        ekf_note = Text(
            "9-state EKF  |  position + velocity + attitude\n"
            "State: [x, y, z,  vx, vy, vz,  droll, dpitch, dyaw]",
            font_size=15, color=MUTED, line_spacing=1.3,
        ).to_edge(DOWN, buff=0.4)
        self.play(FadeIn(ekf_note), run_time=0.5)
        self.wait(2.5)
        self.play(*[FadeOut(m) for m in self.mobjects], run_time=0.7)


# ---------------------------------------------------------------------------
# AllScenes - renders everything in sequence
# ---------------------------------------------------------------------------
class AllScenes(Scene):
    """manim video_gen.py AllScenes -pqh --fps 60"""
    def construct(self):
        for SceneClass in [TitleScene, UWBScene, IMUScene, FusionScene, SummaryScene]:
            sub = SceneClass()
            sub.renderer = self.renderer
            sub.camera   = self.camera
            sub.construct()
