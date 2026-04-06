"""
Hylo UWB + IMU Sensor Fusion — Poster Still Image (v3)
=======================================================
White-background version: no floor/room rectangles, bold colors
that read on white, no distance labels, more cubic camera angle.

Render:
  cd video
  manim poster.py PosterScene -s -qh
  manim poster.py PosterScene -s --resolution 3840,2160 -qh   # 4K
"""

from manim import *
import math
import numpy as np

# Use a taller output frame ratio (4:3) instead of widescreen (16:9).
config.pixel_width = 1920
config.pixel_height = 1440

# ─── Color palette for white poster background ───────────────────────────────
BG           = WHITE
TEXT_COLOR   = "#1a1b26"      # near-black text
ACCENT       = "#2563eb"      # strong blue  – UWB lines
ANCHOR_COLOR = "#2563eb"      # anchor nodes
WAVE_COLOR   = "#3b82f6"      # UWB wave rings
TAG_COLOR    = "#ea580c"      # robot orange
SUCCESS      = "#16a34a"      # dark green – velocity
IMU_X_COL   = "#dc2626"      # red   – IMU X
IMU_Y_COL   = "#16a34a"      # green – IMU Y
IMU_Z_COL   = "#2563eb"      # blue  – IMU Z
TRACK_COL   = "#1e293b"      # near-black track lines
POLE_COL    = "#64748b"      # slate-grey poles
PCB_COLOR   = "#166534"      # dark green PCB
CHASSIS_COLOR = "#1e293b"    # near-black chassis
WHEEL_COL   = "#b45309"      # amber wheels
MUTED       = "#94a3b8"      # light grey for call-out lines

# ─── Scene geometry ──────────────────────────────────────────────────────────
ROOM_W_M  = 6.0
ROOM_H_M  = 5.5
SCALE     = 1.10
POLE_H_M  = 1.2

ANCHOR_XY_M = [(0.3, 0.3), (5.7, 0.3), (0.3, 5.2), (5.7, 5.2)]
ANCHOR_LBLS = ["A0", "A1", "A2", "A3"]

ROBOT_X_M     = 3.1
ROBOT_Y_M     = 2.6
ROBOT_YAW_DEG = 35.0
ROBOT_SPEED   = 0.42

# ─── Coordinate helpers ──────────────────────────────────────────────────────
def m2u(x_m, y_m, z_m=0.0):
    return np.array([
        (x_m - ROOM_W_M / 2) * SCALE,
        (y_m - ROOM_H_M / 2) * SCALE,
        z_m * SCALE,
    ])

def dist_m(ax, ay, bx, by):
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)


class PosterScene(ThreeDScene):
    def construct(self):
        self.camera.background_color = BG
        # More cubic: phi=45° gives true isometric feel
        # More top-down, rotated so long side runs left-right in frame
        self.set_camera_orientation(phi=42 * DEGREES, theta=-40 * DEGREES)

        robot_3d = m2u(ROBOT_X_M, ROBOT_Y_M, 0.0)
        yaw_rad  = math.radians(ROBOT_YAW_DEG)
        fwd      = np.array([math.cos(yaw_rad), math.sin(yaw_rad), 0.0])
        right    = np.array([math.sin(yaw_rad), -math.cos(yaw_rad), 0.0])
        perp     = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0])

        # ── 1. Track outline only — no floor rectangle ────────────────────────
        t_outer_w = (ROOM_W_M - 0.7)  * SCALE
        t_outer_h = (ROOM_H_M - 0.6)  * SCALE
        t_inner_w = (ROOM_W_M - 2.6)  * SCALE
        t_inner_h = (ROOM_H_M - 2.1)  * SCALE
        t_mid_w   = (ROOM_W_M - 1.65) * SCALE
        t_mid_h   = (ROOM_H_M - 1.35) * SCALE

        track_outer = RoundedRectangle(
            corner_radius=0.52, width=t_outer_w, height=t_outer_h,
            stroke_color=TRACK_COL, stroke_width=3.5, stroke_opacity=0.30,
            fill_opacity=0,
        )
        track_inner = RoundedRectangle(
            corner_radius=0.25, width=t_inner_w, height=t_inner_h,
            stroke_color=TRACK_COL, stroke_width=2.5, stroke_opacity=0.25,
            fill_opacity=0,
        )
        # Bold centre driving line
        track_centre = RoundedRectangle(
            corner_radius=0.38, width=t_mid_w, height=t_mid_h,
            stroke_color=TRACK_COL, stroke_width=6, stroke_opacity=0.75,
            fill_opacity=0,
        )
        self.add(track_outer, track_inner, track_centre)

        # ── 2. UWB Anchors — 3D enclosures on stands with radio waves ─────────
        for (ax, ay), _lbl in zip(ANCHOR_XY_M, ANCHOR_LBLS):
            base = m2u(ax, ay, 0.0)
            top  = m2u(ax, ay, POLE_H_M)

            # Wider base plate
            base_plate = Rectangle(
                width=0.28, height=0.28,
                fill_color=POLE_COL, fill_opacity=0.7,
                stroke_color="#475569", stroke_width=1.2,
            ).move_to(base)

            # Thicker pole / stand
            pole = Line(base, top, stroke_color=POLE_COL, stroke_width=5)

            # 3D anchor enclosure box (Prism) at the top
            anchor_box = Prism(
                dimensions=[0.32, 0.24, 0.14],
            ).set_fill(ANCHOR_COLOR, opacity=0.95).set_stroke(
                color="#60a5fa", width=1.5, opacity=0.9,
            )
            anchor_box.move_to(top + OUT * 0.07)

            # Status LED on anchor face
            anchor_led = Dot(
                top + OUT * 0.14 + np.array([0.06, 0.0, 0.0]),
                color="#22c55e", radius=0.025,
            )

            # 5 concentric wave arcs (270° arcs for dynamic radio feel)
            wave_specs = [
                (0.40, 3.5, 0.58),
                (0.68, 3.0, 0.42),
                (0.96, 2.4, 0.28),
                (1.24, 1.8, 0.16),
                (1.52, 1.2, 0.08),
            ]
            for radius, width, opacity in wave_specs:
                arc = Arc(
                    radius=radius,
                    start_angle=PI * 0.15,
                    angle=PI * 1.7,
                    stroke_color=WAVE_COLOR,
                    stroke_width=width,
                    stroke_opacity=opacity,
                ).move_to(top + OUT * 0.07)
                self.add(arc)

            self.add(base_plate, pole, anchor_box, anchor_led)

        # ── 3. UWB Ranging lines (dashed, no distance labels) ─────────────────
        for (ax, ay) in ANCHOR_XY_M:
            anchor_top = m2u(ax, ay, POLE_H_M)
            raw = Line(
                anchor_top, robot_3d + OUT * 0.10,
                stroke_color=ACCENT, stroke_width=2.5, stroke_opacity=0.65,
            )
            dashed = DashedVMobject(raw, num_dashes=18, dashed_ratio=0.5)
            self.add(dashed)

        # ── 4. Robot: chassis platform + raised PCB module on standoffs ──────
        chassis_d = 0.34   # front-back (small robot in big warehouse)
        chassis_w = 0.26   # left-right
        chassis_h = 0.09   # low-profile platform

        chassis = Prism(
            dimensions=[chassis_d, chassis_w, chassis_h],
        ).set_fill(CHASSIS_COLOR, opacity=0.97).set_stroke(
            color="#475569", width=2.0, opacity=0.95,
        )
        chassis.move_to(robot_3d + OUT * chassis_h / 2)
        chassis.rotate(yaw_rad, axis=OUT)
        self.add(chassis)

        # Wheels — mecanum style with tread ring + hub
        wheel_r = 0.052
        for wx, wy in [
            ( 0.13,  0.16),
            (-0.13,  0.16),
        ]:
            wpos = robot_3d + fwd * wx + right * wy + OUT * wheel_r
            # Outer tyre ring
            tyre = Annulus(
                inner_radius=wheel_r * 0.55,
                outer_radius=wheel_r,
                fill_color=WHEEL_COL, fill_opacity=0.95,
                stroke_color="#78350f", stroke_width=2.0,
            ).move_to(wpos)
            # Hub centre
            hub = Circle(
                radius=wheel_r * 0.30,
                fill_color="#1c1917", fill_opacity=1,
                stroke_color="#57534e", stroke_width=1.5,
            ).move_to(wpos)
            # Inner ring (rim)
            rim = Circle(
                radius=wheel_r * 0.55,
                fill_color="#292524", fill_opacity=0.9,
                stroke_color="#57534e", stroke_width=1,
            ).move_to(wpos)
            # Spokes
            spokes = VGroup()
            for angle in [0, 45, 90, 135]:
                a = math.radians(angle)
                sd = np.array([math.cos(a), math.sin(a), 0])
                spokes.add(Line(
                    wpos, wpos + sd * wheel_r * 0.52,
                    stroke_color="#44403c", stroke_width=1.5,
                ))
            self.add(tyre, rim, spokes, hub)

        # ── Standoffs (4 posts raising PCB above chassis) ─────────────────────
        standoff_h = 0.045  # visible gap between chassis top and PCB
        chassis_top_z = robot_3d[2] + chassis_h
        pcb_z = chassis_top_z + standoff_h

        for sx, sy in [(0.065, 0.045), (-0.065, 0.045), (0.065, -0.045), (-0.065, -0.045)]:
            s_base = robot_3d + fwd * sx + right * sy + OUT * chassis_top_z
            s_top  = s_base + OUT * standoff_h
            standoff = Line(s_base, s_top, stroke_color="#94a3b8", stroke_width=3)
            self.add(standoff)

        # ── PCB module raised on standoffs ────────────────────────────────────
        pcb_d = 0.17   # clearly smaller than chassis
        pcb_w = 0.13
        pcb_centre = np.array([robot_3d[0], robot_3d[1], pcb_z])

        def flat_rect(w, h, color, stroke_c="#334155", sw=1.0, opacity=0.95):
            """Flat rectangle in XY plane, aligned with robot yaw."""
            r = Rectangle(width=w, height=h,
                          fill_color=color, fill_opacity=opacity,
                          stroke_color=stroke_c, stroke_width=sw)
            r.move_to(pcb_centre)
            r.rotate(90 * DEGREES, axis=RIGHT)
            r.rotate(yaw_rad, axis=OUT)
            return r

        pcb = flat_rect(pcb_d, pcb_w, PCB_COLOR, "#4ade80", 2.2)
        self.add(pcb)

        # PCB components (scaled to smaller PCB)
        soc = flat_rect(0.045, 0.045, "#0f172a", "#475569", 1.0)
        soc.shift(fwd * 0.018 + OUT * 0.006)

        chip2 = flat_rect(0.028, 0.022, "#1e293b", "#334155", 0.7)
        chip2.shift(-fwd * 0.042 + right * 0.028 + OUT * 0.005)

        chip3 = flat_rect(0.022, 0.022, "#1e293b", "#334155", 0.7)
        chip3.shift(-fwd * 0.042 - right * 0.028 + OUT * 0.005)

        uwb_patch = flat_rect(0.028, 0.028, "#1d4ed8", "#3b82f6", 1.0)
        uwb_patch.shift(fwd * 0.065 + OUT * 0.006)

        led  = Dot(pcb_centre + fwd * 0.007 + right * 0.045 + OUT * 0.01,
                   color="#ef4444", radius=0.010)
        led2 = Dot(pcb_centre + fwd * 0.007 + right * 0.032 + OUT * 0.01,
                   color="#22c55e", radius=0.009)

        connector = flat_rect(0.10, 0.02, "#334155", "#64748b", 0.7)
        connector.shift(-fwd * 0.07 + OUT * 0.005)

        self.add(soc, chip2, chip3, uwb_patch, connector, led, led2)

        # ── 5. Heading arrow ──────────────────────────────────────────────────
        h_start = robot_3d + OUT * (chassis_h + standoff_h + 0.02)
        h_end   = h_start + fwd * 0.30
        self.add(Arrow(
            h_start, h_end,
            color=TAG_COLOR, stroke_width=5, buff=0,
            max_tip_length_to_length_ratio=0.22,
        ))

        # ── 6. IMU body-frame axes (big arrows, well-separated labels) ────────
        AXIS      = 0.55
        imu_orig  = robot_3d + OUT * (chassis_h + standoff_h + 0.03)

        axis_configs = [
            # Custom text offsets keep labels readable in the projected 3D view.
            (fwd,                       IMU_X_COL, "X", AXIS,        fwd * 0.04 + right * 0.02),
            (perp,                      IMU_Y_COL, "Y", AXIS,        perp * 0.12 + right * 0.10 - fwd * 0.04),
            (np.array([0.0, 0.0, 1.0]), IMU_Z_COL, "Z", AXIS * 1.45, np.array([0.0, 0.0, 0.42]) + right * 0.06),
        ]
        for direction, color, label_char, length, label_offset in axis_configs:
            self.add(Arrow(
                imu_orig, imu_orig + direction * length,
                color=color, stroke_width=5.5, buff=0,
                max_tip_length_to_length_ratio=0.18,
            ))
            # lbl = Text(label_char, font_size=30, color=color, weight=BOLD)
            # if label_char == "Z":
            #     lbl.move_to(imu_orig + direction * (length + 0.10) - right * 0.16 + OUT * 0.02)
            # elif label_char == "Y":
            #     y_tip = imu_orig + direction * length
            #     lbl.move_to(y_tip + RIGHT * 0.24 + UP * 0.10)
            # else:
            #     lbl.move_to(imu_orig + direction * (length + 0.30) + label_offset)
            # self.add(lbl)

        # ── 7. Laptop + WiFi comms link ───────────────────────────────────────
        # Laptop sits on the floor at lower-left, outside the track.
        laptop_pos = m2u(1.00, -2.20, 0.0)

        # Floor shadow makes the laptop feel grounded in the scene.
        laptop_shadow = Ellipse(
            width=0.90, height=0.46,
            fill_color=BLACK, fill_opacity=0.14,
            stroke_opacity=0,
        )
        laptop_shadow.move_to(laptop_pos + OUT * 0.002)
        laptop_shadow.rotate(35 * DEGREES, axis=OUT)
        self.add(laptop_shadow)

        # Laptop base (lying flat) — bigger
        laptop_base = Rectangle(
            width=0.70, height=0.50,
            fill_color="#334155", fill_opacity=0.95,
            stroke_color="#64748b", stroke_width=2.0,
        )
        laptop_base.move_to(laptop_pos + OUT * 0.01)
        laptop_base.rotate(35 * DEGREES, axis=OUT)

        # Laptop screen (standing up behind the base)
        screen_base_dir = np.array([math.cos(math.radians(35 + 90)),
                                     math.sin(math.radians(35 + 90)), 0.0])
        screen_pos = laptop_pos + screen_base_dir * 0.20 + OUT * 0.14
        laptop_screen = Rectangle(
            width=0.65, height=0.42,
            fill_color="#1e293b", fill_opacity=0.95,
            stroke_color="#64748b", stroke_width=1.8,
        )
        laptop_screen.move_to(screen_pos)
        screen_right = np.array([math.cos(math.radians(35)),
                                  math.sin(math.radians(35)), 0.0])
        laptop_screen.rotate(62 * DEGREES, axis=screen_right)
        laptop_screen.rotate(35 * DEGREES, axis=OUT)

        # Screen content — blue dashboard glow
        screen_glow = Rectangle(
            width=0.50, height=0.28,
            fill_color="#2563eb", fill_opacity=0.25,
            stroke_color="#3b82f6", stroke_width=1.2,
        )
        screen_glow.move_to(screen_pos)
        screen_glow.rotate(62 * DEGREES, axis=screen_right)
        screen_glow.rotate(35 * DEGREES, axis=OUT)

        self.add(laptop_base, laptop_screen, screen_glow)

        # WiFi curved dashed line (robot → laptop), arcing upward like a ballistic path.
        wifi_start = robot_3d + OUT * 0.05
        wifi_end = laptop_pos + screen_base_dir * 0.10 + OUT * 0.10
        wifi_arc = ArcBetweenPoints(
            wifi_start,
            wifi_end,
            radius=np.linalg.norm(wifi_start - wifi_end) * 1.20,
            stroke_color="#15803d",
            stroke_width=3.0,
            stroke_opacity=0.85,
        )
        wifi_dashed = DashedVMobject(wifi_arc, num_dashes=20, dashed_ratio=0.5)
        self.add(wifi_dashed)

        self.wait(1e-3)