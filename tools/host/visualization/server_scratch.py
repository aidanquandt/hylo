import socket
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk

HOST = "0.0.0.0"
PORT = 5000

events = queue.Queue()      # Thread -> GUI
send_queue = queue.Queue()  # GUI -> client
control_queue = queue.Queue()  # GUI -> socket control


def socket_worker(host: str, port: int):
    """Persistent TCP server that can disconnect clients on request."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(1)

            events.put(("status", f"Listening on {host}:{port}"))

            while True:
                events.put(("status", "Waiting for client..."))
                conn, addr = srv.accept()

                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

                events.put(("status", f"Connected: {addr[0]}:{addr[1]}"))
                conn.settimeout(0.1)

                buf = bytearray()

                with conn:
                    while True:
                        # --- check GUI control commands ---
                        try:
                            cmd = control_queue.get_nowait()
                            if cmd == "DISCONNECT_CLIENT":
                                events.put(("status", "Client disconnected by GUI"))
                                conn.close()
                                break
                        except queue.Empty:
                            pass

                        # --- outgoing messages ---
                        try:
                            msg = send_queue.get_nowait()
                            data = (msg + "\r\n").encode("utf-8")
                            try:
                                conn.sendall(data)
                            except OSError:
                                events.put(("status", "Connection lost"))
                                break
                            events.put(("sent", msg))
                        except queue.Empty:
                            pass

                        # --- incoming data ---
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue

                        if not chunk:
                            events.put(("status", "Client disconnected"))
                            break

                        buf += chunk

                        while True:
                            nl = buf.find(b"\n")
                            if nl == -1:
                                break

                            line = buf[:nl]
                            del buf[:nl + 1]

                            if line.endswith(b"\r"):
                                line = line[:-1]

                            text = line.decode("utf-8", errors="replace")
                            events.put(("msg", text))

    except Exception as e:
        events.put(("status", f"ERROR: {e!r}"))


class Dashboard(tk.Tk):

    def __init__(self):
        super().__init__()

        self.title("TCP Telemetry Dashboard")
        self.geometry("800x600+100+20")

        self.status_var = tk.StringVar(value="Starting…")
        self.total_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="0.0 msg/s")
        self.sent_count_var = tk.StringVar(value="0")

        # IMU data variables
        self.imu_ax = tk.StringVar(value="--")
        self.imu_ay = tk.StringVar(value="--")
        self.imu_az = tk.StringVar(value="--")
        self.imu_gx = tk.StringVar(value="--")
        self.imu_gy = tk.StringVar(value="--")
        self.imu_gz = tk.StringVar(value="--")
        self.imu_temp = tk.StringVar(value="--")
        self.imu_count = tk.StringVar(value="0")

        # Position data variables
        self.pos_x = tk.StringVar(value="--")
        self.pos_y = tk.StringVar(value="--")
        self.pos_z = tk.StringVar(value="--")
        self.pos_vx = tk.StringVar(value="--")
        self.pos_vy = tk.StringVar(value="--")
        self.pos_vz = tk.StringVar(value="--")
        self.pos_conf = tk.StringVar(value="--")
        self.pos_valid = tk.StringVar(value="--")
        self.pos_count = tk.StringVar(value="0")

        # Ranging data variables
        self.range_dist = tk.StringVar(value="--")
        self.range_rssi = tk.StringVar(value="--")
        self.range_addr = tk.StringVar(value="--")
        self.range_x = tk.StringVar(value="--")
        self.range_y = tk.StringVar(value="--")
        self.range_z = tk.StringVar(value="--")
        self.range_valid = tk.StringVar(value="--")
        self.range_count = tk.StringVar(value="0")

        self.total_count = 0
        self.sent_count = 0
        self.last_second = time.monotonic()
        self.count_this_second = 0

        self.imu_msg_count = 0
        self.pos_msg_count = 0
        self.range_msg_count = 0

        self._build_ui()

        self.after(50, self._drain_events)
        self.after(250, self._update_rate)

    def _build_ui(self):
        title_font = ("Segoe UI", 18, "bold")

        # Create a canvas and scrollbar for scrolling
        canvas = tk.Canvas(self, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bind mousewheel to scroll
        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        # Now build all UI elements inside scrollable_frame instead of self
        header = ttk.Label(scrollable_frame, text="Live Receiver", font=title_font)
        header.pack(anchor="w", padx=20, pady=14)

        self.status_big = ttk.Label(
            scrollable_frame,
            textvariable=self.status_var,
            font=("Segoe UI", 14, "bold"),
            foreground="orange",
        )
        self.status_big.pack(anchor="w", padx=20)

        # --- Statistics Row ---
        stats_frame = ttk.Frame(scrollable_frame)
        stats_frame.pack(fill="x", padx=24, pady=8)

        ttk.Label(stats_frame, text="Total:", font=("Segoe UI", 10)).pack(side="left", padx=5)
        ttk.Label(stats_frame, textvariable=self.total_var, font=("Consolas", 10, "bold")).pack(side="left", padx=5)
        
        ttk.Label(stats_frame, text="Rate:", font=("Segoe UI", 10)).pack(side="left", padx=(20, 5))
        ttk.Label(stats_frame, textvariable=self.rate_var, font=("Consolas", 10, "bold")).pack(side="left", padx=5)

        # --- Create main layout with 2 columns: telemetry (left) and controls (right) ---
        main_container = ttk.Frame(scrollable_frame)
        main_container.pack(fill="both", expand=True, padx=24, pady=10)
        main_container.columnconfigure(0, weight=2)
        main_container.columnconfigure(1, weight=1)

        # Left side: 3-column layout for telemetry data
        telemetry_container = ttk.Frame(main_container)
        telemetry_container.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        telemetry_container.columnconfigure(0, weight=1)
        telemetry_container.columnconfigure(1, weight=1)
        telemetry_container.columnconfigure(2, weight=1)

        # IMU Data Box
        self._build_imu_box(telemetry_container)

        # Position Data Box
        self._build_position_box(telemetry_container)

        # Ranging Data Box
        self._build_ranging_box(telemetry_container)

        # Right side: Controls container
        controls_container = ttk.Frame(main_container)
        controls_container.grid(row=0, column=1, sticky="nsew")

        # ---- SEND COMMANDS ----
        send_frame = ttk.LabelFrame(controls_container, text="Send OTA Commands", padding=10)
        send_frame.pack(fill="x", pady=(0, 10))

        # Quick OTA buttons with text inputs
        quick_frame = ttk.Frame(send_frame)
        quick_frame.pack(fill="x", padx=5, pady=5)

        # IMU
        ttk.Label(quick_frame, text="IMU:").grid(row=0, column=0, sticky="e", padx=(0, 5), pady=3)
        self.imu_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.imu_entry.grid(row=0, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set IMU",
            command=lambda: self._send_ota("set-imu-off", self.imu_entry.get())
        ).grid(row=0, column=2, sticky="ew", padx=5, pady=3)

        # FUSION
        ttk.Label(quick_frame, text="Fusion:").grid(row=1, column=0, sticky="e", padx=(0, 5), pady=3)
        self.fusion_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.fusion_entry.grid(row=1, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set Fusion",
            command=lambda: self._send_ota("set-fusion", self.fusion_entry.get())
        ).grid(row=1, column=2, sticky="ew", padx=5, pady=3)

        # ANCHOR 0
        ttk.Label(quick_frame, text="Anchor 0:").grid(row=2, column=0, sticky="e", padx=(0, 5), pady=3)
        self.anchor0_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.anchor0_entry.grid(row=2, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set Anchor 0",
            command=lambda: self._send_ota("set-anchor", 0, self.anchor0_entry.get())
        ).grid(row=2, column=2, sticky="ew", padx=5, pady=3)

        # ANCHOR 1
        ttk.Label(quick_frame, text="Anchor 1:").grid(row=3, column=0, sticky="e", padx=(0, 5), pady=3)
        self.anchor1_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.anchor1_entry.grid(row=3, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set Anchor 1",
            command=lambda: self._send_ota("set-anchor", 1, self.anchor1_entry.get())
        ).grid(row=3, column=2, sticky="ew", padx=5, pady=3)

        # ANCHOR 2
        ttk.Label(quick_frame, text="Anchor 2:").grid(row=4, column=0, sticky="e", padx=(0, 5), pady=3)
        self.anchor2_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.anchor2_entry.grid(row=4, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set Anchor 2",
            command=lambda: self._send_ota("set-anchor", 2, self.anchor2_entry.get())
        ).grid(row=4, column=2, sticky="ew", padx=5, pady=3)

        # ANCHOR 3
        ttk.Label(quick_frame, text="Anchor 3:").grid(row=5, column=0, sticky="e", padx=(0, 5), pady=3)
        self.anchor3_entry = ttk.Entry(quick_frame, width=10, font=("Consolas", 10))
        self.anchor3_entry.grid(row=5, column=1, sticky="ew", padx=2, pady=3)
        ttk.Button(
            quick_frame,
            text="Set Anchor 3",
            command=lambda: self._send_ota("set-anchor", 3, self.anchor3_entry.get())
        ).grid(row=5, column=2, sticky="ew", padx=5, pady=3)

        quick_frame.columnconfigure(1, weight=1)

        # ---- CONTROL BUTTONS ----
        ctrl = ttk.LabelFrame(controls_container, text="Server Controls", padding=10)
        ctrl.pack(fill="x", pady=(0, 10))

        ttk.Button(
            ctrl,
            text="Disconnect Client",
            command=self._disconnect_client
        ).pack(fill="x", padx=5, pady=3)

        ttk.Button(
            ctrl,
            text="Reset Counters",
            command=self._reset
        ).pack(fill="x", padx=5, pady=3)

        # ---- STATS ----
        stats_info = ttk.LabelFrame(controls_container, text="Statistics", padding=10)
        stats_info.pack(fill="x")
        
        stats = ttk.Label(
            stats_info,
            textvariable=self.sent_count_var,
            font=("Segoe UI", 11),
            foreground="blue"
        )
        stats.pack(anchor="w", pady=5)

    def _build_imu_box(self, parent):
        """Build IMU data display box."""
        imu_frame = ttk.LabelFrame(parent, text="IMU Data", padding=10)
        imu_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

        label_font = ("Segoe UI", 9)
        value_font = ("Consolas", 10, "bold")

        def imu_row(lbl, var, r):
            ttk.Label(imu_frame, text=lbl, font=label_font).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Label(imu_frame, textvariable=var, font=value_font, foreground="navy").grid(row=r, column=1, sticky="e", pady=3)

        imu_row("Accel X:", self.imu_ax, 0)
        imu_row("Accel Y:", self.imu_ay, 1)
        imu_row("Accel Z:", self.imu_az, 2)
        imu_row("Gyro X:", self.imu_gx, 3)
        imu_row("Gyro Y:", self.imu_gy, 4)
        imu_row("Gyro Z:", self.imu_gz, 5)
        imu_row("Temp:", self.imu_temp, 6)
        
        ttk.Separator(imu_frame, orient="horizontal").grid(row=7, column=0, columnspan=2, sticky="ew", pady=8)
        imu_row("Messages:", self.imu_count, 8)

        imu_frame.columnconfigure(1, weight=1)

    def _build_position_box(self, parent):
        """Build position data display box."""
        pos_frame = ttk.LabelFrame(parent, text="Position Data", padding=10)
        pos_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

        label_font = ("Segoe UI", 9)
        value_font = ("Consolas", 10, "bold")

        def pos_row(lbl, var, r):
            ttk.Label(pos_frame, text=lbl, font=label_font).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Label(pos_frame, textvariable=var, font=value_font, foreground="darkgreen").grid(row=r, column=1, sticky="e", pady=3)

        pos_row("X:", self.pos_x, 0)
        pos_row("Y:", self.pos_y, 1)
        pos_row("Z:", self.pos_z, 2)
        pos_row("Vel X:", self.pos_vx, 3)
        pos_row("Vel Y:", self.pos_vy, 4)
        pos_row("Vel Z:", self.pos_vz, 5)
        pos_row("Confidence:", self.pos_conf, 6)
        pos_row("Valid:", self.pos_valid, 7)
        
        ttk.Separator(pos_frame, orient="horizontal").grid(row=8, column=0, columnspan=2, sticky="ew", pady=8)
        pos_row("Messages:", self.pos_count, 9)

        pos_frame.columnconfigure(1, weight=1)

    def _build_ranging_box(self, parent):
        """Build ranging data display box."""
        range_frame = ttk.LabelFrame(parent, text="Ranging Data", padding=10)
        range_frame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

        label_font = ("Segoe UI", 9)
        value_font = ("Consolas", 10, "bold")

        def range_row(lbl, var, r):
            ttk.Label(range_frame, text=lbl, font=label_font).grid(row=r, column=0, sticky="w", pady=3)
            ttk.Label(range_frame, textvariable=var, font=value_font, foreground="darkred").grid(row=r, column=1, sticky="e", pady=3)

        range_row("Distance:", self.range_dist, 0)
        range_row("RSSI:", self.range_rssi, 1)
        range_row("Address:", self.range_addr, 2)
        range_row("Anchor X:", self.range_x, 3)
        range_row("Anchor Y:", self.range_y, 4)
        range_row("Anchor Z:", self.range_z, 5)
        range_row("Valid:", self.range_valid, 6)
        
        ttk.Separator(range_frame, orient="horizontal").grid(row=7, column=0, columnspan=2, sticky="ew", pady=8)
        range_row("Messages:", self.range_count, 8)

        range_frame.columnconfigure(1, weight=1)

    # --- OTA Command Sender ---
    def _send_ota(self, cmd, *args):
        full_cmd = "OTA " + cmd
        if args:
            full_cmd += " " + " ".join(str(a) for a in args)
        send_queue.put(full_cmd)

    def _disconnect_client(self):
        control_queue.put("DISCONNECT_CLIENT")

    def _reset(self):
        self.total_count = 0
        self.sent_count = 0
        self.count_this_second = 0
        self.imu_msg_count = 0
        self.pos_msg_count = 0
        self.range_msg_count = 0

        self.total_var.set("0")
        self.sent_count_var.set("Commands sent: 0")
        self.rate_var.set("0.0 msg/s")
        self.imu_count.set("0")
        self.pos_count.set("0")
        self.range_count.set("0")

    def _set_status_color(self, status_text):
        if "ERROR" in status_text:
            color = "red"
        elif "Connected" in status_text:
            color = "green"
        else:
            color = "orange"

        self.status_big.configure(foreground=color)

    def _parse_telemetry(self, msg):
        """Parse CSV telemetry messages and update displays."""
        parts = msg.split(",")
        if len(parts) < 2:
            return

        msg_type = parts[0].strip()

        try:
            if msg_type == "IMU" and len(parts) >= 8:
                # IMU,ax,ay,az,gx,gy,gz,temp
                self.imu_ax.set(f"{float(parts[1]):7.3f}")
                self.imu_ay.set(f"{float(parts[2]):7.3f}")
                self.imu_az.set(f"{float(parts[3]):7.3f}")
                self.imu_gx.set(f"{float(parts[4]):7.3f}")
                self.imu_gy.set(f"{float(parts[5]):7.3f}")
                self.imu_gz.set(f"{float(parts[6]):7.3f}")
                self.imu_temp.set(f"{float(parts[7]):5.1f}°C")
                self.imu_msg_count += 1
                self.imu_count.set(str(self.imu_msg_count))

            elif msg_type == "POS" and len(parts) >= 9:
                # POS,x,y,z,vx,vy,vz,conf,valid
                self.pos_x.set(f"{float(parts[1]):7.3f} m")
                self.pos_y.set(f"{float(parts[2]):7.3f} m")
                self.pos_z.set(f"{float(parts[3]):7.3f} m")
                self.pos_vx.set(f"{float(parts[4]):7.3f} m/s")
                self.pos_vy.set(f"{float(parts[5]):7.3f} m/s")
                self.pos_vz.set(f"{float(parts[6]):7.3f} m/s")
                self.pos_conf.set(f"{float(parts[7]):5.2f}")
                self.pos_valid.set("Yes" if int(parts[8]) else "No")
                self.pos_msg_count += 1
                self.pos_count.set(str(self.pos_msg_count))

            elif msg_type == "RANGE" and len(parts) >= 8:
                # RANGE,dist,rssi,addr,x,y,z,valid
                self.range_dist.set(f"{float(parts[1]):6.3f} m")
                self.range_rssi.set(f"{float(parts[2]):5.1f} dBm")
                self.range_addr.set(f"0x{int(parts[3]):04X}")
                self.range_x.set(f"{float(parts[4]):6.2f} m")
                self.range_y.set(f"{float(parts[5]):6.2f} m")
                self.range_z.set(f"{float(parts[6]):6.2f} m")
                self.range_valid.set("Yes" if int(parts[7]) else "No")
                self.range_msg_count += 1
                self.range_count.set(str(self.range_msg_count))

        except (ValueError, IndexError) as e:
            # Ignore malformed messages
            pass

    # --- Event Drain ---
    def _drain_events(self):
        try:
            while True:
                kind, payload = events.get_nowait()

                if kind == "status":
                    self.status_var.set(payload)
                    self._set_status_color(payload)

                elif kind == "msg":
                    self.total_count += 1
                    self.count_this_second += 1
                    self.total_var.set(str(self.total_count))
                    
                    # Parse and update telemetry displays
                    self._parse_telemetry(payload)

                elif kind == "sent":
                    self.sent_count += 1
                    self.sent_count_var.set(f"Commands sent: {self.sent_count}")

        except queue.Empty:
            pass

        self.after(50, self._drain_events)

    # --- Rate updater ---
    def _update_rate(self):
        now = time.monotonic()
        dt = now - self.last_second

        if dt >= 1.0:
            rate = self.count_this_second / dt
            self.rate_var.set(f"{rate:.1f} msg/s")

            self.count_this_second = 0
            self.last_second = now

        self.after(250, self._update_rate)


def main():
    t = threading.Thread(
        target=socket_worker,
        args=(HOST, PORT),
        daemon=True
    )
    t.start()

    app = Dashboard()
    app.mainloop()


if __name__ == "__main__":
    main()
