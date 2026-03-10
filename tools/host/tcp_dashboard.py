import socket
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk

HOST = "0.0.0.0"
PORT = 5000

# Thread -> GUI message bus
events = queue.Queue()

# GUI -> Thread command bus
send_queue = queue.Queue()

def socket_worker(host: str, port: int):
    """Accept one TCP client and stream received lines into the GUI via a queue."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(1)
            events.put(("status", f"Listening on {host}:{port}"))

            conn, addr = srv.accept()
            events.put(("status", f"Connected: {addr[0]}:{addr[1]}"))

            conn.settimeout(0.1)  # Short timeout for checking send_queue
            buf = bytearray()

            with conn:
                while True:
                    # Check for outgoing messages to send
                    try:
                        msg = send_queue.get_nowait()
                        data = (msg + "\r\n").encode("utf-8")
                        conn.sendall(data)
                        events.put(("sent", msg))
                    except queue.Empty:
                        pass

                    # Receive incoming data
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue
                    
                    if not chunk:
                        events.put(("status", "Client disconnected"))
                        break

                    buf += chunk

                    # Parse CRLF or LF-delimited messages
                    while True:
                        nl = buf.find(b"\n")
                        if nl == -1:
                            break
                        line = buf[:nl]         # excludes '\n'
                        del buf[:nl + 1]

                        # Remove optional '\r'
                        if line.endswith(b"\r"):
                            line = line[:-1]

                        text = line.decode("utf-8", errors="replace")
                        events.put(("msg", text))

    except Exception as e:
        events.put(("status", f"ERROR: {e!r}"))

class Dashboard(tk.Tk):
    def __init__(self):
        super().__init__()

        # Bigger, dashboard-like window
        self.title("TCP Telemetry Dashboard")
        self.geometry("600x650")
        self.minsize(600, 650)

        # UI state vars
        self.status_var = tk.StringVar(value="Starting…")
        self.last_msg_var = tk.StringVar(value="(none)")
        self.total_var = tk.StringVar(value="0")
        self.rate_var = tk.StringVar(value="0.0 msg/s")
        self.last_time_var = tk.StringVar(value="(never)")
        self.sent_count_var = tk.StringVar(value="0")

        # Stats
        self.total_count = 0
        self.sent_count = 0
        self.last_second = time.monotonic()
        self.count_this_second = 0

        self._build_ui()

        # Poll queue + update rate display
        self.after(50, self._drain_events)
        self.after(250, self._update_rate)

    def _build_ui(self):
        # Larger fonts + padding
        title_font = ("Segoe UI", 18, "bold")
        label_font = ("Segoe UI", 13)
        value_font = ("Consolas", 14)

        pad = {"padx": 20, "pady": 14}

        header = ttk.Label(self, text="Live Receiver", font=title_font)
        header.pack(anchor="w", **pad)

        # Big status line with color
        self.status_big = ttk.Label(
            self,
            textvariable=self.status_var,
            font=("Segoe UI", 14, "bold"),
            foreground="orange",
        )
        self.status_big.pack(anchor="w", padx=20, pady=(0, 10))

        # ===== RECEIVE SECTION =====
        recv_frame = ttk.LabelFrame(self, text="📥 Received Data", padding=10)
        recv_frame.pack(fill="both", expand=True, padx=24, pady=(0, 10))

        def row(parent, label, var, r):
            ttk.Label(parent, text=label, font=label_font)\
                .grid(row=r, column=0, sticky="w", padx=12, pady=8)

            ttk.Label(parent, textvariable=var, font=value_font)\
                .grid(row=r, column=1, sticky="w", padx=12, pady=8)

        row(recv_frame, "Last message:", self.last_msg_var, 0)
        row(recv_frame, "Total messages:", self.total_var, 1)
        row(recv_frame, "Rate:", self.rate_var, 2)
        row(recv_frame, "Last update:", self.last_time_var, 3)

        recv_frame.columnconfigure(1, weight=1)

        # ===== SEND SECTION =====
        send_frame = ttk.LabelFrame(self, text="📤 Send Commands", padding=10)
        send_frame.pack(fill="x", padx=24, pady=(10, 10))

        ttk.Label(send_frame, text="Command:", font=label_font)\
            .grid(row=0, column=0, sticky="w", padx=12, pady=8)

        self.cmd_entry = ttk.Entry(send_frame, font=value_font, width=30)
        self.cmd_entry.grid(row=0, column=1, sticky="ew", padx=12, pady=8)
        self.cmd_entry.bind("<Return>", lambda e: self._send_command())

        ttk.Button(send_frame, text="Send", command=self._send_command)\
            .grid(row=0, column=2, padx=12, pady=8)

        send_frame.columnconfigure(1, weight=1)

        # Quick command buttons
        quick_frame = ttk.Frame(send_frame)
        quick_frame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=12, pady=(0, 8))

        ttk.Label(quick_frame, text="Quick commands:", font=("Segoe UI", 11))\
            .pack(side="left", padx=(0, 10))

        ttk.Button(quick_frame, text="LED ON", command=lambda: self._quick_send("LED_ON"))\
            .pack(side="left", padx=2)
        ttk.Button(quick_frame, text="LED OFF", command=lambda: self._quick_send("LED_OFF"))\
            .pack(side="left", padx=2)
        ttk.Button(quick_frame, text="STATUS", command=lambda: self._quick_send("GET_STATUS"))\
            .pack(side="left", padx=2)
        ttk.Button(quick_frame, text="RESET", command=lambda: self._quick_send("RESET"))\
            .pack(side="left", padx=2)

        # Stats
        stats_label = ttk.Label(send_frame, textvariable=self.sent_count_var, 
                                font=("Segoe UI", 11), foreground="blue")
        stats_label.grid(row=2, column=0, columnspan=3, sticky="w", padx=12, pady=(5, 0))

        # ===== CONTROL BUTTONS =====
        btns = ttk.Frame(self)
        btns.pack(fill="x", padx=24, pady=16)

        ttk.Button(btns, text="Reset counters", command=self._reset).pack(side="left")

    def _send_command(self):
        """Send command from entry field"""
        cmd = self.cmd_entry.get().strip()
        if cmd:
            send_queue.put(cmd)
            self.cmd_entry.delete(0, tk.END)

    def _quick_send(self, cmd: str):
        """Send a quick command"""
        send_queue.put(cmd)

    def _reset(self):
        self.total_count = 0
        self.sent_count = 0
        self.count_this_second = 0
        self.total_var.set("0")
        self.sent_count_var.set("Commands sent: 0")
        self.rate_var.set("0.0 msg/s")
        self.last_msg_var.set("(none)")
        self.last_time_var.set("(never)")

    def _set_status_color(self, status_text: str):
        # Simple heuristic coloring
        if "ERROR" in status_text:
            color = "red"
        elif "Connected" in status_text:
            color = "green"
        else:
            color = "orange"
        self.status_big.configure(foreground=color)

    def _drain_events(self):
        """Pull all pending events from the socket thread."""
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
                    self.last_msg_var.set(payload)

                    # human-readable timestamp
                    ts = time.strftime("%Y-%m-%d %H:%M:%S")
                    self.last_time_var.set(ts)

                elif kind == "sent":
                    self.sent_count += 1
                    self.sent_count_var.set(f"Commands sent: {self.sent_count}")

        except queue.Empty:
            pass

        self.after(50, self._drain_events)

    def _update_rate(self):
        """Update msg/s roughly once per second using a rolling 1s counter."""
        now = time.monotonic()
        dt = now - self.last_second
        if dt >= 1.0:
            rate = self.count_this_second / dt
            self.rate_var.set(f"{rate:.1f} msg/s")
            self.count_this_second = 0
            self.last_second = now

        self.after(250, self._update_rate)

def main():
    # Start socket thread
    t = threading.Thread(target=socket_worker, args=(HOST, PORT), daemon=True)
    t.start()

    # Start GUI
    app = Dashboard()
    app.mainloop()

if __name__ == "__main__":
    main()
