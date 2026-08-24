"""
Delta Robot Master Controller (Python GUI)
-------------------------------------------
This program is the "master" brain of the delta robot.
It calculates ALL inverse kinematics and simply sends target
step positions + move timing to the ESP32 ("slave") over serial.

The ESP32 does NOT do any math - it only pulses step/dir (or CW/CCW)
pins to reach the step counts it is told, in the time it is told.

Author: Generated for user delta robot project
Requires: pip install pyserial
"""

import math
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports


# =========================================================
#                DELTA ROBOT KINEMATICS
# =========================================================
# Standard delta robot inverse kinematics (3 towers spaced
# 120 degrees apart). Returns motor angles in degrees for
# each of the 3 arms given a target end-effector position
# (x0, y0, z0) in mm, relative to the robot's center at the
# home/reference height. z0 should be NEGATIVE (effector is
# below the base plane) for a typical delta robot.
#
# Geometry parameters (all in mm):
#   f  = base radius       (center of base to base joint)
#   e  = effector radius   (center of effector to effector joint)
#   rf = bicep length      (upper arm, motor to elbow)
#   re = forearm length    (elbow to effector, parallel rods)

SQRT3 = math.sqrt(3.0)
SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN60 = SQRT3
TAN30 = 1.0 / SQRT3


class DeltaKinematics:
    def __init__(self, f=100.0, e=50.0, rf=150.0, re=280.0):
        self.f = f
        self.e = e
        self.rf = rf
        self.re = re

    def _calc_angle_yz(self, x0, y0, z0):
        """Helper: solves for one tower's angle assuming the problem
        has been rotated so that the tower lies in the Y-Z plane."""
        f, e, rf, re = self.f, self.e, self.rf, self.re

        y1 = -0.5 * TAN30 * f          # base joint y coordinate
        y0 = y0 - 0.5 * TAN30 * e      # shift center to edge of effector

        a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0

        d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
        if d < 0:
            return None  # target position not reachable

        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj
        theta = math.degrees(math.atan2(-zj, (y1 - yj)))
        return theta

    def inverse(self, x0, y0, z0):
        """Returns (theta1, theta2, theta3) in degrees, or None if
        the point is unreachable by the geometry."""
        t1 = self._calc_angle_yz(x0, y0, z0)
        if t1 is None:
            return None

        t2 = self._calc_angle_yz(x0 * COS120 + y0 * SIN120,
                                  y0 * COS120 - x0 * SIN120,
                                  z0)
        if t2 is None:
            return None

        t3 = self._calc_angle_yz(x0 * COS120 - y0 * SIN120,
                                  y0 * COS120 + x0 * SIN120,
                                  z0)
        if t3 is None:
            return None

        return (t1, t2, t3)


# =========================================================
#                SERIAL COMMUNICATION LAYER
# =========================================================
class SerialLink:
    """Handles the serial connection to the ESP32 in a background
    thread so the GUI never freezes while waiting on the port."""

    def __init__(self, rx_queue: queue.Queue):
        self.ser = None
        self.rx_queue = rx_queue
        self._stop_flag = threading.Event()
        self._thread = None

    def connect(self, port, baud=115200):
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2.0)  # allow ESP32 to reset after opening port
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        self._stop_flag.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def send_line(self, text):
        if not self.is_connected():
            raise ConnectionError("Not connected to ESP32")
        line = text.strip() + "\n"
        self.ser.write(line.encode("utf-8"))
        self.rx_queue.put(("TX", line.strip()))

    def _read_loop(self):
        buf = b""
        while not self._stop_flag.is_set():
            try:
                if self.ser and self.ser.in_waiting:
                    buf += self.ser.read(self.ser.in_waiting)
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        text = line.decode("utf-8", errors="replace").strip()
                        if text:
                            self.rx_queue.put(("RX", text))
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                self.rx_queue.put(("RX", "[ERROR] Serial link lost"))
                break


# =========================================================
#                       MAIN GUI
# =========================================================
class DeltaRobotApp:
    STEPS_PER_DEG_DEFAULT = 3200 / 360.0  # 3200 steps/rev (1/16 microstep on 200 step motor)

    def __init__(self, root):
        self.root = root
        self.root.title("Delta Robot Master Controller")
        self.root.geometry("900x650")

        self.rx_queue = queue.Queue()
        self.link = SerialLink(self.rx_queue)
        self.kin = DeltaKinematics()

        # Current known step position of each motor (as tracked by python,
        # the "master"). ESP32 trusts these completely - it has no idea
        # about mm or angles, only step counts.
        self.current_steps = [0, 0, 0]
        self.current_xyz = [0.0, 0.0, -200.0]  # sensible starting guess

        self._build_ui()
        self._poll_serial()

    # ---------------------------------------------------
    # UI CONSTRUCTION
    # ---------------------------------------------------
    def _build_ui(self):
        pad = {"padx": 6, "pady": 4}

        # ---------- Connection frame ----------
        conn = ttk.LabelFrame(self.root, text="Connection")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_combo = ttk.Combobox(conn, width=18, state="readonly")
        self.port_combo.grid(row=0, column=1, **pad)
        self._refresh_ports()

        ttk.Button(conn, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, **pad)

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, **pad)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn, textvariable=self.baud_var, width=10).grid(row=0, column=4, **pad)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=5, **pad)

        self.status_lbl = ttk.Label(conn, text="Disconnected", foreground="red")
        self.status_lbl.grid(row=0, column=6, **pad)

        # ---------- Geometry frame ----------
        geo = ttk.LabelFrame(self.root, text="Robot Geometry / Calibration")
        geo.pack(fill="x", **pad)

        self.geo_vars = {}
        fields = [
            ("Base radius f (mm)", "f", 100.0),
            ("Effector radius e (mm)", "e", 50.0),
            ("Bicep rf (mm)", "rf", 150.0),
            ("Forearm re (mm)", "re", 280.0),
            ("Steps/rev (with microstep)", "steps_per_rev", 3200.0),
            ("Max speed (mm/s)", "max_speed", 80.0),
        ]
        for i, (label, key, default) in enumerate(fields):
            ttk.Label(geo, text=label).grid(row=i // 3, column=(i % 3) * 2, sticky="e", **pad)
            v = tk.StringVar(value=str(default))
            ttk.Entry(geo, textvariable=v, width=10).grid(row=i // 3, column=(i % 3) * 2 + 1, **pad)
            self.geo_vars[key] = v

        ttk.Button(geo, text="Apply Geometry", command=self._apply_geometry).grid(
            row=2, column=0, columnspan=2, **pad)

        # per-axis angle offset & direction inversion (fixes wiring/assembly quirks)
        self.axis_offset = [tk.StringVar(value="0.0") for _ in range(3)]
        self.axis_invert = [tk.BooleanVar(value=False) for _ in range(3)]
        for i in range(3):
            ttk.Label(geo, text=f"Axis {i+1} offset (deg)").grid(row=3, column=i * 2, **pad)
            ttk.Entry(geo, textvariable=self.axis_offset[i], width=6).grid(row=3, column=i * 2 + 1, **pad)
            ttk.Checkbutton(geo, text=f"Invert {i+1}", variable=self.axis_invert[i]).grid(
                row=4, column=i * 2, columnspan=2, **pad)

        # ---------- Position control frame ----------
        pos = ttk.LabelFrame(self.root, text="Move To Position (mm)")
        pos.pack(fill="x", **pad)

        self.xyz_vars = [tk.StringVar(value=str(v)) for v in self.current_xyz]
        for i, axis in enumerate(["X", "Y", "Z"]):
            ttk.Label(pos, text=axis).grid(row=0, column=i * 2, **pad)
            ttk.Entry(pos, textvariable=self.xyz_vars[i], width=10).grid(row=0, column=i * 2 + 1, **pad)

        ttk.Button(pos, text="Move", command=self._move_to_entry_position).grid(row=0, column=6, **pad)

        # jog controls
        jog = ttk.Frame(pos)
        jog.grid(row=1, column=0, columnspan=7, pady=6)
        ttk.Label(jog, text="Jog step (mm):").grid(row=0, column=0, **pad)
        self.jog_step = tk.StringVar(value="5.0")
        ttk.Entry(jog, textvariable=self.jog_step, width=6).grid(row=0, column=1, **pad)

        jog_buttons = [
            ("X-", (-1, 0, 0)), ("X+", (1, 0, 0)),
            ("Y-", (0, -1, 0)), ("Y+", (0, 1, 0)),
            ("Z-", (0, 0, -1)), ("Z+", (0, 0, 1)),
        ]
        for i, (label, delta) in enumerate(jog_buttons):
            ttk.Button(jog, text=label, width=4,
                       command=lambda d=delta: self._jog(d)).grid(row=0, column=2 + i, **pad)

        # ---------- Action buttons ----------
        act = ttk.Frame(self.root)
        act.pack(fill="x", **pad)
        ttk.Button(act, text="Home", command=self._home).pack(side="left", **pad)
        ttk.Button(act, text="Enable Motors", command=lambda: self._send("EN 1")).pack(side="left", **pad)
        ttk.Button(act, text="Disable Motors", command=lambda: self._send("EN 0")).pack(side="left", **pad)
        ttk.Button(act, text="STOP", command=lambda: self._send("STOP")).pack(side="left", **pad)
        ttk.Button(act, text="Query Position", command=lambda: self._send("P?")).pack(side="left", **pad)

        # ---------- Log console ----------
        log_frame = ttk.LabelFrame(self.root, text="Serial Log")
        log_frame.pack(fill="both", expand=True, **pad)
        self.log_text = tk.Text(log_frame, height=15, state="disabled", wrap="none")
        self.log_text.pack(fill="both", expand=True, side="left")
        scroll = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scroll.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=scroll.set)

        self._apply_geometry()  # populate self.geometry values from defaults

    # ---------------------------------------------------
    # SERIAL HELPERS
    # ---------------------------------------------------
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connect(self):
        if self.link.is_connected():
            self.link.disconnect()
            self.connect_btn.config(text="Connect")
            self.status_lbl.config(text="Disconnected", foreground="red")
        else:
            port = self.port_combo.get()
            if not port:
                messagebox.showerror("Error", "Select a COM port first.")
                return
            try:
                self.link.connect(port, int(self.baud_var.get()))
                self.connect_btn.config(text="Disconnect")
                self.status_lbl.config(text="Connected", foreground="green")
            except Exception as ex:
                messagebox.showerror("Connection failed", str(ex))

    def _send(self, text):
        try:
            self.link.send_line(text)
        except ConnectionError as ex:
            messagebox.showerror("Not connected", str(ex))

    def _poll_serial(self):
        while not self.rx_queue.empty():
            direction, text = self.rx_queue.get_nowait()
            self._log(f"{direction}: {text}")
        self.root.after(50, self._poll_serial)

    def _log(self, text):
        self.log_text.config(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ---------------------------------------------------
    # GEOMETRY / CALIBRATION
    # ---------------------------------------------------
    def _apply_geometry(self):
        try:
            f = float(self.geo_vars["f"].get())
            e = float(self.geo_vars["e"].get())
            rf = float(self.geo_vars["rf"].get())
            re = float(self.geo_vars["re"].get())
            self.kin = DeltaKinematics(f=f, e=e, rf=rf, re=re)
            self.steps_per_rev = float(self.geo_vars["steps_per_rev"].get())
            self.steps_per_deg = self.steps_per_rev / 360.0
            self.max_speed = float(self.geo_vars["max_speed"].get())
            self._log("[INFO] Geometry updated.")
        except ValueError:
            messagebox.showerror("Error", "Geometry fields must be numeric.")

    # ---------------------------------------------------
    # MOTION COMMANDS
    # ---------------------------------------------------
    def _angles_to_steps(self, angles):
        """Convert 3 motor angles (deg) into absolute step counts,
        applying per-axis calibration offset and direction inversion."""
        steps = []
        for i, theta in enumerate(angles):
            offset = float(self.axis_offset[i].get())
            corrected = theta + offset
            if self.axis_invert[i].get():
                corrected = -corrected
            steps.append(int(round(corrected * self.steps_per_deg)))
        return steps

    def _move_to_xyz(self, x, y, z):
        angles = self.kin.inverse(x, y, z)
        if angles is None:
            messagebox.showerror("Unreachable", f"Point ({x:.1f}, {y:.1f}, {z:.1f}) "
                                                  "is outside the robot's workspace.")
            return

        target_steps = self._angles_to_steps(angles)

        # figure out how long the move should take based on desired mm/s speed
        dx = x - self.current_xyz[0]
        dy = y - self.current_xyz[1]
        dz = z - self.current_xyz[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = max(self.max_speed, 1.0)
        duration_ms = max(int((distance / speed) * 1000.0), 20)  # 20ms floor

        cmd = f"M {target_steps[0]} {target_steps[1]} {target_steps[2]} {duration_ms}"
        self._send(cmd)

        self.current_steps = target_steps
        self.current_xyz = [x, y, z]
        self._log(f"[INFO] Target angles: {[round(a,2) for a in angles]} deg "
                   f"-> steps {target_steps} over {duration_ms} ms")

    def _move_to_entry_position(self):
        try:
            x = float(self.xyz_vars[0].get())
            y = float(self.xyz_vars[1].get())
            z = float(self.xyz_vars[2].get())
        except ValueError:
            messagebox.showerror("Error", "X/Y/Z must be numeric.")
            return
        self._move_to_xyz(x, y, z)

    def _jog(self, delta):
        step = float(self.jog_step.get())
        x = self.current_xyz[0] + delta[0] * step
        y = self.current_xyz[1] + delta[1] * step
        z = self.current_xyz[2] + delta[2] * step
        self.xyz_vars[0].set(f"{x:.2f}")
        self.xyz_vars[1].set(f"{y:.2f}")
        self.xyz_vars[2].set(f"{z:.2f}")
        self._move_to_xyz(x, y, z)

    def _home(self):
        # Tell ESP32 to run its own homing routine (limit switches).
        # After homing completes, ESP32 will report back and we reset
        # our tracked step/position state to the known home pose.
        self._send("H")
        self.current_steps = [0, 0, 0]
        # NOTE: set this to whatever XYZ your home position physically is
        self.current_xyz = [0.0, 0.0, -200.0]
        self.xyz_vars[0].set("0.0")
        self.xyz_vars[1].set("0.0")
        self.xyz_vars[2].set("-200.0")


if __name__ == "__main__":
    root = tk.Tk()
    app = DeltaRobotApp(root)
    root.mainloop()
