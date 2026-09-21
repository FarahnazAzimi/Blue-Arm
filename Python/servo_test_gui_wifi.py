"""
Robotic Arm - Joint Test GUI (WiFi / HTTP version)
-----------------------------------------------------
Same GUI and same joint controls as servo_test_gui.py, but talks to the
robot over WiFi using HTTP requests - NO USB cable, NO serial/COM port.

HOW THIS COMPARES TO THE WEBPAGE
Your webpage's JavaScript does this:
    fetch('http://192.168.4.1/set?id=1&angle=90')
This Python script does the EXACT SAME thing, just using Python's
'requests' library instead of a browser:
    requests.get('http://192.168.4.1/set?id=1&angle=90')
Same protocol (HTTP), same ESP32 endpoints, same command format -
just a different program sending the request. This is a good example
of one protocol (HTTP) being used by two different clients.

BEFORE YOU RUN THIS
1. Install requests:   pip install requests
2. Connect your LAPTOP's WiFi to the "RoboticArm" network (password:
   arm12345) - the same network your phone uses for the webpage.
   NOTE: while connected to "RoboticArm", your laptop will NOT have
   normal internet access at the same time (unless it has a second
   connection, like Ethernet).
3. Make sure the ESP32 is powered on and running
   esp32_standalone_firmware.ino.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import requests
import threading

ESP32_IP = "192.168.4.1"
BASE_URL = f"http://{ESP32_IP}"
REQUEST_TIMEOUT = 2  # seconds - fail fast if not connected to the right WiFi

# Real safe joint limits (must match kinematics.py / esp32_standalone_firmware.ino)
JOINT_LIMITS = {
    1: (15, 170, "Joint 1 - Base"),
    2: (45, 90, "Joint 2 - Shoulder"),
    3: (25, 100, "Joint 3 - Elbow"),
    4: (0, 80, "Gripper (0=closed/HOME, 80=open)"),
}

# Real HOME position (must match main firmware's SAFE_SHOULDER/SAFE_ELBOW/JOINT1_MIN)
HOME_POSITION = {
    1: 15,
    2: 90,
    3: 45,
    4: 0,
}


class ServoTestGUI_WiFi:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm - Joint Test GUI (WiFi)")
        self.connected = False
        self.pending = {}   # latest wanted angle per joint, while dragging
        self.sending = set()  # joint ids that currently have a sender thread running

        # ---- Connection row ----
        conn_frame = ttk.Frame(root, padding=10)
        conn_frame.pack(fill="x")

        ttk.Label(conn_frame, text=f"ESP32 address: {BASE_URL}").pack(side="left")
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side="left", padx=10)

        self.status_label = ttk.Label(conn_frame, text="Not connected", foreground="red")
        self.status_label.pack(side="left", padx=10)

        # ---- Sliders ----
        self.sliders = {}
        self.value_labels = {}

        for joint_id, (min_v, max_v, name) in JOINT_LIMITS.items():
            frame = ttk.LabelFrame(root, text=f"{name}  (limit: {min_v} - {max_v} deg)", padding=10)
            frame.pack(fill="x", padx=10, pady=5)

            slider = ttk.Scale(
                frame, from_=min_v, to=max_v, orient="horizontal",
                command=lambda val, jid=joint_id: self.on_slider_move(jid, val)
            )
            home_v = HOME_POSITION[joint_id]
            slider.set(home_v)
            slider.pack(fill="x")
            self.sliders[joint_id] = slider

            val_label = ttk.Label(frame, text=f"{home_v} deg")
            val_label.pack()
            self.value_labels[joint_id] = val_label

        # ---- Home button ----
        home_btn = tk.Button(
            root, text="HOME (safe position)", bg="red", fg="white",
            font=("Arial", 12, "bold"), command=self.go_home
        )
        home_btn.pack(fill="x", padx=10, pady=10)

        # ---- Pick and Place ----
        pp_frame = ttk.LabelFrame(root, text="Pick & Place", padding=10)
        pp_frame.pack(fill="x", padx=10, pady=5)

        self.pp_entries = {}
        pick_row = ttk.Frame(pp_frame)
        pick_row.pack(fill="x", pady=3)
        ttk.Label(pick_row, text="Pick (x,y,z):", width=14).pack(side="left")
        for key in ("px", "py", "pz"):
            e = ttk.Entry(pick_row, width=6)
            e.insert(0, "10")
            e.pack(side="left", padx=3)
            self.pp_entries[key] = e

        place_row = ttk.Frame(pp_frame)
        place_row.pack(fill="x", pady=3)
        ttk.Label(place_row, text="Place (x,y,z):", width=14).pack(side="left")
        for key in ("lx", "ly", "lz"):
            e = ttk.Entry(place_row, width=6)
            e.insert(0, "10")
            e.pack(side="left", padx=3)
            self.pp_entries[key] = e

        self.pp_status = ttk.Label(pp_frame, text="", foreground="#555555")
        self.pp_status.pack(fill="x", pady=(5, 0))

        self.pp_button = tk.Button(
            pp_frame, text="Run Pick & Place", bg="#2563eb", fg="white",
            font=("Arial", 11, "bold"), command=self.run_pick_place
        )
        self.pp_button.pack(fill="x", pady=(5, 0))

    def run_pick_place(self):
        if not self.connected:
            messagebox.showerror("Not connected", "Connect to the robot first.")
            return

        try:
            values = {k: float(e.get()) for k, e in self.pp_entries.items()}
        except ValueError:
            messagebox.showerror("Invalid input", "All 6 fields must be numbers.")
            return

        self.pp_button.config(state="disabled")
        self.pp_status.config(text="Running... (takes about 20-30 seconds)")

        def do_pickplace():
            try:
                resp = requests.get(f"{BASE_URL}/pickplace", params=values, timeout=40)
                message = resp.text
            except requests.exceptions.RequestException as e:
                message = f"Request failed: {e}"
            self.root.after(0, lambda: self._on_pickplace_done(message))

        threading.Thread(target=do_pickplace, daemon=True).start()

    def _on_pickplace_done(self, message):
        self.pp_status.config(text=message)
        self.pp_button.config(state="normal")
        # Refresh sliders to show HOME, since pick-and-place ends there
        for joint_id, home_v in HOME_POSITION.items():
            self.sliders[joint_id].set(home_v)
            self.value_labels[joint_id].config(text=f"{home_v} deg")

    # -------------------------------------------------------------
    def toggle_connect(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        self.status_label.config(text="Connecting...", foreground="orange")
        self.root.update()

        def do_connect():
            try:
                # /state is a lightweight endpoint - good for a quick "is it there?" check
                resp = requests.get(f"{BASE_URL}/state", timeout=REQUEST_TIMEOUT)
                if resp.status_code == 200:
                    self.connected = True
                    self.root.after(0, lambda: self._on_connected(resp.json()))
                else:
                    self.root.after(0, lambda: self._on_connect_failed(f"Unexpected response: {resp.status_code}"))
            except requests.exceptions.RequestException as e:
                self.root.after(0, lambda: self._on_connect_failed(str(e)))

        threading.Thread(target=do_connect, daemon=True).start()

    def _on_connected(self, state):
        self.status_label.config(text="Connected", foreground="green")
        self.connect_btn.config(text="Disconnect")
        # Sync sliders to the real current position, same idea as the USB version
        try:
            for joint_id_str, angle in state.items():
                joint_id = int(joint_id_str)
                if joint_id in self.sliders:
                    self.sliders[joint_id].set(angle)
                    self.value_labels[joint_id].config(text=f"{angle} deg")
        except (ValueError, KeyError):
            pass  # if the state format is unexpected, sliders just keep their current values

    def _on_connect_failed(self, error_msg):
        self.connected = False
        self.status_label.config(text="Not connected", foreground="red")
        messagebox.showerror(
            "Connection failed",
            f"Could not reach the robot at {BASE_URL}.\n\n"
            f"Check that your laptop's WiFi is connected to 'RoboticArm', "
            f"and that the ESP32 is powered on.\n\nDetails: {error_msg}"
        )

    def disconnect(self):
        self.connected = False
        self.status_label.config(text="Not connected", foreground="red")
        self.connect_btn.config(text="Connect")

    # -------------------------------------------------------------
    def on_slider_move(self, joint_id, val):
        angle = int(float(val))
        min_v, max_v, _ = JOINT_LIMITS[joint_id]
        angle = max(min_v, min(max_v, angle))  # extra safety clamp on PC side too

        self.value_labels[joint_id].config(text=f"{angle} deg")
        self.pending[joint_id] = angle  # remember the latest wanted value

        # Only start a new sender thread if this joint doesn't already
        # have one running - avoids flooding the ESP32 with a burst of
        # near-duplicate requests while the slider is being dragged.
        if joint_id not in self.sending:
            self.sending.add(joint_id)
            threading.Thread(target=self._sender_loop, args=(joint_id,), daemon=True).start()

    def _sender_loop(self, joint_id):
        # Keeps sending the LATEST pending value for this joint until it
        # matches what was last sent - naturally catches up to wherever
        # the slider ends up, without ever having more than one request
        # for this joint in flight at once.
        last_sent = None
        while True:
            angle = self.pending.get(joint_id)
            if angle == last_sent:
                break
            if not self.connected:
                break
            try:
                requests.get(
                    f"{BASE_URL}/set", params={"id": joint_id, "angle": angle},
                    timeout=REQUEST_TIMEOUT
                )
                last_sent = angle
            except requests.exceptions.RequestException:
                self.root.after(0, self._on_lost_connection)
                break
        self.sending.discard(joint_id)

    def _on_lost_connection(self):
        self.connected = False
        self.status_label.config(text="Lost connection", foreground="red")
        self.connect_btn.config(text="Connect")

    def go_home(self):
        for joint_id, home_v in HOME_POSITION.items():
            self.sliders[joint_id].set(home_v)
            self.value_labels[joint_id].config(text=f"{home_v} deg")

        if not self.connected:
            return

        def do_home():
            try:
                requests.get(f"{BASE_URL}/home", timeout=REQUEST_TIMEOUT)
            except requests.exceptions.RequestException:
                self.root.after(0, self._on_lost_connection)

        threading.Thread(target=do_home, daemon=True).start()


if __name__ == "__main__":
    root = tk.Tk()
    app = ServoTestGUI_WiFi(root)
    root.mainloop()
