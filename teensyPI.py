# teensy_gui_drive.py
# pip install pyserial
import tkinter as tk
from tkinter import ttk, messagebox
import serial, serial.tools.list_ports
import threading, time, sys

BAUD = 115200
DEFAULT_PORT = "COM21"
SEND_PERIOD = 0.05     # 20 Hz
PRIME_SECONDS = 3      # חייב להיות אותו ערך כמו בקוד ה-Teensy

class SerialIO:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()

    def open(self, port):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, BAUD, timeout=0.2)

    def close(self):
        with self.lock:
            if self.ser:
                try:
                    if self.ser.is_open:
                        self._write("s:0 t:0")
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def send(self, steer, throttle):
        steer = max(-1.0, min(1.0, float(steer)))
        throttle = max(-1.0, min(1.0, float(throttle)))
        self._write(f"s:{steer:.3f} t:{throttle:.3f}")

    def prime(self):
        self._write("PRIME")

    def _write(self, line):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((line + "\n").encode("ascii"))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Teensy RC Controller — steer[-1..1], throttle[-1..1]")
        self.geometry("580x300")
        self.resizable(False, False)

        self.io = SerialIO()
        self.priming = False
        self.prime_end = 0.0

        # --- Top bar ---
        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="Port:").pack(side="left")
        self.cmb = ttk.Combobox(top, width=16, values=self._ports())
        if self.cmb["values"]:
            self.cmb.set(DEFAULT_PORT if DEFAULT_PORT in self.cmb["values"] else self.cmb["values"][0])
        else:
            self.cmb.set(DEFAULT_PORT)
        self.cmb.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle)
        self.btn_conn.pack(side="right")

        # --- Status ---
        stat = ttk.Frame(self); stat.pack(fill="x", padx=10, pady=(0,6))
        ttk.Label(stat, text="Status:").pack(side="left")
        self.var_status = tk.StringVar(value="Disconnected")
        ttk.Label(stat, textvariable=self.var_status).pack(side="left", padx=6)

        # --- Controls ---
        box = ttk.LabelFrame(self, text="Control"); box.pack(fill="x", padx=10, pady=6)

        ttk.Label(box, text="Steer (-1..1):").pack(anchor="w", padx=10)
        self.var_steer = tk.DoubleVar(value=0.0)
        self.sld_steer = ttk.Scale(box, from_=-1.0, to=1.0, orient="horizontal",
                                   variable=self.var_steer)
        self.sld_steer.pack(fill="x", padx=10, pady=4)

        row = ttk.Frame(box); row.pack(fill="x", padx=10, pady=4)
        ttk.Label(row, text="Throttle (-1..1):").pack(side="left")
        self.var_thr = tk.DoubleVar(value=0.0)
        self.sld_thr = ttk.Scale(row, from_=-1.0, to=1.0, orient="horizontal",
                                 variable=self.var_thr, state="disabled")
        self.sld_thr.pack(side="left", fill="x", expand=True, padx=8)
        ttk.Button(row, text="Neutral", command=self._neutral).pack(side="left")

        row2 = ttk.Frame(box); row2.pack(fill="x", padx=10, pady=4)
        ttk.Button(row2, text="Center steer", command=lambda: self.var_steer.set(0.0)).pack(side="left")
        ttk.Button(row2, text="PRIME (3s)", command=self._do_prime).pack(side="left", padx=6)

        # sender loop
        self.after(int(SEND_PERIOD*1000), self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- Ports / Connect ----
    def _ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh(self):
        ports = self._ports()
        self.cmb["values"] = ports
        if ports and self.cmb.get() not in ports:
            self.cmb.set(ports[0])

    def _toggle(self):
        if self.io.ser and self.io.ser.is_open:
            self.io.close()
            self.btn_conn.config(text="Connect")
            self.var_status.set("Disconnected")
            self._set_throttle_enabled(False)
        else:
            try:
                self.io.open(self.cmb.get().strip())
                self.btn_conn.config(text="Disconnect")
                self._neutral()
                self._start_local_priming()
            except Exception as e:
                messagebox.showerror("Connection error", str(e))

    # ---- Priming UX ----
    def _start_local_priming(self):
        self.priming = True
        self.prime_end = time.monotonic() + PRIME_SECONDS
        self._set_throttle_enabled(False)
        self._update_status()
        self.after(200, self._poll_priming_done)

    def _poll_priming_done(self):
        if not self.priming:
            return
        if time.monotonic() >= self.prime_end:
            self.priming = False
            self._set_throttle_enabled(True)
            self._update_status()
        else:
            self._update_status()
            self.after(200, self._poll_priming_done)

    def _do_prime(self):
        if self.io.ser and self.io.ser.is_open:
            self.io.prime()
            self._start_local_priming()

    def _update_status(self):
        if not (self.io.ser and self.io.ser.is_open):
            self.var_status.set("Disconnected")
            return
        if self.priming:
            remain = max(0, int(self.prime_end - time.monotonic()) + 1)
            self.var_status.set(f"Priming ESC… {remain}s")
        else:
            self.var_status.set("Ready")

    def _set_throttle_enabled(self, enable: bool):
        self.sld_thr.configure(state=("normal" if enable else "disabled"))
        if not enable:
            self.var_thr.set(0.0)

    # ---- Send loop & helpers ----
    def _tick(self):
        try:
            if self.io.ser and self.io.ser.is_open:
                steer = self.var_steer.get()
                thr = self.var_thr.get() if not self.priming else 0.0
                self.io.send(steer, thr)
        except Exception as e:
            print("Send error:", e, file=sys.stderr)
        self.after(int(SEND_PERIOD*1000), self._tick)

    def _neutral(self):
        self.var_steer.set(0.0)
        self.var_thr.set(0.0)
        if self.io.ser and self.io.ser.is_open:
            self.io.send(0.0, 0.0)

    def _on_close(self):
        try:
            self._neutral()
        except Exception:
            pass
        self.io.close()
        self.destroy()

if __name__ == "__main__":
    App().mainloop()
