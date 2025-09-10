# teensy_gui.py
# pip install pyserial
import tkinter as tk
from tkinter import ttk, messagebox
import serial, serial.tools.list_ports
import threading, time, sys

BAUD = 115200
DEFAULT_PORT = "COM21"
SEND_PERIOD_SEC = 0.05  # 20 Hz

class SerialIO:
    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self.reader_thread = None
        self.reader_running = False
        self.on_line = None  # callback(str)

    def open(self, port):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(port, BAUD, timeout=0.2)
        # start reader
        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def close(self):
        self.reader_running = False
        with self.lock:
            if self.ser:
                try:
                    if self.ser.is_open:
                        self._write_line("t:0.0 s:0.0")  # neutral
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None

    def _write_line(self, line: str):
        if self.ser and self.ser.is_open:
            self.ser.write((line + "\n").encode("ascii"))

    def send_norm(self, steer: float, throttle: float):
        # both in [-1..1]
        line = f"s:{steer:.3f} t:{throttle:.3f}"
        with self.lock:
            self._write_line(line)

    def send_cmd(self, cmd: str):
        with self.lock:
            self._write_line(cmd)

    def _reader_loop(self):
        buf = b""
        while self.reader_running:
            try:
                with self.lock:
                    if self.ser and self.ser.in_waiting:
                        buf += self.ser.read(self.ser.in_waiting)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    s = line.decode(errors="ignore").strip()
                    if self.on_line:
                        self.on_line(s)
            except Exception as e:
                # print to stderr but keep running
                print("Reader error:", e, file=sys.stderr)
            time.sleep(0.02)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Teensy RC Controller")
        self.geometry("520x360")
        self.resizable(False, False)

        self.io = SerialIO()
        self.io.on_line = self._on_serial_line

        # Top row: ports/connect
        top = ttk.Frame(self); top.pack(fill="x", padx=10, pady=8)
        ttk.Label(top, text="Port:").pack(side="left")
        self.cmb = ttk.Combobox(top, width=14, values=self._list_ports())
        self.cmb.set(DEFAULT_PORT if DEFAULT_PORT in self.cmb["values"] else (self.cmb["values"][0] if self.cmb["values"] else DEFAULT_PORT))
        self.cmb.pack(side="left", padx=6)
        ttk.Button(top, text="Refresh", command=self._refresh).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connect", command=self._toggle_conn)
        self.btn_conn.pack(side="right")

        # Controls
        ctl = ttk.LabelFrame(self, text="Control"); ctl.pack(fill="x", padx=10, pady=6)
        self.var_steer = tk.DoubleVar(value=0.0)
        self.sld = ttk.Scale(ctl, from_=-1.0, to=1.0, orient="horizontal",
                             variable=self.var_steer, command=lambda _ : None)
        self.sld.pack(fill="x", padx=8, pady=6)
        row = ttk.Frame(ctl); row.pack(fill="x", padx=8, pady=2)
        ttk.Label(row, text="Throttle (-1..1):").pack(side="left")
        self.var_thr = tk.DoubleVar(value=0.0)
        ttk.Entry(row, textvariable=self.var_thr, width=8).pack(side="left", padx=6)
        ttk.Button(row, text="Center steer", command=lambda: self.var_steer.set(0.0)).pack(side="left", padx=6)
        ttk.Button(row, text="Neutral", command=self._neutral).pack(side="left")

        # SCAN + pin setters
        row2 = ttk.Frame(ctl); row2.pack(fill="x", padx=8, pady=4)
        ttk.Button(row2, text="SCAN pins", command=lambda: self._send_cmd("SCAN")).pack(side="left")
        ttk.Label(row2, text="SET_STEER_PIN:").pack(side="left", padx=(10,2))
        self.var_sp = tk.IntVar(value=2)
        ttk.Entry(row2, textvariable=self.var_sp, width=5).pack(side="left")
        ttk.Button(row2, text="Set", command=lambda: self._send_cmd(f"SET_STEER_PIN:{self.var_sp.get()}")).pack(side="left", padx=6)

        ttk.Label(row2, text="SET_ESC_PIN:").pack(side="left", padx=(12,2))
        self.var_ep = tk.IntVar(value=3)
        ttk.Entry(row2, textvariable=self.var_ep, width=5).pack(side="left")
        ttk.Button(row2, text="Set", command=lambda: self._send_cmd(f"SET_ESC_PIN:{self.var_ep.get()}")).pack(side="left", padx=6)

        # Log box
        logf = ttk.LabelFrame(self, text="Log (from Teensy)"); logf.pack(fill="both", expand=True, padx=10, pady=6)
        self.txt = tk.Text(logf, height=10)
        self.txt.pack(fill="both", expand=True, padx=6, pady=6)

        # background sender
        self.last_send = 0.0
        self.after(int(SEND_PERIOD_SEC*1000), self._tick)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # Utils
    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh(self):
        ports = self._list_ports()
        self.cmb["values"] = ports
        if ports and self.cmb.get() not in ports:
            self.cmb.set(ports[0])

    def _toggle_conn(self):
        if self.io.ser and self.io.ser.is_open:
            self.io.close()
            self.btn_conn.config(text="Connect")
            self.title("Teensy RC Controller")
        else:
            try:
                self.io.open(self.cmb.get().strip())
                self.btn_conn.config(text="Disconnect")
                self.title(f"Teensy RC Controller — {self.cmb.get().strip()}")
                self._neutral()
            except Exception as e:
                messagebox.showerror("Connection error", f"{e}")

    def _tick(self):
        # send current controls at fixed rate
        try:
            if self.io.ser and self.io.ser.is_open:
                steer = float(self.var_steer.get())
                try:
                    thr = float(self.var_thr.get())
                except:
                    thr = 0.0
                self.io.send_norm(steer, thr)
        except Exception as e:
            print("Send error:", e, file=sys.stderr)
        self.after(int(SEND_PERIOD_SEC*1000), self._tick)

    def _send_cmd(self, cmd: str):
        if self.io.ser and self.io.ser.is_open:
            self.io.send_cmd(cmd)
        else:
            messagebox.showwarning("Not connected", "Open a COM port first.")

    def _neutral(self):
        self.var_steer.set(0.0)
        self.var_thr.set(0.0)
        self._send_cmd("s:0 t:0")

    def _on_serial_line(self, s: str):
        # append to log textbox
        self.txt.insert("end", s + "\n")
        self.txt.see("end")

    def _on_close(self):
        try:
            self._neutral()
        except Exception:
            pass
        self.io.close()
        self.destroy()

if __name__ == "__main__":
    App().mainloop()
