"""GUI described in thesis §3.1: left = live list of detected objects;
centre = pick / drop buttons, command entry + Start Mission, Update Background;
right = live camera view with the detections (embedded instead of a separate OpenCV window).
"""

import base64
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import cv2

from .world import build_world_state

PREVIEW_WIDTH = 560  # px; the 640x480 frame is scaled down to fit next to the controls


def frame_to_photo(img, width=PREVIEW_WIDTH):
    """BGR image -> tk.PhotoImage (PNG round-trip: Tk reads PNG natively, no Pillow needed)."""
    h, w = img.shape[:2]
    if w != width:
        img = cv2.resize(img, (width, round(h * width / w)), interpolation=cv2.INTER_AREA)
    ok, png = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    if not ok:
        return None
    return tk.PhotoImage(data=base64.b64encode(png.tobytes()).decode("ascii"), format="png")


class App:
    def __init__(self, system, preview=True):
        self.sys = system
        self.preview = preview
        self.msgs = queue.Queue()
        self.sys.log = self.sys.executor.log = self.sys.mission.log = self._log
        self.ws = None  # world state used by manual pick/drop
        self.holding = None
        self.busy = False

        self.root = tk.Tk()
        self.root.title("Braccio++ neuro-symbolic control")
        self.root.protocol("WM_DELETE_WINDOW", self.quit)

        left = ttk.Frame(self.root, padding=8)
        left.grid(row=0, column=0, sticky="ns")
        ttk.Label(left, text="Detected objects").pack(anchor="w")
        self.objects = tk.Listbox(left, width=42, height=18, exportselection=False)
        self.objects.pack(fill="y", expand=True)
        self.objects.bind("<Double-Button-1>", lambda e: self.pick())
        self.objects.bind("<Return>", lambda e: self.pick())
        ttk.Label(left, text="Double-click (or Enter) to pick it with the gripper").pack(anchor="w")
        self._lines = []

        right = ttk.Frame(self.root, padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        self.btn_pick = ttk.Button(right, text="Pick selected", command=self.pick)
        self.btn_pick.grid(row=0, column=0, sticky="ew")
        self.target = ttk.Combobox(right, width=22, values=self.sys.grid.cells())
        self.target.grid(row=1, column=0, sticky="ew", pady=2)
        self.btn_drop = ttk.Button(right, text="Drop on target", command=self.drop)
        self.btn_drop.grid(row=2, column=0, sticky="ew")
        ttk.Separator(right).grid(row=3, column=0, sticky="ew", pady=6)
        ttk.Label(right, text="Command").grid(row=4, column=0, sticky="w")
        self.cmd = ttk.Entry(right, width=50)
        self.cmd.grid(row=5, column=0, sticky="ew")
        self.cmd.bind("<Return>", lambda e: self.start_mission())
        self.btn_start = ttk.Button(right, text="Start Mission", command=self.start_mission)
        self.btn_start.grid(row=6, column=0, sticky="ew", pady=2)
        self.btn_bg = ttk.Button(right, text="Update Background", command=self.update_background)
        self.btn_bg.grid(row=7, column=0, sticky="ew", pady=2)
        self.btn_home = ttk.Button(right, text="Home", command=self.home)
        self.btn_home.grid(row=8, column=0, sticky="ew")
        self.btn_cal = ttk.Button(right, text="Calibrate", command=self.calibrate)
        self.btn_cal.grid(row=9, column=0, sticky="ew", pady=2)
        self.log_box = tk.Text(right, width=60, height=16, state="disabled")
        self.log_box.grid(row=10, column=0, sticky="nsew", pady=6)
        right.rowconfigure(10, weight=1)

        self.video, self._photo = None, None
        if self.preview:
            cam = ttk.Frame(self.root, padding=8)
            cam.grid(row=0, column=2, sticky="n")
            ttk.Label(cam, text="Camera").pack(anchor="w")
            self.video = tk.Label(cam, text="Waiting for the camera...", background="black", foreground="white")
            self.video.pack()
            self.show_grid = tk.BooleanVar(value=True)
            ttk.Checkbutton(cam, text="Show grid", variable=self.show_grid).pack(anchor="w")
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.sys.start_vision()
        self.root.after(300, self._tick)

    # ---------------- helpers
    def _log(self, msg):
        self.msgs.put(str(msg))

    def _set_busy(self, busy):
        self.busy = busy
        for b in (self.btn_pick, self.btn_drop, self.btn_start, self.btn_bg, self.btn_home, self.btn_cal):
            b.state(["disabled"] if busy else ["!disabled"])

    def _run(self, fn, *args):
        if self.busy:
            return
        self._set_busy(True)

        def work():
            try:
                fn(*args)
            except Exception as e:
                self._log(f"ERROR: {type(e).__name__}: {e}")
            finally:
                self.root.after(0, lambda: self._set_busy(False))

        threading.Thread(target=work, daemon=True).start()

    def _snapshot(self):
        return self.sys.perception.target_memory()

    def _tick(self):
        while not self.msgs.empty():
            self.log_box.configure(state="normal")
            self.log_box.insert("end", self.msgs.get() + "\n")
            self.log_box.see("end")
            self.log_box.configure(state="disabled")
        mem = self._snapshot()
        ws = build_world_state(mem, self.sys.grid, self.sys.cfg["vision"]["cube_size_m"]) if mem else None
        lines = [f"{oid}  lvl {mem[oid]['level']}  on {ws.blocks[oid].support if ws else '?'}" for oid in sorted(mem)]
        if lines != self._lines:  # rebuild only on change, so clicks and scrolling are not lost
            sel = self._selected()
            self.objects.delete(0, "end")
            for i, line in enumerate(lines):
                self.objects.insert("end", line)
                if line.split()[0] == sel:
                    self.objects.selection_set(i)
                    self.objects.activate(i)
            self._lines = lines
        self.target["values"] = sorted(mem) + self.sys.grid.cells()
        if self.video is not None:
            img = self.sys.perception.overlay(show_grid=self.show_grid.get())
            photo = frame_to_photo(img) if img is not None else None
            if photo is not None:
                self._photo = photo  # keep a reference, or Tk drops the image
                self.video.configure(image=photo, text="")
        self.root.after(100, self._tick)

    # ---------------- actions
    def update_background(self):
        self._log("Capturing background: keep the workspace empty...")
        self._run(self.sys.capture_background)

    def start_mission(self):
        text = self.cmd.get().strip()
        if not text:
            return
        self._log(f"> {text}")
        self._run(lambda: self._log(self.sys.run_mission(text)))

    def calibrate(self):
        if self.busy:
            return
        if self.holding:
            self._log(f"Drop {self.holding} before calibrating")
            return
        n = self.sys.cfg["calibration"]["n_points"]
        if not self.sys.sim and not messagebox.askokcancel(
            "Calibrate",
            f"The arm will visit {n} poses over the grid.\n\n"
            "Attach the ArUco marker to the gripper and clear the workspace.\n\nStart the calibration?",
            parent=self.root,
        ):
            return
        self._log("Calibrating: do not touch the arm or the camera...")

        def do():
            tf = self.sys.calibrate()
            self._log(f"Calibration applied: RMSE {tf.rmse * 1000:.1f} mm")

        self._run(do)

    def home(self):
        if self.holding:  # firmware HOME would loosen the gripper and drop the block
            self._log(f"Home keeping {self.holding} in the gripper")
            self._run(self.sys.executor.home_holding)
        else:
            self._run(self.sys.executor.home_open)

    def _selected(self):
        cur = self.objects.curselection()
        return self.objects.get(cur[0]).split()[0] if cur else None

    def pick(self):
        if self.busy:
            return
        oid = self._selected()
        if oid is None:
            self._log("Select an object first")
            return
        if self.holding:
            self._log(f"Already holding {self.holding}: drop it first")
            return
        self.ws = build_world_state(self._snapshot(), self.sys.grid, self.sys.cfg["vision"]["cube_size_m"])
        if oid not in self.ws.blocks:
            self._log(f"{oid} is no longer detected")
            return
        if not self.ws.clear(oid):
            self._log(f"{oid} has something on top")
            return
        self._log(f"Picking {oid}...")

        def do():
            self.sys.executor.pick(self.ws, oid)
            self.holding = oid
            self._log(f"Holding {oid}: choose a target and press Drop on target")
            self.root.after(0, self.target.focus_set)

        self._run(do)

    def drop(self):
        tgt = self.target.get().strip().upper()
        if not self.holding or not tgt:
            self._log("Pick an object and choose a target first")
            return

        def do():
            self.sys.executor.drop(self.ws, tgt)
            self.ws.apply_move(self.holding, tgt, self.sys.grid)
            self.sys.perception.sync(self.ws)  # keep the block IDs across the manual move
            self._log(f"Dropped {self.holding} on {tgt}")
            self.holding = None

        self._run(do)

    def quit(self):
        self.sys.shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
