"""GUI described in thesis §3.1: left = live list of detected objects;
right = pick / drop buttons, command entry + Start Mission, Update Background.
"""

import queue
import threading
import tkinter as tk
from tkinter import ttk

import cv2

from .world import build_world_state


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
        self.btn_home = ttk.Button(right, text="Home", command=lambda: self._run(self.sys.arm.home))
        self.btn_home.grid(row=8, column=0, sticky="ew")
        self.log_box = tk.Text(right, width=60, height=16, state="disabled")
        self.log_box.grid(row=9, column=0, sticky="nsew", pady=6)
        right.rowconfigure(9, weight=1)

        self.sys.start_vision()
        self.root.after(300, self._tick)

    # ---------------- helpers
    def _log(self, msg):
        self.msgs.put(str(msg))

    def _set_busy(self, busy):
        self.busy = busy
        for b in (self.btn_pick, self.btn_drop, self.btn_start, self.btn_bg, self.btn_home):
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
        sel = self.objects.get(self.objects.curselection()[0]).split()[0] if self.objects.curselection() else None
        self.objects.delete(0, "end")
        ws = build_world_state(mem, self.sys.grid, self.sys.cfg["vision"]["cube_size_m"]) if mem else None
        for oid in sorted(mem):
            on = ws.blocks[oid].support if ws else "?"
            self.objects.insert("end", f"{oid}  lvl {mem[oid]['level']}  on {on}")
            if oid == sel:
                self.objects.selection_set("end")
        self.target["values"] = sorted(mem) + self.sys.grid.cells()
        if self.preview:
            img = self.sys.perception.overlay()
            if img is not None:
                cv2.imshow("RealSense", img)
                cv2.waitKey(1)
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

    def pick(self):
        if not self.objects.curselection():
            self._log("Select an object first")
            return
        oid = self.objects.get(self.objects.curselection()[0]).split()[0]
        self.ws = build_world_state(self._snapshot(), self.sys.grid, self.sys.cfg["vision"]["cube_size_m"])
        if not self.ws.clear(oid):
            self._log(f"{oid} has something on top")
            return

        def do():
            self.sys.executor.pick(self.ws, oid)
            self.holding = oid
            self._log(f"Holding {oid}")

        self._run(do)

    def drop(self):
        tgt = self.target.get().strip().upper()
        if not self.holding or not tgt:
            self._log("Pick an object and choose a target first")
            return

        def do():
            self.sys.executor.drop(self.ws, tgt)
            self.ws.apply_move(self.holding, tgt, self.sys.grid)
            self._log(f"Dropped {self.holding} on {tgt}")
            self.holding = None

        self._run(do)

    def quit(self):
        self.sys.shutdown()
        if self.preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()
