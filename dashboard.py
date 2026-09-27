# -*- coding: utf-8 -*-
"""
Tkinter GUI Dashboard Module for RoboMaster SLAM.
Provides real-time visualization of the grid map, detected foam walls,
robot pose, gimbal scanning direction, and telemetry control panels.
"""

import threading
import tkinter as tk

from config import DELTA, DIRECTIONS, GRID_H, GRID_W, SYMBOL, in_bounds
import state


class Dashboard:
    def __init__(self, on_start=None, on_stop=None, sim_mode=False, default_start=(1, 1, "NORTH")):
        self.root = tk.Tk()
        self.root.title("RoboMaster SLAM Explore - Dashboard")
        self.root.geometry("1060x750")
        self.root.configure(bg="#07111f")
        self.closed = False
        self.is_running = False
        self.on_start = on_start
        self.on_stop = on_stop
        self.sim_mode = sim_mode

        self.cell_size = 115
        self.pad = 45

        # Variables for start configuration
        self.start_x_var = tk.IntVar(value=default_start[0])
        self.start_y_var = tk.IntVar(value=default_start[1])
        self.start_heading_var = tk.StringVar(value=default_start[2])

        self.start_x_var.trace_add("write", lambda *args: self.redraw_preview())
        self.start_y_var.trace_add("write", lambda *args: self.redraw_preview())
        self.start_heading_var.trace_add("write", lambda *args: self.redraw_preview())

        # Main Layout: Left Canvas, Right Control & Status Panel
        self.canvas = tk.Canvas(
            self.root,
            width=self.pad * 2 + self.cell_size * GRID_W,
            height=self.pad * 2 + self.cell_size * GRID_H,
            bg="#020617",
            highlightthickness=1,
            highlightbackground="#1d4e73",
            cursor="hand2",
        )
        self.canvas.pack(side=tk.LEFT, padx=16, pady=16)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

        panel = tk.Frame(self.root, bg="#0d1b2a", padx=16, pady=16)
        panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(0, 16), pady=16)

        # Title
        title_text = "SLAM EXPLORER (SIM)" if sim_mode else "SLAM EXPLORER (ROBOT)"
        tk.Label(
            panel,
            text=title_text,
            bg="#0d1b2a",
            fg="#7df9ff",
            font=("Segoe UI", 16, "bold"),
        ).pack(anchor="w", pady=(0, 10))

        # Start Configuration Box
        self.config_frame = tk.LabelFrame(
            panel,
            text=" [ ระบุจุดเริ่มต้น (Start Configuration) ] ",
            bg="#0d1b2a",
            fg="#00e5ff",
            font=("Segoe UI", 10, "bold"),
            padx=10,
            pady=8,
            highlightbackground="#1d4e73",
            highlightthickness=1,
        )
        self.config_frame.pack(fill=tk.X, pady=(0, 10))

        coord_row = tk.Frame(self.config_frame, bg="#0d1b2a")
        coord_row.pack(fill=tk.X, pady=3)

        tk.Label(coord_row, text="Start X:", bg="#0d1b2a", fg="#d9f3ff", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        self.spin_x = tk.Spinbox(
            coord_row,
            from_=1,
            to=GRID_W,
            textvariable=self.start_x_var,
            width=4,
            bg="#07111f",
            fg="#7df9ff",
            font=("Segoe UI", 9, "bold"),
            buttonbackground="#1d4e73",
            justify=tk.CENTER,
        )
        self.spin_x.pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(coord_row, text="Start Y:", bg="#0d1b2a", fg="#d9f3ff", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        self.spin_y = tk.Spinbox(
            coord_row,
            from_=1,
            to=GRID_H,
            textvariable=self.start_y_var,
            width=4,
            bg="#07111f",
            fg="#7df9ff",
            font=("Segoe UI", 9, "bold"),
            buttonbackground="#1d4e73",
            justify=tk.CENTER,
        )
        self.spin_y.pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(coord_row, text="ทิศทาง:", bg="#0d1b2a", fg="#d9f3ff", font=("Segoe UI", 9, "bold")).pack(
            side=tk.LEFT, padx=(0, 4)
        )
        self.heading_menu = tk.OptionMenu(coord_row, self.start_heading_var, *DIRECTIONS)
        self.heading_menu.config(
            bg="#07111f",
            fg="#7df9ff",
            font=("Segoe UI", 8, "bold"),
            activebackground="#1d4e73",
            activeforeground="#ffffff",
            highlightthickness=0,
        )
        self.heading_menu["menu"].config(bg="#07111f", fg="#7df9ff")
        self.heading_menu.pack(side=tk.LEFT)

        tk.Label(
            self.config_frame,
            text="💡 คลิกบนตาราง Grid เพื่อเลือกพิกัด Start ได้โดยตรง",
            bg="#0d1b2a",
            fg="#8eb9d6",
            font=("Segoe UI", 8),
        ).pack(anchor="w", pady=(4, 0))

        # Buttons Frame
        btn_frame = tk.Frame(panel, bg="#0d1b2a")
        btn_frame.pack(fill=tk.X, pady=(0, 10))

        self.btn_start = tk.Button(
            btn_frame,
            text="▶ เริ่มสำรวจ (Start)",
            bg="#0284c7",
            fg="#ffffff",
            font=("Segoe UI", 11, "bold"),
            activebackground="#0369a1",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.handle_start_click,
        )
        self.btn_start.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))

        self.btn_stop = tk.Button(
            btn_frame,
            text="⏹ หยุด (Stop)",
            bg="#ef4444",
            fg="#ffffff",
            font=("Segoe UI", 11, "bold"),
            activebackground="#b91c1c",
            activeforeground="#ffffff",
            relief=tk.FLAT,
            padx=12,
            pady=6,
            cursor="hand2",
            state=tk.DISABLED,
            command=self.handle_stop_click,
        )
        self.btn_stop.pack(side=tk.RIGHT, padx=(6, 0))

        # Status Label
        self.status = tk.Label(
            panel,
            justify=tk.LEFT,
            anchor="nw",
            bg="#07111f",
            fg="#d9f3ff",
            font=("Consolas", 10),
            padx=10,
            pady=8,
            highlightbackground="#1d4e73",
            highlightthickness=1,
        )
        self.status.pack(fill=tk.X, pady=(0, 8))

        # Log Text Box
        self.logs = tk.Text(
            panel,
            height=12,
            bg="#07111f",
            fg="#8eb9d6",
            font=("Consolas", 9),
            bd=0,
            padx=6,
            pady=6,
        )
        self.logs.pack(fill=tk.BOTH, expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.close)

        # Initial render of grid and start preview
        self.redraw_preview()
        self.log("ยินดีต้อนรับ! แผนที่รองรับกำแพงโฟม (Foam Walls) ตามขอบช่อง")
        self.log("ระบบจะหมุนเฉพาะ Gimbal ในการสแกน และขยับตัวหุ่นเฉพาะตอนเดิน")

    def on_canvas_click(self, event):
        if self.is_running:
            return
        gx = int((event.x - self.pad) // self.cell_size) + 1
        gy = GRID_H - int((event.y - self.pad) // self.cell_size)
        if in_bounds((gx, gy)):
            self.start_x_var.set(gx)
            self.start_y_var.set(gy)
            self.log(f"เลือกจุดเริ่มต้น: ({gx}, {gy})")

    def handle_start_click(self):
        if self.is_running:
            return
        sx = self.start_x_var.get()
        sy = self.start_y_var.get()
        if not in_bounds((sx, sy)):
            self.log(f"พิกัด ({sx}, {sy}) ไม่อยู่ในตาราง!")
            return
        self.is_running = True
        self.btn_start.config(state=tk.DISABLED, bg="#334155")
        self.btn_stop.config(state=tk.NORMAL)
        self.spin_x.config(state=tk.DISABLED)
        self.spin_y.config(state=tk.DISABLED)
        self.heading_menu.config(state=tk.DISABLED)
        self.canvas.config(cursor="arrow")

        start_config = (sx, sy, self.start_heading_var.get())
        if self.on_start:
            threading.Thread(target=self.on_start, args=(start_config,), daemon=True).start()

    def handle_stop_click(self):
        state.stop_requested = True
        self.log("⚠️ ผู้ใช้กดปุ่มหยุดภารกิจฉุกเฉิน")
        if self.on_stop:
            self.on_stop()

    def close(self):
        state.stop_requested = True
        self.closed = True
        try:
            self.root.destroy()
        except Exception:
            pass

    def log(self, message):
        if not self.closed:
            def _append():
                if not self.closed:
                    self.logs.insert(tk.END, message + "\n")
                    self.logs.see(tk.END)
            self.root.after(0, _append)

    def redraw_preview(self):
        if self.is_running:
            return
        try:
            sx = self.start_x_var.get()
            sy = self.start_y_var.get()
            heading = self.start_heading_var.get()
            if in_bounds((sx, sy)):
                self.update((sx, sy), heading, 0, "รอเริ่มสำรวจ (กด 'เริ่มสำรวจ')", extra_readings=None)
        except Exception:
            pass

    def update(self, position, heading, step, status, extra_readings=None, gimbal_dir=None):
        if self.closed:
            return

        def _do_update():
            if self.closed:
                return
            self.canvas.delete("all")

            # 1. Draw cell tiles
            for y in range(1, GRID_H + 1):
                for x in range(1, GRID_W + 1):
                    cx = self.pad + (x - 1) * self.cell_size
                    cy = self.pad + (GRID_H - y) * self.cell_size
                    cell_coord = (x, y)

                    if cell_coord in state.visited_cells:
                        fill_color = "#064e3b"   # Visited emerald green
                        label = "VISITED"
                        label_color = "#a7f3d0"
                    elif cell_coord in state.discovered_cells:
                        fill_color = "#0f3557"   # Discovered open passage
                        label = "OPEN"
                        label_color = "#38bdf8"
                    else:
                        fill_color = "#07111f"   # Unknown
                        label = "?"
                        label_color = "#475569"

                    self.canvas.create_rectangle(
                        cx, cy, cx + self.cell_size, cy + self.cell_size,
                        fill=fill_color, outline="#1e293b", width=1, dash=(2, 4)
                    )
                    self.canvas.create_text(
                        cx + self.cell_size / 2, cy + 22, text=f"({x},{y})",
                        fill="#d9f3ff", font=("Segoe UI", 9, "bold")
                    )
                    self.canvas.create_text(
                        cx + self.cell_size / 2, cy + 60, text=label,
                        fill=label_color, font=("Segoe UI", 10, "bold")
                    )

            # 2. Draw foam wall lines
            wall_color = "#ef476f" # Vibrant foam wall coral-red
            wall_glow = "#881337"

            # Horizontal foam walls: between (x, y) and (x, y+1)
            for (wx, wy) in state.detected_h_walls:
                lx1 = self.pad + (wx - 1) * self.cell_size
                lx2 = lx1 + self.cell_size
                ly = self.pad + (GRID_H - wy) * self.cell_size
                self.canvas.create_line(lx1, ly, lx2, ly, fill=wall_glow, width=8, capstyle=tk.ROUND)
                self.canvas.create_line(lx1, ly, lx2, ly, fill=wall_color, width=5, capstyle=tk.ROUND)

            # Vertical foam walls: between (x, y) and (x+1, y)
            for (wx, wy) in state.detected_v_walls:
                lx = self.pad + wx * self.cell_size
                ly1 = self.pad + (GRID_H - wy) * self.cell_size
                ly2 = ly1 + self.cell_size
                self.canvas.create_line(lx, ly1, lx, ly2, fill=wall_glow, width=8, capstyle=tk.ROUND)
                self.canvas.create_line(lx, ly1, lx, ly2, fill=wall_color, width=5, capstyle=tk.ROUND)

            # 3. Draw trajectory path
            for item in state.trajectory:
                if item["previous"] is None:
                    continue
                previous = item["previous"]
                current = item["cell"]
                x1 = self.pad + (previous[0] - 0.5) * self.cell_size
                y1 = self.pad + (GRID_H - previous[1] + 0.5) * self.cell_size
                x2 = self.pad + (current[0] - 0.5) * self.cell_size
                y2 = self.pad + (GRID_H - current[1] + 0.5) * self.cell_size
                self.canvas.create_line(x1, y1, x2, y2, fill="#7df9ff", width=3)

            # 4. Draw robot chassis marker
            cx = self.pad + (position[0] - 0.5) * self.cell_size
            cy = self.pad + (GRID_H - position[1] + 0.5) * self.cell_size

            self.canvas.create_oval(
                cx - 26, cy - 26, cx + 26, cy + 26,
                fill="#0284c7", outline="#e0f2fe", width=3
            )
            self.canvas.create_text(
                cx, cy, text=SYMBOL.get(heading, "^"), fill="white",
                font=("Segoe UI", 20, "bold")
            )

            # 5. Draw active gimbal scanning beam if scanning
            if gimbal_dir:
                gdx, gdy = DELTA[gimbal_dir]
                beam_x = cx + gdx * 45
                beam_y = cy - gdy * 45
                self.canvas.create_line(cx, cy, beam_x, beam_y, fill="#facc15", width=3, arrow=tk.LAST)

            visited_cnt = len(state.visited_cells)
            total_cells = GRID_W * GRID_H
            wall_cnt = len(state.detected_h_walls) + len(state.detected_v_walls)

            readings_str = ""
            if extra_readings:
                readings_str = "\nToF: " + " ".join(f"{k[0]}:{v:.0f}" for k, v in extra_readings.items())

            self.status.config(text=(
                "สถานะ: " + status + "\n"
                f"รอบที่: {step} | พิกัด: {position} | ทิศหุ่น: {heading}\n"
                f"เดินสำรวจแล้ว: {visited_cnt}/{total_cells} ช่อง ({visited_cnt / total_cells * 100:.1f}%)\n"
                f"กำแพงโฟมที่ตรวจพบ: {wall_cnt} แนวกำแพง | ToF: {state.current_distance:.0f} mm" + readings_str
            ))

        self.root.after(0, _do_update)
