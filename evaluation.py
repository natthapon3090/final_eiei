# -*- coding: utf-8 -*-
"""
Map Evaluation and Output Exporter Module.
Calculates map coverage and accuracy metrics against ground truth,
and exports CSV logs, text reports, and a high-resolution trajectory map plot.
"""

import csv
import os
from config import (
    GRID_H,
    GRID_SIZE_M,
    GRID_W,
    GROUND_TRUTH_H_WALLS,
    GROUND_TRUTH_V_WALLS,
    OUTPUT_DIR,
)
import state


def evaluate_map_accuracy(gt_h_walls=None, gt_v_walls=None):
    """
    Evaluate exploration metrics:
    - Coverage = (visited cells / total cells) * 100
    - Map Accuracy = (correct cells / total cells) * 100
    """
    if gt_h_walls is None:
        gt_h_walls = GROUND_TRUTH_H_WALLS
    if gt_v_walls is None:
        gt_v_walls = GROUND_TRUTH_V_WALLS

    total_cells = GRID_W * GRID_H
    coverage_pct = (len(state.visited_cells) / total_cells * 100.0) if total_cells else 0.0

    correct_cells = 0
    cell_details = {}

    for x in range(1, GRID_W + 1):
        for y in range(1, GRID_H + 1):
            det_north = (x, y) in state.detected_h_walls
            det_south = (x, y - 1) in state.detected_h_walls
            det_east = (x, y) in state.detected_v_walls
            det_west = (x - 1, y) in state.detected_v_walls

            gt_north = (x, y) in gt_h_walls
            gt_south = (x, y - 1) in gt_h_walls
            gt_east = (x, y) in gt_v_walls
            gt_west = (x - 1, y) in gt_v_walls

            is_correct = (
                det_north == gt_north
                and det_south == gt_south
                and det_east == gt_east
                and det_west == gt_west
            )
            if is_correct:
                correct_cells += 1
            cell_details[(x, y)] = {
                "correct": is_correct,
                "detected": {"N": det_north, "S": det_south, "E": det_east, "W": det_west},
                "ground_truth": {"N": gt_north, "S": gt_south, "E": gt_east, "W": gt_west},
            }

    map_accuracy_pct = (correct_cells / total_cells * 100.0) if total_cells else 0.0

    return {
        "coverage_pct": coverage_pct,
        "map_accuracy_pct": map_accuracy_pct,
        "correct_cells": correct_cells,
        "total_cells": total_cells,
        "cell_details": cell_details,
        "detected_walls": len(state.detected_h_walls) + len(state.detected_v_walls),
        "gt_walls": len(gt_h_walls) + len(gt_v_walls),
    }


def save_outputs():
    """Save trajectory CSV, evaluation report, walls CSV, and high-res SLAM plot outside code folder."""
    log_file = os.path.join(OUTPUT_DIR, "exploration_log.csv")
    report_file = os.path.join(OUTPUT_DIR, "map_evaluation_report.txt")
    walls_file = os.path.join(OUTPUT_DIR, "walls_data.csv")
    img_file = os.path.join(OUTPUT_DIR, "robot_trajectory.png")

    fields = ["step", "timestamp", "grid_x", "grid_y", "x_m", "y_m", "heading", "tof_mm"]
    with open(log_file, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{key: item[key] for key in fields} for item in state.trajectory])

    # Save formal evaluation report to text file
    eval_metrics = evaluate_map_accuracy()
    try:
        with open(report_file, "w", encoding="utf-8") as rep:
            rep.write("======================================================================\n")
            rep.write("📊 ผลการวัดความถูกต้องของแผนที่ (MAP EVALUATION REPORT)\n")
            rep.write("======================================================================\n")
            rep.write("• Coverage     = (จำนวน Cell ที่สำรวจแล้ว / จำนวน Cell ทั้งหมด) x 100\n")
            rep.write(
                f"               = ({len(state.visited_cells)} / {eval_metrics['total_cells']}) x 100 = {eval_metrics['coverage_pct']:.2f}%\n\n"
            )
            rep.write("• Map Accuracy = (จำนวน Cell ที่ทายถูก / จำนวน Cell ทั้งหมด) x 100\n")
            rep.write(
                f"               = ({eval_metrics['correct_cells']} / {eval_metrics['total_cells']}) x 100 = {eval_metrics['map_accuracy_pct']:.2f}%\n\n"
            )
            rep.write("• สรุปแนวกำแพงโฟม:\n")
            rep.write(f"  - ตรวจพบจริง : {eval_metrics['detected_walls']} แนวกำแพง\n")
            rep.write(f"  - Ground Truth : {eval_metrics['gt_walls']} แนวกำแพง\n")
            rep.write("======================================================================\n")
    except Exception as e:
        print(f"ไม่สามารถบันทึก evaluation report: {e}")

    # Export foam walls coordinates for external analysis (e.g., MATLAB)
    with open(walls_file, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["type", "x", "y"])
        for wx, wy in state.detected_h_walls:
            w.writerow(["H", wx, wy])
        for wx, wy in state.detected_v_walls:
            w.writerow(["V", wx, wy])

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
        from matplotlib.lines import Line2D

        # Luxury Black-Red-Blue Theme
        fig_bg = "#050811"       # Deep Obsidian Black
        ax_bg = "#080d1a"        # Dark Tactical Slate
        cell_visited_bg = "#0c1d36" # Luxury Royal Navy Glass
        cell_visited_edge = "#1e3a66"

        neon_red_glow = "#ff0055"   # Laser Crimson Red Outer Glow
        neon_red_core = "#ff2a5f"   # Laser Crimson Red Core
        neon_red_bright = "#ff7597" # Bright Highlights

        neon_blue_glow = "#0284c7"  # Electric Blue Glow
        neon_blue_mid = "#00b4d8"   # Cyber Azure Path
        neon_blue_core = "#38bdf8"  # Neon Cyan Highlights

        start_col = "#00f5d4"       # Glowing Emerald Cyan for Start
        end_col = "#ff1744"         # Radiant Ruby for End

        fig, ax = plt.subplots(figsize=(7.8, 10.6), facecolor=fig_bg)
        ax.set_facecolor(ax_bg)

        # 1. Draw Grid Cells with Luxury Dark Glass Styling
        for x in range(1, GRID_W + 1):
            for y in range(1, GRID_H + 1):
                is_vis = (x, y) in state.visited_cells
                is_disc = (x, y) in state.discovered_cells

                if is_vis:
                    cell_patch = patches.Rectangle(
                        (x - 1 + 0.03, y - 1 + 0.03), 0.94, 0.94,
                        facecolor=cell_visited_bg,
                        edgecolor=cell_visited_edge,
                        linewidth=1.2,
                        alpha=0.92,
                        zorder=2,
                    )
                    ax.add_patch(cell_patch)

                coord_color = "#38bdf8" if is_vis else ("#64748b" if is_disc else "#334155")
                badge_edge = "#13233c" if is_vis else "none"
                ax.text(
                    x - 1 + 0.12, y - 0.14, f"({x},{y})",
                    ha="left", va="top", color=coord_color,
                    fontsize=8, fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.18", facecolor="#060c18", edgecolor=badge_edge, linewidth=0.6, alpha=0.95),
                    zorder=15,
                )

                ax.plot(x - 0.5, y - 0.5, marker="+", color="#172640", markersize=5, zorder=2)

        # 2. Draw Detected Foam Border Walls
        for (wx, wy) in state.detected_h_walls:
            ax.plot([wx - 1, wx], [wy, wy], color=neon_red_glow, linewidth=8.0, alpha=0.22, solid_capstyle="round", zorder=6)
            ax.plot([wx - 1, wx], [wy, wy], color=neon_red_core, linewidth=4.5, alpha=0.65, solid_capstyle="round", zorder=7)
            ax.plot([wx - 1, wx], [wy, wy], color=neon_red_bright, linewidth=2.0, alpha=0.95, solid_capstyle="round", zorder=8)
            ax.plot([wx - 1, wx], [wy, wy], marker="o", color=neon_red_glow, markersize=4.0, zorder=9)

        for (wx, wy) in state.detected_v_walls:
            ax.plot([wx, wx], [wy - 1, wy], color=neon_red_glow, linewidth=8.0, alpha=0.22, solid_capstyle="round", zorder=6)
            ax.plot([wx, wx], [wy - 1, wy], color=neon_red_core, linewidth=4.5, alpha=0.65, solid_capstyle="round", zorder=7)
            ax.plot([wx, wx], [wy - 1, wy], color=neon_red_bright, linewidth=2.0, alpha=0.95, solid_capstyle="round", zorder=8)
            ax.plot([wx, wx], [wy - 1, wy], marker="o", color=neon_red_glow, markersize=4.0, zorder=9)

        # 3. Draw Robot Trajectory
        if state.trajectory:
            tx = [item["grid_x"] - 0.5 for item in state.trajectory]
            ty = [item["grid_y"] - 0.5 for item in state.trajectory]

            ax.plot(tx, ty, color=neon_blue_glow, linewidth=7.5, alpha=0.25, solid_capstyle="round", zorder=4)
            ax.plot(tx, ty, color=neon_blue_mid, linewidth=4.0, alpha=0.65, solid_capstyle="round", zorder=4)
            ax.plot(tx, ty, color=neon_blue_core, linewidth=2.0, alpha=0.95, solid_capstyle="round", zorder=5)

            ax.plot(
                tx, ty, marker="o", linestyle="None",
                markerfacecolor=ax_bg, markeredgecolor=neon_blue_core,
                markeredgewidth=1.5, markersize=5.5, zorder=5,
            )

            for i in range(len(tx) - 1):
                dx = tx[i + 1] - tx[i]
                dy = ty[i + 1] - ty[i]
                dist = (dx**2 + dy**2)**0.5
                if dist > 0.1:
                    mid_x = tx[i] + dx * 0.55
                    mid_y = ty[i] + dy * 0.55
                    ax.annotate(
                        "", xy=(mid_x + dx * 0.08, mid_y + dy * 0.08),
                        xytext=(mid_x - dx * 0.08, mid_y - dy * 0.08),
                        arrowprops=dict(
                            arrowstyle="->",
                            color="#38bdf8",
                            lw=1.6,
                            shrinkA=0, shrinkB=0,
                        ),
                        zorder=5,
                    )

            # Start Point Badge
            ax.plot(tx[0], ty[0], marker="o", color=start_col, markersize=18, alpha=0.28, zorder=10)
            ax.plot(tx[0], ty[0], marker="o", color=start_col, markersize=11, alpha=0.8, zorder=11)
            ax.plot(tx[0], ty[0], marker="o", color="#ffffff", markersize=5, zorder=12)
            ax.text(
                tx[0], ty[0] - 0.28, "START", color=start_col, fontsize=7.5,
                fontweight="heavy", ha="center", va="top",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#031f1f", edgecolor=start_col, linewidth=0.8, alpha=0.85),
                zorder=13,
            )

            # End Point Badge
            ax.plot(tx[-1], ty[-1], marker="o", color=end_col, markersize=20, alpha=0.32, zorder=10)
            ax.plot(tx[-1], ty[-1], marker="D", color=end_col, markersize=11, alpha=0.9, zorder=11)
            ax.plot(tx[-1], ty[-1], marker="*", color="#ffffff", markersize=6, zorder=12)
            ax.text(
                tx[-1], ty[-1] + 0.28, "END", color="#ff7597", fontsize=7.5,
                fontweight="heavy", ha="center", va="bottom",
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#26040c", edgecolor=end_col, linewidth=0.8, alpha=0.85),
                zorder=13,
            )

        # 4. Axes & Tick Formatting
        ax.set_xlim(-0.15, GRID_W + 0.15)
        ax.set_ylim(-0.15, GRID_H + 0.15)
        ax.set_xticks(range(GRID_W + 1))
        ax.set_yticks(range(GRID_H + 1))
        ax.set_xticklabels(range(1, GRID_W + 2), color="#94a3b8", fontsize=9, fontweight="bold")
        ax.set_yticklabels(range(1, GRID_H + 2), color="#94a3b8", fontsize=9, fontweight="bold")

        ax.tick_params(colors="#64748b", which="both", length=4, width=1.2)
        for spine in ax.spines.values():
            spine.set_edgecolor("#1e293b")
            spine.set_linewidth(1.4)

        ax.grid(True, linestyle=":", color="#142138", alpha=0.6, zorder=0)
        ax.set_xlabel("Grid Coordinates X (60cm / cell)", color="#64748b", fontsize=9, labelpad=8)
        ax.set_ylabel("Grid Coordinates Y (60cm / cell)", color="#64748b", fontsize=9, labelpad=8)

        # 5. Header HUD Card
        total_walls = len(state.detected_h_walls) + len(state.detected_v_walls)
        total_steps = len(state.trajectory) if state.trajectory else 0

        fig.text(0.5, 0.965, "ROBOMASTER AUTONOMOUS SLAM",
                 ha="center", va="top", color="#f8fafc", fontsize=14, fontweight="bold")
        fig.text(0.5, 0.940, "TACTICAL MAZE RECONSTRUCTION  •  CYBER RED & BLUE EDITION",
                 ha="center", va="top", color="#38bdf8", fontsize=8, fontweight="bold", alpha=0.9)

        stats_str = f"COVERAGE: {eval_metrics['coverage_pct']:.0f}%   |   ACCURACY: {eval_metrics['map_accuracy_pct']:.1f}%   |   STEPS: {total_steps}   |   WALLS: {total_walls}   |   STATUS: COMPLETED"
        fig.text(0.5, 0.912, stats_str,
                 ha="center", va="top", color="#94a3b8", fontsize=7.8, fontweight="bold",
                 bbox=dict(boxstyle="round,pad=0.35", facecolor="#091222", edgecolor="#1e3a66", linewidth=1.0, alpha=0.95))

        # 6. HUD Legend
        legend_elements = [
            Line2D([0], [0], color=neon_red_core, lw=3.0, label="Foam Wall (Red)"),
            Line2D([0], [0], color=neon_blue_core, lw=2.0, marker="o", markerfacecolor=ax_bg, markeredgecolor=neon_blue_core, markersize=4.5, label="Trajectory (Blue)"),
            patches.Patch(facecolor=cell_visited_bg, edgecolor=cell_visited_edge, label="Explored Cell"),
            Line2D([0], [0], marker="o", color="w", markerfacecolor=start_col, markersize=6.5, label="Start Position"),
            Line2D([0], [0], marker="D", color="w", markerfacecolor=end_col, markersize=6.5, label="End Position"),
        ]
        legend = ax.legend(
            handles=legend_elements,
            loc="lower center",
            bbox_to_anchor=(0.5, 1.025),
            ncol=3,
            facecolor="#060c18",
            edgecolor="#1e3a66",
            framealpha=1.0,
            fontsize=7.3,
            labelcolor="#e2e8f0",
            handletextpad=0.5,
            columnspacing=1.0,
            borderpad=0.55,
        )
        legend.get_frame().set_linewidth(1.0)
        legend.set_zorder(30)

        fig.text(0.5, 0.02, "Class Work 8 • RoboMaster EP SLAM Engine • Real-time ToF & Odometry Fusion",
                 ha="center", va="bottom", color="#334155", fontsize=7.2, fontweight="medium")

        fig.subplots_adjust(top=0.810, bottom=0.075, left=0.12, right=0.94)
        fig.savefig(img_file, dpi=300, facecolor=fig_bg)
        plt.close(fig)
        print(f"บันทึกไฟล์ผลลัพธ์ทั้งหมดไว้ที่: {OUTPUT_DIR}")
        print("-> exploration_log.csv, map_evaluation_report.txt, walls_data.csv, robot_trajectory.png เรียบร้อยแล้ว")
    except Exception as e:
        print(f"ไม่สามารถบันทึกรูปภาพได้: {e}")


def re_evaluate_from_saved():
    """
    โหลดข้อมูลแนวกำแพงและประวัติการเดินจากไฟล์ CSV ที่บันทึกไว้
    นำมาคำนวณ Map Accuracy เทียบกับ Ground Truth ใน config.py อีกครั้ง
    โดยไม่ต้องนำหุ่นไปวิ่งใหม่
    """
    import config
    import importlib
    importlib.reload(config)

    walls_file = os.path.join(OUTPUT_DIR, "walls_data.csv")
    log_file = os.path.join(OUTPUT_DIR, "exploration_log.csv")

    if not os.path.exists(walls_file) or not os.path.exists(log_file):
        print("⚠️ ไม่พบไฟล์ walls_data.csv หรือ exploration_log.csv กรุณารันหุ่นยนต์หรือ simulation อย่างน้อย 1 ครั้งก่อน")
        return

    state.detected_h_walls.clear()
    state.detected_v_walls.clear()
    state.visited_cells.clear()
    state.discovered_cells.clear()
    state.trajectory.clear()

    # 1. โหลดแนวกำแพงที่หุ่นเคยสแกนพบ
    with open(walls_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if not row or len(row) < 3:
                continue
            w_type, wx, wy = row[0].strip(), int(row[1]), int(row[2])
            if w_type == "H":
                state.detected_h_walls.add((wx, wy))
            elif w_type == "V":
                state.detected_v_walls.add((wx, wy))

    # 2. โหลดประวัติการเดินและช่องที่เคยไป
    prev_cell = None
    with open(log_file, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            cell = (int(row["grid_x"]), int(row["grid_y"]))
            state.visited_cells.add(cell)
            state.discovered_cells.add(cell)
            state.trajectory.append({
                "step": int(row.get("step", 0)),
                "timestamp": float(row.get("timestamp", 0)),
                "grid_x": cell[0],
                "grid_y": cell[1],
                "x_m": float(row.get("x_m", 0)),
                "y_m": float(row.get("y_m", 0)),
                "heading": row.get("heading", "NORTH"),
                "tof_mm": float(row.get("tof_mm", 9999)),
                "cell": cell,
                "previous": prev_cell,
            })
            prev_cell = cell

    eval_metrics = evaluate_map_accuracy(config.GROUND_TRUTH_H_WALLS, config.GROUND_TRUTH_V_WALLS)
    print("\n=======================================================")
    print("📊 ผลการประเมินความถูกต้องใหม่ (RE-EVALUATION REPORT)")
    print("=======================================================")
    print(f"• Coverage     = {eval_metrics['coverage_pct']:.2f}% ({len(state.visited_cells)}/{eval_metrics['total_cells']} ช่อง)")
    print(f"• Map Accuracy = {eval_metrics['map_accuracy_pct']:.2f}% ({eval_metrics['correct_cells']}/{eval_metrics['total_cells']} ช่อง)")
    print(f"• แนวกำแพงโฟม  : ตรวจพบ {eval_metrics['detected_walls']} แนวกำแพง (Ground Truth: {eval_metrics['gt_walls']} แนว)")
    print("=======================================================\n")

    save_outputs()
    print("✅ อัปเดต map_evaluation_report.txt และ robot_trajectory.png เรียบร้อยแล้ว!")


if __name__ == "__main__":
    re_evaluate_from_saved()

