# -*- coding: utf-8 -*-
"""
Class Work 8: RoboMaster Autonomous SLAM in Foam Wall Maze.
Main entry point orchestrating exploration, sensor fusion, path planning,
and graphical visualization.

Usage:
    python slam.py --sim
    python slam.py
    python slam.py --sim --no-dashboard --start-x 1 --start-y 1 --start-heading NORTH
"""

import argparse
import time
import tkinter as tk

from config import DIRECTIONS, GRID_SIZE_M, adjacent
from dashboard import Dashboard
from evaluation import evaluate_map_accuracy, save_outputs
from navigation import find_path_to_nearest_unvisited
from robot_control import (
    align_heading,
    move_one_cell,
    recenter_in_cell,
    robot,
    scan_4_directions,
    stop_and_settle,
    sub_attitude_handler,
    sub_ir_handler,
    sub_position_handler,
    sub_tof_handler,
    turn_to_direction,
)
import state


def run_exploration(sim_mode, start_config, dashboard, ground_truth=None):
    """
    Main exploration loop ensuring the robot visits every reachable cell in the maze.
    Orchestrates gimbal scanning, Mecanum recentering, BFS path planning, and movement.
    """
    start_pos = (start_config[0], start_config[1])
    heading = start_config[2]
    position = start_pos

    # Reset all runtime state for a fresh run
    state.reset_state(start_pos)
    state.initial_heading = heading

    ep_robot = ep_chassis = ep_gimbal = ep_sensor = ep_ir_adapter = None

    try:
        if not sim_mode:
            if robot is None:
                raise RuntimeError("ติดตั้ง RoboMaster SDK ด้วย python -m pip install robomaster")
            if dashboard:
                dashboard.log("กำลังเชื่อมต่อ RoboMaster...")
            ep_robot = robot.Robot()
            ep_robot.initialize(conn_type="ap")
            ep_chassis, ep_gimbal, ep_sensor = ep_robot.chassis, ep_robot.gimbal, ep_robot.sensor
            ep_ir_adapter = ep_robot.sensor_adaptor
            ep_gimbal.recenter().wait_for_completed()
            ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
            ep_ir_adapter.sub_adapter(freq=50, callback=sub_ir_handler)
            ep_chassis.sub_attitude(freq=20, callback=sub_attitude_handler)
            ep_chassis.sub_position(freq=20, callback=sub_position_handler)
            time.sleep(0.80)  # Allow IMU attitude stream to fully settle before calibrating initial_yaw
            if dashboard:
                dashboard.log("เชื่อมต่อ RoboMaster สำเร็จ!")

        print(f"เริ่มสำรวจจากตำแหน่ง {position} ทิศ {heading}")
        if dashboard:
            dashboard.log(f"เริ่มสำรวจจาก {position} ทิศ {heading}")

        step = 0
        while not state.stop_requested:
            # Stop and let chassis motion settle before every gimbal scan,
            # including when revisiting an already explored cell.
            stop_and_settle(ep_chassis, sim_mode, settle_s=0.25)
            readings = scan_4_directions(
                ep_chassis, ep_gimbal, position, heading, sim_mode, dashboard, step
            )
            state.visited_cells.add(position)
            if state.stop_requested:
                break

            # One four-direction gimbal scan, followed by one recenter move.
            recenter_in_cell(ep_chassis, ep_gimbal, readings, position, heading, sim_mode, dashboard)
            if state.stop_requested:
                break

            # 3. Log trajectory
            previous = state.trajectory[-1]["cell"] if state.trajectory else None
            state.trajectory.append({
                "step": step,
                "timestamp": round(time.time(), 2),
                "grid_x": position[0],
                "grid_y": position[1],
                "x_m": round((position[0] - 0.5) * GRID_SIZE_M, 3),
                "y_m": round((position[1] - 0.5) * GRID_SIZE_M, 3),
                "heading": heading,
                "tof_mm": min(readings.values()) if readings else 9999,
                "cell": position,
                "previous": previous,
                "stack_depth": len(state.exploration_stack),
            })

            # 4. Find path to nearest unvisited cell through open passages
            path = find_path_to_nearest_unvisited(position, state.visited_cells)
            if not path or len(path) < 2:
                if dashboard:
                    dashboard.log("✅ สำรวจครบทุกช่องที่เดินได้แล้ว (All reachable cells visited)!")
                break

            next_cell = path[1]
            target_heading = next(d for d in DIRECTIONS if adjacent(position, d) == next_cell)

            if dashboard:
                target_desc = "ช่องใหม่" if next_cell not in state.visited_cells else "เดินทางผ่าน"
                dashboard.log(f"-> เดินไป {next_cell} [{target_desc}] (หัน {target_heading})")
                dashboard.update(position, target_heading, step, f"กำลังเคลื่อนที่ไป {next_cell}")

            # 5. Turn chassis to target heading at the center of the cell
            # Because recentering just completed, the robot has full clearance from all walls (~9cm+)
            if heading != target_heading and dashboard:
                dashboard.log(f"-> หันตัวจาก {heading} -> {target_heading} ที่จุดกึ่งกลางช่อง...")
            turn_to_direction(ep_chassis, ep_gimbal, heading, target_heading, sim_mode)
            align_heading(ep_chassis, target_heading, sim_mode)
            heading = target_heading
            # Previously visited cells are path retraces: keep a straight
            # heading correction active while the forward IR/ToF prevents impact.
            move_ok = move_one_cell(
                ep_chassis,
                sim_mode,
                heading=heading,
                returning=next_cell in state.visited_cells,
            )

            if move_ok:
                position = next_cell
                state.exploration_stack.append(position)
            else:
                # Emergency stop: mark the wall we almost hit so BFS won't try again
                x, y = position
                if heading == "NORTH":
                    state.detected_h_walls.add((x, y))
                elif heading == "SOUTH":
                    state.detected_h_walls.add((x, y - 1))
                elif heading == "EAST":
                    state.detected_v_walls.add((x, y))
                elif heading == "WEST":
                    state.detected_v_walls.add((x - 1, y))
                if dashboard:
                    dashboard.log(f"⚠ หยุดฉุกเฉิน! พบกำแพงระหว่างเดินทิศ {heading} → mark wall & re-plan")

            step += 1

        eval_metrics = evaluate_map_accuracy()
        print("\n=======================================================")
        print("📊 ผลการประเมินความถูกต้อง (MAP EVALUATION REPORT)")
        print("=======================================================")
        print("• Coverage     = (จำนวน Cell ที่สำรวจแล้ว / จำนวน Cell ทั้งหมด) x 100")
        print(
            f"               = ({len(state.visited_cells)} / {eval_metrics['total_cells']}) x 100 = {eval_metrics['coverage_pct']:.2f}%"
        )
        print("• Map Accuracy = (จำนวน Cell ที่ทายถูก / จำนวน Cell ทั้งหมด) x 100")
        print(
            f"               = ({eval_metrics['correct_cells']} / {eval_metrics['total_cells']}) x 100 = {eval_metrics['map_accuracy_pct']:.2f}%"
        )
        print(f"• แนวกำแพงโฟม  : ตรวจพบ {eval_metrics['detected_walls']} แนวกำแพง (Ground Truth: {eval_metrics['gt_walls']} แนว)")
        print(f"• จบภารกิจที่   : {position} | เดินครบ {len(state.visited_cells)} ช่อง | เริ่มต้นที่ {start_pos}")
        print("=======================================================\n")
        save_outputs()

        if dashboard:
            status_text = "หยุดการทำงาน" if state.stop_requested else f"สำรวจครบทุกช่องแล้ว ({len(state.visited_cells)} ช่อง)"
            dashboard.update(position, heading, step, status_text)
            dashboard.log(
                f"📊 สรุปผล: Coverage = {eval_metrics['coverage_pct']:.1f}% | Map Accuracy = {eval_metrics['map_accuracy_pct']:.1f}%"
            )
            dashboard.log("💾 บันทึก exploration_log.csv, map_evaluation_report.txt และ robot_trajectory.png เรียบร้อย")
            dashboard.btn_stop.config(state=tk.DISABLED)

    except KeyboardInterrupt:
        print("หยุดฉุกเฉิน (KeyboardInterrupt)")
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
        if dashboard:
            dashboard.log(f"เกิดข้อผิดพลาด: {e}")
    finally:
        if ep_sensor:
            try:
                ep_sensor.unsub_distance()
            except Exception:
                pass
        if ep_ir_adapter:
            try:
                ep_ir_adapter.unsub_adapter()
            except Exception:
                pass
        if ep_gimbal:
            try:
                ep_gimbal.recenter().wait_for_completed()
            except Exception:
                pass
        if ep_robot:
            try:
                ep_robot.close()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description="Class Work 8: SLAM Explore Foam Wall Maze")
    parser.add_argument("--sim", action="store_true", help="จำลองโดยไม่เชื่อมต่อหุ่นยนต์จริง")
    parser.add_argument("--no-dashboard", action="store_true", help="ไม่เปิดหน้าต่าง dashboard")
    parser.add_argument("--start-x", type=int, default=1, help="พิกัด X เริ่มต้น (1 ถึง 4)")
    parser.add_argument("--start-y", type=int, default=1, help="พิกัด Y เริ่มต้น (1 ถึง 5)")
    parser.add_argument("--start-heading", choices=DIRECTIONS, default="NORTH", help="ทิศทางเริ่มต้น")
    args = parser.parse_args()

    default_start = (args.start_x, args.start_y, args.start_heading)

    if args.no_dashboard:
        run_exploration(args.sim, default_start, None)
    else:
        dashboard = None

        def on_start(start_cfg):
            run_exploration(args.sim, start_cfg, dashboard)

        def on_stop():
            state.stop_requested = True

        dashboard = Dashboard(
            on_start=on_start,
            on_stop=on_stop,
            sim_mode=args.sim,
            default_start=default_start,
        )
        dashboard.root.mainloop()


if __name__ == "__main__":
    main()
