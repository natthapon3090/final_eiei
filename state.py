# -*- coding: utf-8 -*-
"""
Shared SLAM State Module.
Stores runtime state including detected walls, visited cells, trajectory logs,
and control flags.
"""

detected_h_walls = set()
detected_v_walls = set()
visited_cells = set()
discovered_cells = set()
trajectory = []
exploration_stack = []
current_distance = 9999
ir_values = None
current_yaw = 0.0
initial_yaw = None
initial_heading = "NORTH"
chassis_pos_x = 0.0
chassis_pos_y = 0.0
stop_requested = False


def reset_state(start_pos):
    """Reset all SLAM runtime state variables for a new exploration run."""
    global current_distance, current_yaw, initial_yaw, initial_heading, chassis_pos_x, chassis_pos_y, stop_requested, ir_values
    detected_h_walls.clear()
    detected_v_walls.clear()
    visited_cells.clear()
    discovered_cells.clear()
    discovered_cells.add(start_pos)
    trajectory.clear()
    exploration_stack.clear()
    exploration_stack.append(start_pos)
    current_distance = 9999
    ir_values = None
    current_yaw = 0.0
    initial_yaw = None
    initial_heading = "NORTH"
    chassis_pos_x = 0.0
    chassis_pos_y = 0.0
    stop_requested = False

