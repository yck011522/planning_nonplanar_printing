import cProfile
from pstats import SortKey
import pstats
import time
import random
import math
from compas_fab.robots import Robot
from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import LockRenderer, set_camera, wait_if_gui
import os
import sys
from compas.data import json_load, json_dump
from compas.robots import RobotModel, ToolModel, Configuration
from compas_fab.robots import Tool, CollisionMesh
from compas.geometry import Plane, Frame, Point, Vector, Transformation, Translation
from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera

from npp.tasks import PlanningProblem, RoboticMovement, RoboticLinearMovement
from npp.plan import random_ik, rotate_frame, create_rotated_frames_by_steps
from npp.load import load_pybullet_with_robot, add_tool_to_client, load_planning_problem

current_file_location = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_location)
root_folder_path = os.path.dirname(current_folder_path)
sys.setrecursionlimit(500000000)
print(sys.getrecursionlimit())

viewer_enabled = True  # Normally False to plan faster, True to visualize
avoid_collision = True  # Normally True, False only for debugging


# Lower number for quickly trying many different starting angles
first_plane_rotation_steps = 10
plane_rotation_steps = 50  # Higher number for more accurate search
jump_tolerance = 0.7

# Load
try:
    from typing import Optional, List, Tuple
except:
    pass

# Load Pybullet Client
urdf_filename = os.path.join(
    root_folder_path, 'robot', 'abb_crb15000_support', 'urdf', 'crb15000_5_95.urdf')
srdf_filename = os.path.join(
    root_folder_path, 'robot', 'abb_crb15000_support', 'srdf', 'abb_crb15000_5_95.srdf')
client, robot, robot_uid = load_pybullet_with_robot(
    urdf_filename, srdf_filename, viewer=viewer_enabled)
# client, robot, robot_uid = load_pybullet_with_robot(urdf_filename, srdf_filename, viewer=False)

# Load Tool Model from json, create Tool, add it to Robot
tool_json_path = os.path.join(
    root_folder_path, 'tool', 'BioPrint901', 'BioPrint901.json')
tool_model = json_load(tool_json_path)  # type: ToolModel
tool = Tool.from_tool_model(tool_model)
tool.link_name = 'tool0'
touch_links = ['tool0', 'flange', 'link_6']
robot.attach_tool(tool, touch_links=touch_links)

# Add Tool to Pybullet Client
urdf_package_path = os.path.join('tool', 'BioPrint901')
add_tool_to_client(client, robot, tool, urdf_package_path,
                   touch_links=touch_links)

# Load Planning Problem
planning_problem_path = os.path.join(
    # 'test', 'design', 'PathPlanning', '240207_PathPlanning_BioPrint901_Panel_V2.json')
    # 'test', 'design', 'PlanningResult', 'planning_result_231010_AutomatedPathes_VariableSpeed_V1.json')
    # 'test', 'design', 'PlanningResult', 'planning_result_230901_Cake_V1_Part1.json')
    'test', 'design', 'PathPlanning', '231011_PathPlanning_BioPrint901_AutomatedPathes+VariableSpeed_12ptCake_V1.json')
# 'test', 'design', 'PathPlanning', 'temp.json')
pp = load_planning_problem(planning_problem_path)
pp.renumber_task_ids()

# Print number of movements
print("Number of Movements: ", len(pp.get_robotic_movements()))


# Set Collision Meshes
for i, cm in enumerate(pp.static_collision_meshes):
    client.add_collision_mesh(cm)


# def rotated_frames_generator(source_frame, plane_rotation_steps=36):
#     step_size = 360 / plane_rotation_steps
#     for i in range(plane_rotation_steps):
#         factor = 0
#         if i > 0:
#             factor = math.ceil(i / 2)
#         if i % 2 == 0:
#             factor *= -1
#         angle_rad = math.radians(step_size * factor)
#         yield rotate_frame(source_frame, angle_rad)


def rotated_frames_generator(source_frame: Frame, plane_rotation_steps: int = 36, guide_vector_x: Vector = None):
    # If guide_vector_x is provided, rotate the source_frame to align with the guide_vector_x
    if guide_vector_x is not None:
        guide_vector_projected = source_frame.to_world_coordinates(
            guide_vector_x)
        angle = math.atan2(
            guide_vector_projected[1], guide_vector_projected[0])
        source_frame = rotate_frame(source_frame, angle)

    step_size = 360 / plane_rotation_steps
    for i in range(plane_rotation_steps):
        factor = 0
        if i > 0:
            factor = math.ceil(i / 2)
        if i % 2 == 0:
            factor *= -1
        angle_rad = math.radians(step_size * factor)
        yield rotate_frame(source_frame, angle_rad)


def test_frame_IK(client, robot, tool, tcp_frame, starting_config=None, tries=1, return_all=False):
    flange_frame = tool.from_tcf_to_t0cf([tcp_frame])[0]
    return random_ik(client, robot, flange_frame, starting_config=starting_config, tries=tries, collision=avoid_collision, visualize=False, return_all=return_all)


# Start time
start_time = time.time()

movements = pp.get_robotic_movements()
# Skip the last movement
movements = movements[1:-1]

# Plan each of the movements, with a depth first search approach


def dfs(all_movements, current_step, initial_config, frame_generators=[]):
    """Depth First Search to find a valid plan for all movements.

    The search will explore only one options at a time for a given step.
    Once a IK solution is found for a given step, the search will move to the next step.
    If the search reaches a deadend, it will backtrack to the previous step and try the next option.

    This function is written without recursion to avoid reaching system recursion limit.
    """
    planned_configurations = []
    option_index = [0]

    def stop_condition():
        return len(planned_configurations) == len(all_movements)

    while True:
        # Check 1: if the current step generator has any more options
        frame_option = next(frame_generators[-1], None)
        option_index[-1] += 1

        # If C1 == NO: Decrement the step_index and try the next option in the previous step
        if frame_option is None and current_step == 0:
            print("Search Failed")
            return None

        if frame_option is None and current_step > 0:
            print("Step: %i reached deadend after %i options" %
                  (current_step, option_index[-1] - 1))
            current_step -= 1
            frame_generators.pop()
            planned_configurations.pop()
            option_index.pop()
            continue

        # If C1 == YES: Check 2: if the current step has a valid IK solution
        ik_start_config = [initial_config] + planned_configurations
        tries = 16 if current_step == 0 else 1
        # return_all = True if current_step == 0 else False
        return_all = False
        result_config = test_frame_IK(
            client, robot, tool, frame_option, ik_start_config[-1], tries, return_all)

        # If C2 == YES: Add the configuration to the plan,  create generator for next step and increment the step_index
        if result_config is not None:
            print("Step: %i Option %i is valid" %
                  (current_step, option_index[-1]))
            planned_configurations.append(result_config)

            # If this is the last step, return the plan
            if stop_condition():
                print("Search is complete. Returning %i configurations" %
                      len(planned_configurations))
                return planned_configurations

            # Create a generator for the next step
            next_target_frame = all_movements[current_step + 1].target_frame
            next_generator = rotated_frames_generator(
                next_target_frame, plane_rotation_steps, frame_option.xaxis)
            frame_generators.append(next_generator)
            option_index.append(0)
            current_step += 1
            continue

        # If C2 == NO: Let Loop Continue and try the next option
        if result_config is None:
            # print("Step: %i Option %i is invalid" %
            #       (current_step, option_index[-1]))
            continue


def plan_all_movements_with_initial_config(movements, first_plane_rotation_steps=10):
    """Function to create the initial generator and start the search for a complete plan."""
    first_movement = movements[0]  # type: RoboticMovement

    # Starting configuration defined by user to help with the search
    starting_config = pp.start_configuration
    initial_generator = rotated_frames_generator(
        first_movement.target_frame, first_plane_rotation_steps)
    complete_plan = dfs(movements, 0, starting_config, [initial_generator])
    return complete_plan


cProfile.run(
    'plan_all_movements_with_initial_config(movements)', 'restats')
print("Total Time: %.2f seconds" % (time.time() - start_time))

p = pstats.Stats('restats')
print(" -------------- ")
p.sort_stats(SortKey.CUMULATIVE).print_stats(30)


# The first call of the function would look like this:

# complete_plan = plan_all_movements_with_initial_config(movements)

# if complete_plan is not None:
#     print("Complete Plan is found, total time: %.2f seconds" %
#           (time.time() - start_time))
#     print("Total Steps: %i" % len(complete_plan))
# else:
#     print("Complete Plan is not found")
