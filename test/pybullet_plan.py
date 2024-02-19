import time
import random
import math
from compas_fab.robots import Robot
from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import LockRenderer, set_camera, wait_if_gui
import os
from compas.data import json_load, json_dump
from compas.robots import RobotModel, ToolModel, Configuration
from compas_fab.robots import Tool, CollisionMesh
from compas.geometry import Plane, Frame, Point, Vector, Transformation, Translation
from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera

from npp.tasks import PlanningProblem, RoboticMovement, RoboticLinearMovement
from npp.plan import random_ik, rotate_frame, create_rotated_frames_by_steps, check_pp_ik, check_pp_with_global_ik_rotation, test_all_angles_for_a_frame
from npp.load import load_pybullet_with_robot, add_tool_to_client, load_planning_problem

current_file_location = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_location)
root_folder_path = os.path.dirname(current_folder_path)

# Enable to viewer to see how Pybullet is planning
# Disable viewer to speed up the planning process
viewer_enabled = False

try:
    from typing import Optional, List, Tuple
except:
    pass

# Load Pybullet Client
# Change the location of the URDF and SRDF files if needed
urdf_filename = os.path.join(
    root_folder_path, 'robot', 'abb_crb15000_support', 'urdf', 'crb15000_5_95.urdf')
srdf_filename = os.path.join(
    root_folder_path, 'robot', 'abb_crb15000_support', 'srdf', 'abb_crb15000_5_95.srdf')
client, robot, robot_uid = load_pybullet_with_robot(
    urdf_filename, srdf_filename, viewer=viewer_enabled)

# Load Tool Model from json, create Tool, add it to Robot
# Change the location of the Tool json file if needed
tool_json_path = os.path.join(
    root_folder_path, 'tool', 'BioPrint901', 'BioPrint901.json')
tool_model = json_load(tool_json_path)  # type: ToolModel
tool = Tool.from_tool_model(tool_model)

# Define touch links for attaching the tool to the robot
# The touch links are the links on the robot that can collide with the tool
tool.link_name = 'tool0'
touch_links = ['tool0', 'flange', 'link_6']
robot.attach_tool(tool, touch_links=touch_links)

# Add Tool to Pybullet Client
# Change the tool name if needed
urdf_package_path = os.path.join('tool', 'BioPrint901')
add_tool_to_client(client, robot, tool, urdf_package_path,
                   touch_links=touch_links)

# Load Planning Problem
# Change the location of the Planning Problem json file if needed
planning_problem_path = os.path.join(
    'test', 'design', 'planning_problem_1_newtool.json')
pp = load_planning_problem(planning_problem_path)  # type: PlanningProblem
pp.renumber_task_ids()

# Output Location for Planning Result
# Change the location of the output json file if needed
result_filename = os.path.join(
    root_folder_path, 'test', 'design', 'planning_result_1_newtool.json')

# Set Collision Meshes
for i, cm in enumerate(pp.static_collision_meshes):
    client.add_collision_mesh(cm)

# ===============
# Planning Begins
# ===============

# Start counting time for planning

# # Check planning problem IK (no target rotation)
# # ==============================================
# start_time = time.time()
# total_count, reachable_count, collisionfree_count = check_pp_ik(
#     client, robot, tool, pp, plane_rotation_angle=None, diagnose=False)
# print("%i Movements, %i Reachable, %i CollisionFree" %
#       (total_count, reachable_count, collisionfree_count))
# end_time = time.time()
# print("Check IK time: %.2f seconds" % (end_time - start_time))

# # Check planning problem IK (with target rotation)
# # ================================================
# start_time = time.time()
# plane_rotation_steps = 20
# global_rotation_angle = check_pp_with_global_ik_rotation(
#     client, robot, tool, pp, plane_rotation_steps=plane_rotation_steps, diagnose=False)
# end_time = time.time()
# print("Check IK (%d steps) time: %.2f seconds" %
#       (plane_rotation_steps, end_time - start_time))


# Planning all movements in the planning problem
# ==============================================

movements = pp.get_robotic_movements()
# Skip the last movement
movements = movements[1:-2]

first_movement = movements[0]  # type: RoboticMovement
plane_rotation_steps = 10
starting_config = pp.start_configuration    # type: Configuration
possible_angles, planned_configurations = test_all_angles_for_a_frame(
    client, robot, tool, first_movement, starting_config, tries=1, plane_rotation_steps=plane_rotation_steps)

starting_config = planned_configurations[0] # type: Configuration

# Plan each of the movements, with a depth first search approach
# The first movement is planned with random IK, the rest are planned with gradient IK
plane_rotation_steps = 50
jump_tolerance = 0.5


def dfs(all_movements, currentStep, planned_configurations=None):
    if planned_configurations is None:
        planned_configurations = []

    # Define a function to check if current step is goal
    if currentStep >= len(all_movements):
        print("Search is complete")
        return planned_configurations

    this_movement = movements[currentStep]
    possible_angles, possible_configurations = test_all_angles_for_a_frame(
        client, robot, tool, this_movement, planned_configurations[-1], tries=1, plane_rotation_steps=plane_rotation_steps)
    possible_configurations = [c for c in possible_configurations if c.close_to(
        planned_configurations[-1], jump_tolerance)]
    # random.shuffle(possible_configurations)
    print("Step: %i has %i options" %
          (currentStep, len(possible_configurations)))

    for possible_configuration in possible_configurations:

        planned_configurations.append(possible_configuration)
        result = dfs(all_movements, currentStep + 1, planned_configurations)
        if result is not None:
            return result
        planned_configurations.pop()

    print("Step: %i no options works" % (currentStep))


# Using the DFS function to plan the movements
complete_plan = dfs(movements, 1, planned_configurations=[starting_config])

# After planning, set the trajectory result in each movement
for i in range(len(movements)):
    movement = movements[i]
    movement.trajectory = [complete_plan[i]]

# Save the result to file
json_dump(pp, result_filename)

# Visualize the result if viewer is enabled
if viewer_enabled:
    if complete_plan is not None:
        step = 0
        while (True):
            config = complete_plan[step]
            client.set_robot_configuration(robot, config)
            value = input("Click to continue")
            if value == "q":
                break
            elif value == "b":
                step = max(0, (step - 1))
            else:
                step = min(len(complete_plan) - 1, (step + 1))
