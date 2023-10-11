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
from npp.plan import random_ik, plan_one_stroke, rotate_frame, create_rotated_frames_by_steps
from npp.load import load_pybullet_with_robot, add_tool_to_client, load_planning_problem

current_file_location = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_location)
root_folder_path = os.path.dirname(current_folder_path)
sys.setrecursionlimit(50000)

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
    urdf_filename, srdf_filename, viewer=True)
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
    'test', 'design', 'PathPlanning', '231010_PathPlanning_BioPrint901_AutomatedPathes_VariableSpeed_V1.json')
# 'test', 'design', 'PathPlanning', 'temp.json')
pp = load_planning_problem(planning_problem_path)
pp.renumber_task_ids()

# Output Location
result_filename = os.path.join(
    root_folder_path, 'test', 'design', 'PlanningResult', 'planning_result_231010_AutomatedPathes_VariableSpeed_V1.json')
# root_folder_path, 'test', 'design', 'PlanningResult', 'planning_result_temp.json')

# Set Collision Meshes
for i, cm in enumerate(pp.static_collision_meshes):
    client.add_collision_mesh(cm)

# Check IK (no plane rotation)


def check_pp_ik(robot, client, tool, pp, plane_rotation_angle=None, diagnose=False):
    # type: (Robot, PyChoreoClient, Tool, PlanningProblem, Optional[float], bool) -> None
    total_count = 0
    reachable_count = 0
    collisionfree_count = 0
    with LockRenderer():
        for robotic_movement in pp.get_robotic_movements():
            total_count += 1
            # Some robotic movement may not have target frame because it uses target configuration
            if robotic_movement.target_frame is not None:
                # Rotate target frame if plane_rotation is provided
                if plane_rotation_angle is not None:
                    tcp_frame = rotate_frame(
                        robotic_movement.target_frame, plane_rotation_angle)
                else:
                    tcp_frame = robotic_movement.target_frame
                # Convert target frame from tool frame to flange frame
                target_frame = tool.from_tcf_to_t0cf([tcp_frame])[0]
                # Compute IK
                configuration = client.inverse_kinematics(
                    robot, target_frame, group=None, options={'avoid_collisions': False})
                # print ("Task [%s] %s" % (robotic_movement.task_id, robotic_movement.tag))
                # print ("IK Result:" , configuration)
                # Check Collision
                if configuration is not None:
                    reachable_count += 1
                    cc_result = client.check_collisions(
                        robot, configuration, options={'diagnosis': diagnose})
                    if not cc_result:
                        collisionfree_count += 1
            else:
                # Movements with target configuration are assumed to be reachable
                reachable_count += 1
                # Check Collision for target configuration
                cc_result = client.check_collisions(
                    robot, robotic_movement.target_configuration, options={'diagnosis': diagnose})
                if not cc_result:
                    collisionfree_count += 1
    return total_count, reachable_count, collisionfree_count


# total_count, reachable_count, collisionfree_count = check_pp_ik(
#     robot, client, tool, pp, plane_rotation_angle=None, diagnose=False)
# print("%i Movements, %i Reachable, %i CollisionFree" %
#       (total_count, reachable_count, collisionfree_count))


def check_pp_with_global_ik_rotation(robot, client, tool, pp, plane_rotation_steps=36, diagnose=False):
    possible_angles = []
    for i in range(plane_rotation_steps):
        angle = 360 / plane_rotation_steps * i
        angle_rad = math.radians(angle)
        total_count, reachable_count, collisionfree_count = check_pp_ik(
            robot, client, tool, pp, plane_rotation_angle=angle_rad, diagnose=False)
        print("Rotation(%.1f degrees) : %i Movements, %i Reachable, %i CollisionFree" % (
            angle, total_count, reachable_count, collisionfree_count))
        if total_count == collisionfree_count:
            possible_angles.append(angle)
    return possible_angles


# global_rotation_angle = check_pp_with_global_ik_rotation(
#     robot, client, tool, pp, plane_rotation_steps=20, diagnose=False)

# Find all possible angles for the first linear movement
# random_ik is used for the first movement because it is not bounded by neighbours


def test_all_angles_for_a_frame(client, robot, tool, robotic_movement, starting_config, tries=1, plane_rotation_steps=36):
    # type: (PyChoreoClient, Robot, Tool, RoboticMovement, Configuration, int, int) -> Tuple[List[float], List[Configuration]]
    """ Test all angles for a given robotic movement, return a list of possible angles and planned configurations"""
    possible_angles = []
    planned_configurations = []
    for i in range(plane_rotation_steps):
        angle = 360 / plane_rotation_steps * i
        angle_rad = math.radians(angle)
        tcp_frame = rotate_frame(robotic_movement.target_frame, angle_rad)
        # Convert target frame from tool frame to flange frame
        target_frame = tool.from_tcf_to_t0cf([tcp_frame])[0]
        # Compute IK
        configuration = random_ik(
            client, robot, target_frame, starting_config=starting_config, tries=tries, collision=True, visualize=False)
        # configuration = random_ik(client, robot, target_frame, starting_config=starting_config, tries = tries, collision = False, visualize = False)
        if configuration is not None:
            possible_angles.append(angle)
            planned_configurations.append(configuration)
            starting_config = configuration
    return possible_angles, planned_configurations


movements = pp.get_robotic_movements()
# Skip the last movement
# movements = movements[1:-2]
movements = movements[1:-1]
# movements = movements[1:10]

first_movement = movements[0]  # type: RoboticMovement
plane_rotation_steps = 10
starting_config = pp.start_configuration
possible_angles, planned_configurations = test_all_angles_for_a_frame(
    client, robot, tool, first_movement, starting_config, tries=1, plane_rotation_steps=plane_rotation_steps)

starting_config = planned_configurations[0]

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


# The first call of the function would look like this:
complete_plan = dfs(movements, 1, planned_configurations=[starting_config])
for i in range(len(movements)):
    movement = movements[i]
    movement.trajectory = [complete_plan[i]]


json_dump(pp, result_filename)

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

pass


# Usage:

# ik_result = client.inverse_kinematics(
#     robot, first_motion.target_frame,
#     group=None,
#     options={'avoid_collisions': False})
# print(ik_result)
# pass

# Load Planes and convert them to tcp_frames
# planes = load_planes(design_path, scale=1e-3)
# tcp_frames = [Frame(p.point, Vector(0.0, 1.0, 0.0), Vector(1.0, 0.0, 0.0)) for p in planes]

# Move the design in space
# T = Translation.from_vector([0.2,0,-0.5])
# T = Translation.from_vector([0.2,0,0.0])
# tcp_frames = [f.transformed(T) for f in tcp_frames]

# Load Tool Model
# tool = load_toolmodel_from_json(toolmodel_path)
# add_tool_to_client(client, robot, tool, toolmodel_path)

# Compute robot target (aka. flange tcp_frame)

# Create different rotated tcp_frames around the plane
# draw_pose(first_motion.target_frame)
# rotated_tcp_frames = create_rotated_frames_by_steps(first_motion.target_frame, num_steps= 20)
# flange_frames = tool.from_tcf_to_t0cf(rotated_tcp_frames)
# flange_frames = rotated_tcp_frames
# configurations= []

# Demonstrating differnt rotational planning oppurtunity, no collision check
# for frame in flange_frames:
#     configuration = client.inverse_kinematics(robot, frame, group=None, options={'avoid_collisions': False})
#     configurations.append(configuration)
#     print ("Frame:" , frame)
#     print ("Result:" , configuration)
#     if configuration is not None:
#         cc_result = client.check_collisions(robot, configuration, options={'diagnosis':True})
#         print ("Collision:" , cc_result)

# # Demonstrating random IK, with collision check
# for i in range (20):
#     configuration = random_ik(client, robot, flange_frames[0], tries = 100, collision = True, visualize=False)
#     configurations.append(configuration)
#     print ("Attempt:" , i)
#     print ("Result:" , configuration)

# # Demonstrating random IK, same rotated plane, with CC.
# for frame in flange_frames:
#     configuration = random_ik(client, robot, frame, tries = 20, collision = True, visualize=False)
#     configurations.append(configuration)
#     print ("Frame:" , frame)
#     print ("Result:" , configuration)

# Planning the whole design, no rotated plane, with CC.
# for angle in range(0, 360, 18):
#     configurations = []
#     tcp_frames = [rotate_frame(f, angle) for f in tcp_frames]
#     flange_frames = tool.from_tcf_to_t0cf(tcp_frames)
#     current_config = robot.zero_configuration()
#     for frame in flange_frames:
#         current_config = random_ik(client, robot, frame,
#                                 starting_config=current_config,
#                                 tries = 20,
#                                 collision = True,
#                                 visualize=False)
#         configurations.append(current_config)
#         # print ("Frame:" , frame)
#         # print ("Result:" , current_config)


#     num_items = len(configurations)
#     num_not_none = sum(c is not None for c in configurations)
#     print(f"Angle {angle} : {num_not_none}/{num_items} succeeded")

# configurations = plan_one_stroke(robot, client, tcp_frames, draw=False)
# for config in configurations:
#     print (config)
#     if config is not None:
#         client.set_robot_configuration(robot, config)
#         _  = input("Waiting")
