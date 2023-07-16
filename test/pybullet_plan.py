from compas.robots import RobotModel, Configuration, ToolModel
from compas_fab.robots import RobotSemantics
from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera
from compas_fab_pychoreo.client import PyChoreoClient
from compas.geometry import Plane, Frame, Point, Vector, Transformation, Translation
from compas.data import DataDecoder 
import os
from npp.plan import random_ik, plan_one_stroke, rotate_frame, create_rotated_frames_by_steps
from npp.load import load_pybullet_with_robot, load_toolmodel_from_json, add_tool_to_client, load_planes

current_file_location = os.path.abspath(__file__)
current_folder_path = os.path.dirname(current_file_location)

# File locations for robot and tool
urdf_filename = os.path.join(current_folder_path, "..", "robot", "abb_crb15000_support",  "urdf", "crb15000_5_95.urdf")
srdf_filename = os.path.join(current_folder_path, "..", "robot", "abb_crb15000_support",  "srdf", "abb_crb15000_5_95.srdf")
toolmodel_path = os.path.join(current_folder_path, "..", 'tool', 'otto1', 'otto1.json')
# File location for design
design_path = os.path.join(current_folder_path, "design", "planning_one_circle.json")

# Loading Robots and Planners
# __________________



# Planning
# __________________



client, robot, robot_uid = load_pybullet_with_robot(urdf_filename, srdf_filename, viewer=True)

# Load Planes and convert them to tcp_frames
planes = load_planes(design_path, scale=1e-3)
tcp_frames = [Frame(p.point, Vector(0.0, 1.0, 0.0), Vector(1.0, 0.0, 0.0)) for p in planes]

# Move the design in space
# T = Translation.from_vector([0.2,0,-0.5])
T = Translation.from_vector([0.2,0,0.0])
tcp_frames = [f.transformed(T) for f in tcp_frames]

# Load Tool Model
tool = load_toolmodel_from_json(toolmodel_path)
add_tool_to_client(client, robot, tool, toolmodel_path)

# Compute robot target (aka. flange tcp_frame) 

# Create different rotated tcp_frames around the plane
tcp_frame = tcp_frames[0]
rotated_tcp_frames = create_rotated_frames_by_steps(tcp_frame, num_steps= 20)
flange_frames = tool.from_tcf_to_t0cf(rotated_tcp_frames)

configurations= []

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
for angle in range(0, 360, 18):
    configurations = []
    tcp_frames = [rotate_frame(f, angle) for f in tcp_frames]
    flange_frames = tool.from_tcf_to_t0cf(tcp_frames)
    current_config = robot.zero_configuration()
    for frame in flange_frames:
        current_config = random_ik(client, robot, frame, 
                                starting_config=current_config, 
                                tries = 20, 
                                collision = True, 
                                visualize=False)
        configurations.append(current_config)
        # print ("Frame:" , frame) 
        # print ("Result:" , current_config) 



    num_items = len(configurations)
    num_not_none = sum(c is not None for c in configurations)
    print(f"Angle {angle} : {num_not_none}/{num_items} succeeded")

# configurations = plan_one_stroke(robot, client, tcp_frames, draw=False)
# for config in configurations:
#     print (config) 
#     if config is not None:
#         client.set_robot_configuration(robot, config)
#         _  = input("Waiting")

