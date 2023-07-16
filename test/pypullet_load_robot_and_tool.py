import os

from compas.data import json_load
from compas.robots import  RobotModel, ToolModel
from compas_fab.robots import  Tool, CollisionMesh

from pybullet_planning import LockRenderer, set_camera, wait_if_gui
from npp.load import load_pybullet_with_robot, add_tool_to_client

# Load Pybullet Client
urdf_filename = os.path.join('robot', 'abb_crb15000_support', 'urdf', 'crb15000_5_95.urdf')
srdf_filename = os.path.join('robot', 'abb_crb15000_support', 'srdf', 'abb_crb15000_5_95.srdf')
client, robot, robot_uid = load_pybullet_with_robot(urdf_filename, srdf_filename, viewer=True)

# Load Tool Model from json, create Tool, add it to Robot
tool_json_path = os.path.join('tool', 'otto1', 'otto1.json')
tool_model = json_load(tool_json_path) #type: ToolModel
tool = Tool.from_tool_model(tool_model)
tool.link_name = 'tool0'
touch_links = ['tool0', 'flange', 'link_6']
robot.attach_tool(tool,touch_links=touch_links)

# Add Tool to Pybullet Client
urdf_package_path = os.path.join('tool', 'otto1')
add_tool_to_client(client, robot, tool, urdf_package_path, touch_links=touch_links)

# Load some Collision Meshes
collision_meshes_path = os.path.join('test', 'design', 'CollisionMesh1_Table.json')
collision_meshes = json_load(collision_meshes_path)
for i, mesh in enumerate(collision_meshes):
    cm = CollisionMesh(mesh, 'static_cm_%i' % i)
    client.add_collision_mesh(cm, {})

wait_if_gui()
pass