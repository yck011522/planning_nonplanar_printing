import os
import sys

from compas.geometry import Frame, Transformation
from compas.robots import RobotModel, Configuration, ToolModel
from compas_fab.robots import RobotSemantics, Tool
from compas.data import DataDecoder
from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera
from compas_fab_pychoreo.client import PyChoreoClient

# For testing files locations
sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


def load_pybullet_with_robot(urdf_filename, srdf_filename, viewer=True, verbose=False):
    """Load pybullet with robot and return the planner, robot and robot_uid

    Attributes
    ----------
    urdf_filename : str
        file path to the urdf file
    srdf_filename : str
        file path to the srdf file
    viewer : bool, optional
        whether to show the pybullet viewer, by default True. 
        (Enable this will slow down the loading and planning process)
    verbose : bool, optional
        whether to print out the pybullet planner's log, by default False
    """
    # * load pybullet planner
    planner = PyChoreoClient(viewer=viewer, verbose=verbose)
    planner.connect()

    # * Load robot model and semantics
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    # * Add robot to pybullet
    with LockRenderer():
        robot = planner.load_robot(urdf_filename)
    robot.semantics = semantics

    # * Get robot's unique body index in pybullet
    robot_uid = planner.get_robot_pybullet_uid(robot)

    # * Draw base tcp_frame and locate camera in pybullet
    draw_pose(unit_pose(), length=1.)

    return planner, robot, robot_uid

def load_tool(toolmodel_path):
    import json
    from integral_timber_joints.tools import ToolChanger

    with open(toolmodel_path, 'r') as f:
        tool = json.load(f, cls=DataDecoder) # type:ToolChanger

    tool_urdf_path = os.path.dirname(toolmodel_path)
    tool.save_as_urdf(tool_urdf_path, triangulize=True)
    return tool

def load_toolmodel_from_json(toolmodel_path):
    import json
    from compas.robots import ToolModel

    with open(toolmodel_path, 'r') as f:
        tool = json.load(f, cls=DataDecoder) # type:ToolModel
    return tool

def add_tool_to_robot(client, tool, urdf_path):
    from compas_fab.robots import AttachedCollisionMesh, CollisionMesh

    client.add_tool_from_urdf(tool.name, urdf_path)
    client.add_attached_collision_mesh(
        AttachedCollisionMesh(CollisionMesh(None, 'otto1'),
                            'tool0', touch_links=['tool0', 'flange', 'link_6']),
        options={'robot': robot,
                # 'attached_child_link_name': tool.get_base_link_name(),
                'attached_child_link_name': 'attached_tool_link',
                'parent_link_from_child_link_transformation' : Transformation.from_frame(Frame.worldXY()),
                })
    

    
# Load Pybullet Client
urdf_filename = os.path.join('robot', 'abb_crb15000_support', 'urdf', 'crb15000_5_95.urdf')
srdf_filename = os.path.join('robot', 'abb_crb15000_support', 'srdf', 'abb_crb15000_5_95.srdf')

planner, robot, robot_uid = load_pybullet_with_robot(urdf_filename, srdf_filename, viewer=True)

# Load Tool Model from json
urdf_package_path = os.path.join('test_result', 'tool', 'create_tool')
tool_json_path = os.path.join(urdf_package_path, 'otto1.json')
tool_urdf_package_path = os.path.join(urdf_package_path, 'otto1.urdf')

tool_model = load_toolmodel_from_json(tool_json_path)
tool = Tool.from_tool_model(tool_model)
robot.attach_tool(tool,touch_links=['tool0', 'flange', 'link_6'])
# planner.add_tool_from_urdf(tool.name, tool_urdf_package_path)

planner.add_tool(tool)



add_tool_to_robot(planner, tool, tool_urdf_package_path)


pass