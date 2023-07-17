import os
from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
from compas.robots import MeshDescriptor
from compas.files import URDF
from compas.data import Data
from compas.datastructures import Mesh
from compas.geometry import Frame, Transformation
from compas.robots import RobotModel, ToolModel
from compas_fab.robots import RobotSemantics, Tool, Robot
from compas.data import DataDecoder, json_load
from pybullet_planning import GREY
import pybullet_planning as pp

from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera, wait_if_gui
from compas_fab_pychoreo.client import PyChoreoClient
from npp.tasks import PlanningProblem

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
    # load pybullet planner
    planner = PyChoreoClient(viewer=viewer, verbose=verbose)
    planner.connect()

    # Load robot model and semantics
    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    # Add robot to pybullet
    with LockRenderer():
        robot = planner.load_robot(urdf_filename)
    robot.semantics = semantics

    # Get robot's unique body index in pybullet
    robot_uid = planner.get_robot_pybullet_uid(robot)
    # pp.set_color(robot_uid, (0.5, 0.5, 0.5, 0.1))
    #
    tool0_pose = pp.get_link_pose(
        robot_uid, pp.link_from_name(robot_uid, 'tool0'))
    draw_pose(tool0_pose, length=0.3)

    # Draw base tcp_frame and locate camera in pybullet
    draw_pose(unit_pose(), length=1.)

    return planner, robot, robot_uid


def _tool_model_to_urdf(tool, urdf_package_path):  # type: (ToolModel, str) -> str
    """Export a tool model to a URDF file.
    This is necessary baecause PyBullet reads from a URDF file.
    Returns the path to the URDF file.
    """
    from compas.files import OBJ, URDF

    # Iterate through all links in the tool and export the meshes to obj files
    for link in tool.links:
        link_name = link.name

        # Create folder Visual objects
        os.makedirs(os.path.join(urdf_package_path, tool.name,
                    'meshes', 'visual'), exist_ok=True)
        for i, visual in enumerate(link.visual):
            mesh_descriptor = visual.geometry.shape
            # Change the filename of the visual mesh to be exported
            file_path = [tool.name, 'meshes',
                         'visual', link_name + '_%i.obj' % i]
            mesh_descriptor.filename = 'package://' + '/'.join(file_path)
            # Export the visual mesh to obj file
            obj = OBJ(os.path.join(urdf_package_path, *file_path))
            obj.write(mesh_descriptor.meshes)

        # Create folder Collision objects
        os.makedirs(os.path.join(urdf_package_path, tool.name,
                    'meshes', 'collision'), exist_ok=True)
        for i, visual in enumerate(link.collision):
            mesh_descriptor = visual.geometry.shape
            # Change the filename of the visual mesh to be exported
            file_path = [tool.name, 'meshes',
                         'collision', link_name + '_%i.obj' % i]
            mesh_descriptor.filename = 'package://' + '/'.join(file_path)
            # Export the visual mesh to obj file
            obj = OBJ(os.path.join(urdf_package_path, *file_path))
            obj.write(mesh_descriptor.meshes)

    # Create folder for tool export if it didn't exist
    os.makedirs(os.path.join(urdf_package_path,
                tool.name, 'urdf'), exist_ok=True)
    tool_urdf_path = os.path.join(
        urdf_package_path, tool.name, 'urdf', tool.name + '.urdf')
    tool.to_urdf_file(tool_urdf_path, True)
    return tool_urdf_path


def add_tool_to_client(client, robot, tool, urdf_package_path, touch_links):
    # type: (PyChoreoClient, Robot, Tool, str, list) -> None
    # Add tool definition to pybullet environment
    from compas_fab.robots import AttachedCollisionMesh, CollisionMesh
    tool_urdf_file_path = _tool_model_to_urdf(
        tool.tool_model, urdf_package_path)
    client.add_tool_from_urdf(tool.name, tool_urdf_file_path)

    # Add tool to the robot flange link
    attached_collision_mesh = AttachedCollisionMesh(CollisionMesh(
        None, tool.name), tool.link_name, touch_links=touch_links)
    client.add_attached_collision_mesh(attached_collision_mesh,
                                       options={'robot': robot,
                                                'attached_child_link_name': 'attached_tool_link',
                                                'parent_link_from_child_link_transformation': Transformation(),
                                                })

    # This is to trigger simulator to update the tool position according to the grasp
    # We will remove later as we improved the API
    for attachment in client.pychoreo_attachments[tool.name]:
        attachment.assign()


def load_planning_problem (file_path): # type: (str) -> PlanningProblem
    pp = json_load(file_path) # type: PlanningProblem
    assert type(pp) == PlanningProblem
    return pp