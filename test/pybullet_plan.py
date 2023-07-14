from compas.robots import RobotModel, Configuration, ToolModel
from compas_fab.robots import RobotSemantics
from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera
from compas_fab_pychoreo.client import PyChoreoClient
from compas.geometry import Plane, Frame, Point, Vector, Transformation, Translation
from compas.data import DataDecoder 
import os

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


def load_planning_client(urdf_filename, srdf_filename, viewer=True, verbose=False):

    model = RobotModel.from_urdf_file(urdf_filename)
    semantics = RobotSemantics.from_srdf_file(srdf_filename, model)

    client = PyChoreoClient(viewer=viewer, verbose=verbose)
    client.connect()

    with LockRenderer():
        robot = client.load_robot(urdf_filename)
    robot.semantics = semantics
    
    # robot's unique body index in pybullet
    robot_uid = client.get_robot_pybullet_uid(robot)

    # * draw base tcp_frame and locate camera in pybullet
    draw_pose(unit_pose(), length=1.)
    # cam_info = pp.get_camera()
    # cam_info = rfl_camera()
    # set_camera(cam_info.yaw, cam_info.pitch, cam_info.dist, cam_info.target)

    return client, robot, robot_uid

def load_toolmodel(toolmodel_path):
    import json
    from compas.robots import ToolModel

    with open(toolmodel_path, 'r') as f:
        tool = json.load(f, cls=DataDecoder) # type:ToolModel
    return tool

def load_planes (design_path, scale = None):
    import json
    # Load Design Frames
    with open(design_path, 'r') as f:
        planes = json.load(f, cls=DataDecoder)

    if scale is not None:
        for plane in planes:
            plane.point *= scale
    
    return planes


# URDF export/import
# __________________

def get_urdf_path(self, save_dir):
    package_name = self.name or str(self.guid)
    urdf_subdir = os.path.join(package_name, 'urdf')
    robot_file_name = package_name + '.urdf'
    robot_filepath = os.path.join(save_dir, urdf_subdir, robot_file_name)
    return robot_filepath

def tool_to_urdf(tool, save_dir, scale=1.0, triangulize=True):
    from compas.robots import MeshDescriptor
    from compas.datastructures.mesh.triangulation import mesh_quads_to_triangles
    from compas.files import URDF
    import os
    def _export_element(element, save_dir, mesh_subdir, mesh_name, address_dict, triangulize=True):
        shape = element.geometry.shape
        if isinstance(shape, MeshDescriptor):
            mesh = shape.geometry.copy()
            if triangulize:
                mesh_quads_to_triangles(mesh)
            assert mesh_name not in address_dict
            shape.filename = mesh_name  # str(mesh.guid)
            try:
                sub_path = os.path.join(mesh_subdir, mesh_name + '.stl')
                mesh.to_stl(os.path.join(save_dir, sub_path), binary=True)
            except:
                sub_path = os.path.join(mesh_subdir, mesh_name + '.obj')
                mesh.to_obj(os.path.join(save_dir, sub_path))
            address_dict[mesh_name] = 'package://' + sub_path.replace('\\', '/')
        return address_dict

    # modified from: https://github.com/compas-dev/compas_fab/blob/6e68dbd7440fa68a58606bb9100495583bc79980/src/compas_fab/backends/pybullet/client.py#L235
    package_name = tool.name or str(tool.guid)
    tool.ensure_geometry()
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    visual_mesh_subdir = os.path.join(package_name, 'meshes', 'visual')
    collision_mesh_subdir = os.path.join(package_name, 'meshes', 'collision')
    urdf_subdir = os.path.join(package_name, 'urdf')
    os.makedirs(os.path.join(save_dir, visual_mesh_subdir), exist_ok=True)
    os.makedirs(os.path.join(save_dir, collision_mesh_subdir), exist_ok=True)
    os.makedirs(os.path.join(save_dir, urdf_subdir), exist_ok=True)

    # TODO this alter the object!
    tool.scale(scale)
    # * write meshes to cache
    address_dict = {}
    mesh_scale = '{} {} {}'.format(scale, scale, scale)
    for link_id, link in enumerate(tool.links):
        for v_eid, element in enumerate(link.visual):
            _export_element(element, save_dir, visual_mesh_subdir, 'L{}_visual_{}'.format(link_id, v_eid),
                                    address_dict, triangulize)
        for c_eid, element in enumerate(link.collision):
            _export_element(element, save_dir, collision_mesh_subdir, 'L{}_collision_{}'.format(link_id, c_eid),
                                    address_dict, triangulize)

    # * create urdf with new mesh locations
    urdf = URDF.from_robot(tool)
    meshes = list(urdf.xml.root.iter('mesh'))
    for mesh in meshes:
        filename = mesh.attrib['filename']
        # TODO why sometimes filename is ''?
        if filename in address_dict:
            mesh.attrib['filename'] = address_dict[filename]
            mesh.attrib['scale'] = mesh_scale
    # write urdf
    robot_file_name = package_name + '.urdf'
    urdf.to_file(tool.get_urdf_path(save_dir), prettify=True)

def tool_to_urdf_new(tool, urdf_package_path): #type: (ToolModel, str) -> str
    """Export a tool model to a URDF file.
    Returns the path to the URDF file.
    """
    from compas.files import OBJ, URDF
    
    # Iterate through all links in the tool and export the meshes to obj files
    for link in tool.links:
        link_name = link.name
        
        # Create folder Visual objects
        os.makedirs(os.path.join(urdf_package_path, tool.name, 'meshes', 'visual'),exist_ok=True)
        for i, visual in enumerate(link.visual):
            mesh_descriptor = visual.geometry.shape
            # Change the filename of the visual mesh to be exported
            file_path = [tool.name, 'meshes', 'visual', link_name + '_%i.obj' % i]
            mesh_descriptor.filename = 'package://' + '/'.join(file_path)
            # Export the visual mesh to obj file
            obj = OBJ(os.path.join(urdf_package_path, *file_path))
            obj.write(mesh_descriptor.meshes)

        # Create folder Collision objects
        os.makedirs(os.path.join(urdf_package_path, tool.name, 'meshes', 'collision'),exist_ok=True)
        for i, visual in enumerate(link.collision):
            mesh_descriptor = visual.geometry.shape
            # Change the filename of the visual mesh to be exported
            file_path = [tool.name, 'meshes', 'collision', link_name + '_%i.obj' % i]
            mesh_descriptor.filename = 'package://' + '/'.join(file_path)
            # Export the visual mesh to obj file
            obj = OBJ(os.path.join(urdf_package_path, *file_path))
            obj.write(mesh_descriptor.meshes)

    # Create folder for tool export if it didn't exist
    os.makedirs(os.path.join(urdf_package_path, tool.name, 'urdf'), exist_ok=True)
    urdf = URDF.from_robot(tool)
    tool_urdf_path = os.path.join(urdf_package_path, tool.name, 'urdf', tool.name + '.urdf')
    tool.to_urdf_file(tool_urdf_path, True)
    return tool_urdf_path


def add_tool_to_robot(tool, toolmodel_path):
    from compas_fab.robots import AttachedCollisionMesh, CollisionMesh

    urdf_package_path = os.path.dirname(toolmodel_path)
    tool_urdf_file_path = tool_to_urdf_new(tool, urdf_package_path)
    
    client.add_tool_from_urdf(tool.name , tool_urdf_file_path)
    client.add_attached_collision_mesh(
                            AttachedCollisionMesh(CollisionMesh(None, tool.name),
                                                'tool0', touch_links=['tool0', 'flange', 'link_6']),
                            options={'robot': robot,
                                    'attached_child_link_name': tool.get_base_link_name(),
                                    'parent_link_from_child_link_transformation' : Transformation.from_frame(Frame.worldXY()),
                                    })

# Planning
# __________________

def random_ik (client, robot, frame, starting_config= None, tries = 10, collision = True, visualize = True):
    """ pybullet IK wrapper, provide random IK solution for a given frame.
    If starting_config is provided, the random IK will start from that configuration.
    If tries is more than 1, the random IK will try multiple times with randomized starting configuration."""
    with LockRenderer(not visualize):
        configuration = starting_config
        for i in range(tries):
            if configuration is None:
                configuration = robot.random_configuration()
            client.set_robot_configuration(robot, configuration)
            configuration = client.inverse_kinematics(
                robot, frame, group=None, options={'avoid_collisions': collision})
            
            # configuration will be None if planning failed.
            if configuration is not None:
                return configuration
    return None

def plan_one_stroke(robot, client, planes, draw=False):
    configurations = []

    if not draw:
        lock = LockRenderer()

    for plane in planes:
        # * pybullet gradient-based IK
        
        conf = client.inverse_kinematics(
            robot, plane, group=None, options={'avoid_collisions': False})
        configurations.append(conf)

    if not draw:
        lock.restore()

    return configurations

def rotate_frame(tcp_frame : Frame, angle):
    from compas.geometry.transformations.matrices import matrix_from_axis_and_angle
    from compas.geometry import Rotation
    r = Rotation.from_axis_and_angle(tcp_frame.zaxis, angle, tcp_frame.point)
    return tcp_frame.transformed(r)

def create_rotated_frames_by_steps(tcp_frame, num_steps = 36):
    import math
    rotation_angle = 360  # Degrees
    rotation_step = rotation_angle / num_steps
    tcp_frames = []
    for i in range(num_steps):
        rotation = rotation_step * i
        rotation_rad = math.radians(rotation)
        rotated_frame = rotate_frame(tcp_frame, rotation_rad)
        tcp_frames.append(rotated_frame)
    return tcp_frames


client, robot, robot_uid = load_planning_client(urdf_filename, srdf_filename, viewer=False)

# Load Planes and convert them to tcp_frames
planes = load_planes(design_path, scale=1e-3)
tcp_frames = [Frame(p.point, Vector(0.0, 1.0, 0.0), Vector(1.0, 0.0, 0.0)) for p in planes]

# Move the design in space
# T = Translation.from_vector([0.2,0,-0.5])
T = Translation.from_vector([0.2,0,0.0])
tcp_frames = [f.transformed(T) for f in tcp_frames]

# Load Tool Model
tool = load_toolmodel(toolmodel_path)
add_tool_to_robot(tool, toolmodel_path)

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

