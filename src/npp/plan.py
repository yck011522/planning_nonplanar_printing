import math

from compas.geometry import (Frame, Plane, Point, Transformation, Translation,
                             Vector)
from compas_fab.robots import CollisionMesh, Robot, Tool
from compas_fab_pychoreo.client import PyChoreoClient
from pybullet_planning import (LockRenderer, draw_pose, load_pybullet,
                               set_camera, set_camera_pose, unit_pose)
from compas.robots import Configuration

from npp.tasks import PlanningProblem, RoboticLinearMovement, RoboticMovement

try:
    from typing import List, Optional, Tuple
except:
    pass

def random_ik(client, robot, frame, starting_config=None, tries=10, collision=True, visualize=True, return_all=False):
    """ pybullet IK wrapper, provide random IK solution for a given frame.
    If starting_config is provided, the random IK will start from that configuration.
    If tries is more than 1, the random IK will try multiple times with randomized starting configuration."""
    with LockRenderer(not visualize):
        configuration = starting_config
        if configuration is None:
            configuration = robot.random_configuration()
        client.set_robot_configuration(robot, configuration)
        configuration = client.inverse_kinematics(
            robot, frame, group=None, options={
                'avoid_collisions': collision,
                'attempts': tries,
                'return_all': return_all,
            })

        # configuration will be None if planning failed.
        if configuration is not None:
            return configuration
    return None


def rotate_frame(tcp_frame, angle):  # type: (Frame, float) -> Frame
    from compas.geometry import Rotation
    from compas.geometry.transformations.matrices import \
        matrix_from_axis_and_angle
    r = Rotation.from_axis_and_angle(tcp_frame.zaxis, angle, tcp_frame.point)
    return tcp_frame.transformed(r)


def create_rotated_frames_by_steps(tcp_frame, num_steps=36):
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


def check_pp_ik(client, robot, tool, pp, plane_rotation_angle=None, diagnose=False):
    # type: (PyChoreoClient, Robot, Tool, PlanningProblem, Optional[float], bool) -> None
    """ Check IK for all movements in a Planning Problem
    The collision geometry should be set to the client before calling this function
    If plane_rotation_angle is provided, the target frame will be rotated by that angle
    If diagnose is True, the collision diagnosis will be printed

    Example:
    >>> total_count, reachable_count, collisionfree_count = check_pp_ik(
            client, robot, tool, pp, plane_rotation_angle=None, diagnose=False)
    >>> print("%i Movements, %i Reachable, %i CollisionFree" %
            (total_count, reachable_count, collisionfree_count))
    """

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

def check_pp_with_global_ik_rotation(client, robot, tool, pp, plane_rotation_steps=36, diagnose=False):
    # type: (PyChoreoClient, Robot, Tool, PlanningProblem, Optional[int], bool) -> None
    """ Check IK for all movements in a Planning Problem
    Similar to check_pp_ik, but the target frame will be rotated in many steps and checked for their reachability and collision
    See check_pp_ik for more details
    """
    possible_angles = []
    for i in range(plane_rotation_steps):
        angle = 360 / plane_rotation_steps * i
        angle_rad = math.radians(angle)
        total_count, reachable_count, collisionfree_count = check_pp_ik(
            client, robot, tool, pp, plane_rotation_angle=angle_rad, diagnose=False)
        print("Rotation(%.1f degrees) : %i Movements, %i Reachable, %i CollisionFree" % (
            angle, total_count, reachable_count, collisionfree_count))
        if total_count == collisionfree_count:
            possible_angles.append(angle)
    return possible_angles


def test_all_angles_for_a_frame(client, robot, tool, robotic_movement, starting_config, tries=1, plane_rotation_steps=36, visualize=False):
    # type: (PyChoreoClient, Robot, Tool, RoboticMovement, Configuration, int, Optional[int], Optional[bool]) -> Tuple[List[float], List[Configuration]]
    """ Test all angles for a given robotic movement, return a list of possible angles and planned configurations"""
    possible_angles = []
    planned_configurations = []
    with LockRenderer(not visualize):
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
