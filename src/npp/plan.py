from pybullet_planning import draw_pose, set_camera_pose, load_pybullet, unit_pose, LockRenderer, set_camera
from compas.geometry import Plane, Frame, Point, Vector, Transformation, Translation


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
    from compas.geometry.transformations.matrices import matrix_from_axis_and_angle
    from compas.geometry import Rotation
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
