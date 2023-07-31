from compas.data import Data
import os

from compas.robots.configuration import Configuration
from compas.geometry import Frame, Transformation
from compas_fab.robots import JointTrajectory, CollisionMesh, Tool, Trajectory, JointTrajectoryPoint

try:
    from typing import Optional, List, Tuple
except:
    pass


class Task(Data):
    def __init__(self, tag=None):
        super(Task, self).__init__()
        self.task_id = ""  # type: str
        self.execution_result = None
        self.execution_end_time = None
        self.tag = tag or "Generic Task"  # type: str

    @property
    def data(self):
        data = {
            'task_id': self.task_id,
            'execution_result': self.execution_result,
            'execution_end_time': self.execution_end_time,
            'tag': self.tag,
        }
        return data

    @data.setter
    def data(self, data):
        self.task_id = data.get('task_id', "")
        self.execution_result = data.get('execution_result', None)
        self.execution_end_time = data.get('execution_end_time', None)
        self.tag = data.get('tag', "")

    def get_filepath(self, subdir='tasks'):
        # type: (str) -> str
        """ Returns the location of the json file when saved externally.
        This is useful to save a Movement Task containing computed trajectory and has a large file size
        e.g.: 'tasks\A2_M2.json'
        """
        return os.path.join(subdir, "%s.json" % self.task_id)

    @property
    def short_summary(self):
        return '{}(#{}, {})'.format(self.__class__.__name__, self.task_id, self.tag)


class RoboticMovement(Task):
    """ Generic class for movements related to Robot Arm movement.

    `RoboticMovement.allowed_collision_matrix`
    - List of Tuple[object_id, object_id] representing allowable collision between beams and tools

    `RoboticMovement.target_configuration`
    - Optional Robotic Configuration (J / E values) for the target,
    When set, the configuration will become a constraint in the end-state for the path planning.
    When None, the pathplanner should decide for the configuration using IK sampling.
    """

    def __init__(
        self,
        target_frame=None,  # type: Frame # Target of the Robotic Movement
        attached_objects=[],  # type: List[str]
        speed_type="",  # type: str # A string linking to a setting
        target_configuration=None,  # type: Optional[Configuration]
        allowed_collision_matrix=[],  # type: list(tuple(str,str))
        tag=None,  # type: str
        seed=None  # type: int
    ):
        super(RoboticMovement, self).__init__()
        self.target_frame = target_frame  # type: Frame
        self.attached_objects = attached_objects  # type: List[str]
        self.speed_type = speed_type  # type: str # A string linking to a setting
        self.trajectory = None  # type: JointTrajectory
        # type: Optional[Configuration]
        self.target_configuration = target_configuration
        # type: list(tuple(str,str))
        self.allowed_collision_matrix = allowed_collision_matrix
        self.tag = tag or "Generic Robotic Movement"
        self.seed = seed  # or hash(time.time())

    @property
    def data(self):
        """ Sub class specific data added to the dictionary of the parent class
        """
        data = super(RoboticMovement, self).data
        data['target_frame'] = self.target_frame
        data['attached_objects'] = self.attached_objects
        data['trajectory'] = self.trajectory
        data['speed_type'] = self.speed_type
        data['target_configuration'] = self.target_configuration
        data['allowed_collision_matrix'] = self.allowed_collision_matrix
        data['seed'] = self.seed
        return data

    @data.setter
    def data(self, data):
        """ Sub class specific data loaded
        """
        super(RoboticMovement, type(self)).data.fset(self, data)
        self.target_frame = data['target_frame']
        self.attached_objects = data.get('attached_objects', [])
        self.trajectory = data.get('trajectory', None)
        self.speed_type = data.get('speed_type', "")
        self.target_configuration = data.get('target_configuration', None)
        self.allowed_collision_matrix = data.get(
            'allowed_collision_matrix', [])
        self.seed = data.get('seed', None)

    @property
    def short_summary(self):
        return '{}(#{}, {}, has_traj {})'.format(self.__class__.__name__, self.task_id, self.tag,
                                                 self.trajectory is not None)

    def __str__(self):
        if self.target_configuration is None:
            return "Move to %s" % (self.target_frame)
        else:
            return "Move to %s" % (self.target_configuration)


class RoboticFreeMovement(RoboticMovement):

    def __str__(self):
        str = super(RoboticFreeMovement, self).__str__()
        return "Free " + str


class RoboticLinearMovement(RoboticMovement):

    def __str__(self):
        str = super(RoboticLinearMovement, self).__str__()
        return "Linear " + str


class DigitalOutput(Task):
    def __init__(self, tag=None):
        super(DigitalOutput, self).__init__()
        self.tag = tag or "Digital Output"

    def __str__(self):
        return "DigitalOutput " + self.__class__.__name__


class ExtruderOn(DigitalOutput):
    def __init__(self, tag=None):
        super(ExtruderOn, self).__init__()
        self.tag = tag or "Extruder On"


class ExtruderOff(DigitalOutput):
    def __init__(self, tag=None):
        super(ExtruderOff, self).__init__()
        self.tag = tag or "Extruder Off"


class AirAssistOn(DigitalOutput):
    def __init__(self, tag=None):
        super(AirAssistOn, self).__init__()
        self.tag = tag or "Air Assist On"


class AirAssistOff(DigitalOutput):
    def __init__(self, tag=None):
        super(AirAssistOff, self).__init__()
        self.tag = tag or "Air Assist Off"


class PlanningProblem(Data):
    def __init__(self, tag=None):
        super(PlanningProblem, self).__init__()
        self.tasks = ""  # type: list[Task]
        self.start_configuration = ""  # type: Optional[Configuration]
        self.static_collision_meshes = ""  # type: list[CollisionMesh]
        self.robot = None  # type: Optional[Tool]

    @property
    def data(self):
        data = {
            'tasks': self.tasks,
            'start_configuration': self.start_configuration,
            'static_collision_meshes': self.static_collision_meshes,
            'robot': self.robot,
        }
        return data

    @data.setter
    def data(self, data):
        self.tasks = data.get('tasks', [])
        self.start_configuration = data.get('start_configuration', None)
        self.static_collision_meshes = data.get('static_collision_meshes', [])
        self.robot = data.get('robot', None)

    def get_robotic_movements(self):
        # type: () -> List[RoboticMovement]
        """ Returns all the Robotic Movements in the planning problem
        """
        return [t for t in self.tasks if isinstance(t, RoboticMovement)]

    def renumber_task_ids(self):
        # type: () -> None
        """ Renumber the movement ids in the planning problem
        """
        for i, t in enumerate(self.tasks):
            t.task_id = "t%i" % i

    def get_linear_movement_groups(self):
        # type: () -> List[List[RoboticLinearMovement]]
        """ Returns a list of list of RoboticLinearMovement
        """
        groups = []
        for t in self.get_robotic_movements():
            if isinstance(t, RoboticLinearMovement):
                if len(groups) == 0:
                    groups.append([])
                groups[-1].append(t)
            else:
                if len(groups) > 0:
                    if len(groups[-1]) > 0:
                        groups.append([])
        # Remove empty groups
        if len(groups) > 0:
            if groups[-1] == []:
                groups.pop()
        return groups
