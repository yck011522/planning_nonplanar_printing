# This file requires compas_rrc to communicate to the ABB robot
import compas_rrc as rrc
import os
import json
import time
from compas.geometry import Frame, Point, Vector
from compas.data import json_load, json_dump
from compas_fab.robots import to_degrees

from npp.tasks import PlanningProblem, Task, RoboticFreeMovement, RoboticLinearMovement, AirAssistOn, AirAssistOff, ExtruderOn, ExtruderOff

PATH = os.path.dirname(os.path.abspath(__file__))
FILE_NAME = 'test/design/planning_result_230808_DefTest_V1.json'


x_offset = 0.0
y_offset = 0.0
z_offset = 0.0

travel_speed = 100
print_speed = 50

start_pos = [5.23, 17.86, 0.12, -2.18, 69.80, -176.39]


def execute_robotic_free_movement(abb, task):
    movement = task  # type: RoboticFreeMovement
    trajectory = movement.trajectory
    sent_instructions = []
    for trajectory_point in trajectory:
        joint_values = to_degrees(trajectory_point.joint_values)
        # send robot to position
        instruction = rrc.MoveToJoints(
            joint_values, [], print_speed, zone=rrc.Zone.Z1, feedback_level=1)
        result = abb.send(instruction)
        sent_instructions.append(result)
    task.execution_result = sent_instructions


def execute_robotic_linear_movement(abb, task):
    return execute_robotic_free_movement(abb, task)


def execute_air_assist_on(abb, task):
    instruction = rrc.SetDigital('doUnitC141Out2', 1, feedback_level=1)
    result = abb.send(instruction)
    task.execution_result = [result]


def execute_air_assist_off(abb, task):
    instruction = rrc.SetDigital('doUnitC141Out2', 0, feedback_level=1)
    result = abb.send(instruction)
    task.execution_result = [result]


def execute_extruder_on(abb, task):
    instruction = rrc.SetDigital('doUnitC141Out1', 1, feedback_level=1)
    result = abb.send(instruction)
    task.execution_result = [result]


def execute_extruder_off(abb, task):
    instruction = rrc.SetDigital('doUnitC141Out1', 0, feedback_level=1)
    result = abb.send(instruction)
    task.execution_result = [result]


def print_process():
    planning_result = json_load(os.path.join(
        PATH, FILE_NAME))  # type: PlanningProblem

    # --- Create Ros Client + ABB Client, and connect
    ros = rrc.RosClient()
    ros.run()
    abb = rrc.AbbClient(ros, '/rob1')
    print('Initialized RosClient and AbbClient.')

    # --- Set tool
    abb.send_and_wait(rrc.SetTool('BioPrint901'))

    # --- Set workobject
    abb.send_and_wait(rrc.SetWorkObject('wobj0'))

    # --- Print start message
    print('Sending print Joints to robot')
    abb.send_and_wait(rrc.PrintText('Sending print Joints to robot'))

    tasks = planning_result.tasks
    sent_instructions = []
    for i in range(len(tasks)):
        task = tasks[i]

        if type(task) == RoboticFreeMovement:
            if task.trajectory is None:
                print("Warning: Task %s not Planned." % (task.tag))
                continue
            execute_robotic_free_movement(abb, task)
        elif type(task) == RoboticLinearMovement:
            if task.trajectory is None:
                print("Warning: Task %s not Planned." % (task.tag))
                continue
            execute_robotic_linear_movement(abb, task)
        elif type(task) == AirAssistOn:
            execute_air_assist_on(abb, task)
        elif type(task) == AirAssistOff:
            execute_air_assist_off(abb, task)
        elif type(task) == ExtruderOn:
            execute_extruder_on(abb, task)
        elif type(task) == ExtruderOff:
            execute_extruder_off(abb, task)

    # --- get feedback from robot, so we know when each command has been executed
    for i, task in enumerate(tasks):
        if task.execution_result is None:
            print("Warning: Task (%d of %d): %s not Planned therefore not Executed." % (
                i, len(tasks), task.tag))
            continue
        while not all([result.done for result in task.execution_result]):
            time.sleep(0.01)
        print('Executed Task (%d of %d): %s' % (i, len(tasks), task.tag))

    abb.send_and_wait(rrc.PrintText('Finished'))
    print('Finished')

    # --- Close client
    ros.close()
    ros.terminate()


if __name__ == '__main__':
    print_process()
