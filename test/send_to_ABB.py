# This file requires compas_rrc for communication with the ABB robot.
import compas_rrc as rrc
import os
import json
import time
from compas.geometry import Frame, Point, Vector

PATH = os.path.dirname(os.path.abspath(__file__))
FILE_NAME = 'planes.json'


def load_data(PATH, FILE_NAME, x_offset, y_offset, z_offset):
    with open(os.path.join(PATH, FILE_NAME), 'r') as f:
        data = json.load(f)
        print("Loaded json: '" + os.path.join(PATH, FILE_NAME) + "'")

    if len(data) != 0:
        frames = []
        for i in range(len(data)):
            d = data[str(i)]
            frames.append(Frame(point=Point(d["point"][0] + x_offset, d["point"][1] + y_offset, d["point"][2] + z_offset),
                                xaxis=Vector(
                                    d["xaxis"][0], d["xaxis"][1], d["xaxis"][2]),
                                yaxis=Vector(d["yaxis"][0], d["yaxis"][1], d["yaxis"][2])))
        return frames


x_offset = 0.0
y_offset = 0.0
z_offset = 0.0

travel_speed = 150
print_speed = 50

start_pos = [5.23, -14.86, 0.12, -2.18, 69.80, -176.39]


def print_process():

    frames = load_data(PATH, FILE_NAME, x_offset, y_offset, z_offset)

    # --- Create Ros Client + ABB Client, and connect
    ros = rrc.RosClient()
    ros.run()
    abb = rrc.AbbClient(ros, '/rob1')
    print('Initialized RosClient and AbbClient.')

    print('Going to start position')
    abb.send_and_wait(rrc.MoveToJoints(
        start_pos, [], travel_speed, rrc.Zone.FINE))

    # --- Set tool
    abb.send_and_wait(rrc.SetTool('ottoshand'))

    # --- Set workobject
    abb.send_and_wait(rrc.SetWorkObject('wobj0'))

    # --- Turn on tool
    # extr = rrc.SetDigital('doUnitC141Out01', 0)

    start_i = 0
    end_i = 1e10

    print('Sending print frames to robot')
    results = []
    for i in range(len(frames)):
        if start_i <= i <= end_i:

            # send robot to position
            instruction = rrc.MoveToFrame(
                frames[i], print_speed, zone=rrc.Zone.Z1, motion_type=rrc.Motion.LINEAR)
            instruction.feedback_level = 1
            result = abb.send(instruction)
            results.append(result)
            abb.send(rrc.SetDigital('doUnitC141Out1', 0))

    # --- get feedback from robot, so we know when each command has been executed
    for i, result in enumerate(results):
        print('\nPoint : %d out of %d' % (i + start_i, len(frames)))
        while not result.done:
            time.sleep(0.01)

    # --- turn off tool
    # extr = rrc.SetDigital('doUnitC141Out01', 1)
    abb.send(rrc.SetDigital('doUnitC141Out1', 1))

    print('Going to end position')
    abb.send_and_wait(rrc.MoveToJoints(
        start_pos, [], travel_speed, rrc.Zone.FINE))

    abb.send_and_wait(rrc.PrintText('Finished'))
    print('Finished')

    # --- Close client
    ros.close()
    ros.terminate()


if __name__ == '__main__':
    print_process()
