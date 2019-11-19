import cozmo
import threading
import json
import math
import sys
import time
import numpy as np


def create_config_file(values):
    with open("./config.json", "w") as file:
        json.dump(values, file)


async def CozmoTuning(robot: cozmo.robot.Robot):
    global stopevent
    tuning_values = {"kp": 0, "ki": 0, "kd": 0}
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    tuning_values["kp"] = 1.5
    tuning_values["ki"] = 0.5
    tuning_values["kd"] = 0.1
    ###############################
    
    create_config_file(tuning_values)


class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoTuning)
        stopevent.set()


if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
