import cozmo
import threading
import json
import math
import sys
import time
import numpy as np
import time


async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    # look around to find any cubes present
    cube = None
    cubefound = False
    rt = 0
    et = 0
    yt = 0 # the desired distance you want to travel
    ut = 0
    at = 0
    goal_reached = False
    while not cubefound:
        await robot.world.wait_for_observed_light_cube(timeout=30)
        for obj in robot.world.visible_objects:
            if obj is not None:
                cube = obj
                print("cube found!")
                cubefound = True
                
    rt = cube.pose.position.x - robot.pose.position.x - 125
    while not goal_reached:
        et = rt - yt
        ut = kp*et + ki*yt + kd*(-ut)
        robot.drive_wheel_motors(ut, ut)
        time.sleep(0.1)
        yt += 0.1*ut
        if et < 5.0:
            break
    
            
            
            

    
    
    ###############################


class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoPID)
        stopevent.set()


if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
