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
    # initialize the variables and objects in memory
    cube = None
    cubefound = False
    rt = 0
    et = 0
    yt = 0 # the desired distance you want to travel
    ut = 0
    while not cubefound:
        await robot.world.wait_for_observed_light_cube(timeout=30)
        for obj in robot.world.visible_objects:
            if obj is not None:
                cube = obj
                # print in cmd to indicate a cube found
                print("cube found!")
                # print the distance of the cube in front of
                # the robot
                print(cube.pose.position.x)
                # prepare to break the while loop
                cubefound = True
                # break the for loop to get rid of duplicates
                break
    # calculate rt, the desired distance to travel
    rt = cube.pose.position.x - robot.pose.position.x - 125 + 5
    # print rt in case something went wrong
    print(rt)
    '''
    PID control loop
    '''
    while True:
        # calculate the error associated with time t
        print(et)
        et = rt - yt
        # update the ut using PID
        ut = kp*et + ki*yt + kd*(-ut)
        # inject the input to the wheel motors
        robot.drive_wheel_motors(ut, ut)
        # travel for about 0.1 second
        time.sleep(0.1)
        # update yt after setting new speed
        yt += 0.1*ut
        # breaking condition for debugging, not needed
        '''
        if et < 1.0:
            break
        '''
    
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
