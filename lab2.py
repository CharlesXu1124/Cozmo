# state machines

import time
import tensorflow as tf
import cozmo
from joblib import load
from cozmo.util import degrees, distance_mm, speed_mmps
import sys
import cv2
import numpy as np
import imutils
from keras.preprocessing.image import img_to_array
import statistics
from PIL import Image



def rescale(image, width, height):
    # Grab the dimensions of the image, then initialize the padding values
    (h, w) = image.shape[:2]

    # If the width is greater than the height then resize along the width
    if w > h:
        image = imutils.resize(image, width=width)
    # Otherwise, the height is greater than the width so resize along the height
    else:
        image = imutils.resize(image, height=height)

    # Determine the padding values for the width and height to obtain the target dimensions
    pad_w = int((width - image.shape[1]) / 2.0)
    pad_h = int((height - image.shape[0]) / 2.0)

    # Pad the image then apply one more resizing to handle any rounding issues
    image = cv2.copyMakeBorder(image, pad_h, pad_h, pad_w, pad_w, cv2.BORDER_REPLICATE)
    image = cv2.resize(image, (width, height))

    # Return the pre-processed image
    return image


def idle(robot: cozmo.robot.Robot):
    # code for state idle
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    # reset head angle
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    # reuse the model from lab 1
    classifier = load('clf.joblib')
    img_window = ['none', 'none', 'none', 'none', 'none']
    label_arr = ['drone', 'hands', 'inspection', 'none', 'order', 'place', 'plane', 'truck']
    state_idle = True
    # pause 0.5 sec
    while (state_idle):
        time.sleep(0.5)
        latest_image = robot.world.latest_image
        new_image = latest_image.raw_image
        new_image = np.array(new_image)
        image = rescale(new_image,32,32)
        image = img_to_array(image)
        img = np.array(image, dtype="float") / 255.0
        img = img.reshape(-1, 32, 32, 3)
        result = classifier.predict(img)
        result = result[0].tolist()
        idx = result.index(max(result))
        label = label_arr[idx]
        for i in range(1, len(img_window)):
            img_window[i] = img_window[i - 1]
        img_window[0] = label
        majority_label = statistics.mode(img_window)
        print(majority_label)
        if majority_label == 'drone':
            img_window = ['none', 'none', 'none', 'none', 'none']
            robot.say_text('drone').wait_for_completed()
            robot.abort_all_actions(log_abort_messages=True)
            drone(robot)
        if majority_label == 'order':
            img_window = ['none', 'none', 'none', 'none', 'none']
            robot.say_text('order').wait_for_completed()
            robot.abort_all_actions(log_abort_messages=True)
            order(robot)
        if majority_label == 'inspection':
            img_window = ['none', 'none', 'none', 'none', 'none']
            robot.say_text('inspection').wait_for_completed()
            robot.abort_all_actions(log_abort_messages=True)
            inspection(robot)
    idle(robot)
        


def inspection(robot: cozmo.robot.Robot):
    # Use a "for loop" to repeat the indented code 4 times
    # Note: the _ variable name can be used when you don't need the value
    # drive in a square with side 20cm
    robot.set_lift_height(0.0, in_parallel=True, duration=1.0).wait_for_completed()
    robot.abort_all_actions(log_abort_messages=True)
    for i in range(4):
        if i % 2 == 1:
            robot.move_lift(-0.25)
        else:
            robot.move_lift(0.25)
        robot.drive_straight(distance_mm(200), speed_mmps(50)).wait_for_completed()
        robot.turn_in_place(degrees(90)).wait_for_completed()
    robot.abort_all_actions(log_abort_messages=True)
    robot.set_lift_height(0.0, in_parallel=True, duration=1.0)
    idle(robot)



def drone(robot: cozmo.robot.Robot):
    # code for state drone
    lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
    cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
    lookaround.stop()
    current_action = robot.pickup_object(cubes[0], num_retries=5)
    current_action.wait_for_completed()
    # move forward 100mm
    robot.drive_straight(distance_mm(100), speed_mmps(50)).wait_for_completed()
    # drop the cube
    current_action = robot.place_object_on_ground_here(cubes[0]).wait_for_completed()
    # go back 100mm
    robot.drive_straight(distance_mm(-100), speed_mmps(50)).wait_for_completed()
    robot.abort_all_actions(log_abort_messages=True)
    idle(robot)


    
def order(robot: cozmo.robot.Robot):
    # code for state order
    # drive in a circle with radius of approximately 10 cm
    robot.drive_wheels(80, 40)
    time.sleep(13)
    # stop driving after 13 sec
    robot.drive_wheels(0, 0)
    robot.abort_all_actions(log_abort_messages=True)
    idle(robot)


cozmo.run_program(idle)
