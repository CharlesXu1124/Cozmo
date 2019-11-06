import cozmo
import math
import sys
import time
import random
from cozmo.util import degrees, distance_mm, speed_mmps

from cmap import *
from gui import *
from utils import *

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    distance = get_dist(node0, node1)
    if distance >= limit:
        # things get tricky here: the limit is the 2nd norm, but the ratio of distance / limit stays the same
        x = node0.x + (node1.x - node0.x) * limit / distance
        y = node0.y + (node1.y - node0.y) * limit / distance
        node1 = Node((x, y))
    return node1
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    if random.random() < 0.05:
        goal = cmap.get_goals()[0]
        rand_node = Node((goal.x, goal.y))

    # loop until rand_node is:
    # 1. not null
    # 2. not inside any obstacles
    # 3. outside the boundary of the map
    # until it finds a legitimate location for randomly generated node
    while rand_node == None or cmap.is_inside_obstacles(rand_node) or (not cmap.is_inbound(rand_node)):
        w = random.random() * cmap.width
        h = random.random() * cmap.height
        rand_node = Node((w, h))
    
    ############################################################################
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nearest_dist = 1e7
        nearest_node = None
        # loop through all the surrounding nodes to find the nearest one
        for n in cmap.get_nodes():
            if get_dist(rand_node, n) < nearest_dist:
                nearest_node = n
                nearest_dist = get_dist(rand_node, n)
        rand_node = step_from_to(nearest_node, rand_node)
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")

def get_current_pose_cmap(robot):
    startX = 50.0
    startY = 35.0
    currentx = robot.pose.position.x
    currenty = robot.pose.position.y

    return Node((currentx + startX, currenty + startY))


async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    #the start position of the robot
    startX = 35
    startY = 50
    angle = 0
    # marker_list initialized to empty
    # it is a dictionary
    marked = {}
    # initialize the heading to 0 degree
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    globalWidth, globalHeight = cmap.get_size()
    # initialize path
    path = None
    index = 0
    robot_pos = Node((50 + robot.pose.position.x, 35 + robot.pose.position.y))
    cmap.set_start(robot_pos)
    while True:
        index += 1
        print(index)
        robot_pos = Node((50 + robot.pose.position.x, 35 + robot.pose.position.y))
        cmap.set_start(robot_pos)
        # get three parameters
        # update_cmap: whether the cmap is updated
        # goal_center: whether there is a valid goal position
        # marked: whether the 
        update_cmap, goal_center, marked = await detect_cube_and_update_cmap(robot, marked, robot_pos)
        print(update_cmap)
        print(goal_center)
        print(marked)
        print(path)
        if update_cmap:
            # update cmap
            print("map changed, resetting...")
            # clear all existing paths found
            if not path is None:
                cmap.reset_paths()
            print("resetted")
            
                
            
        # if the cmap is solved using RRT
        if cmap.is_solved():
            
            # begin routing
            for i in range(len(path)):
                p = path[i]
                p_next = path[i+1]
                delta_x = p_next.x - p.x
                delta_y = p_next.y - p.y
                angle = math.atan2(delta_y, delta_x) * 57.2958
                # calculate distance for driving
                dist = math.sqrt(delta_x**2 + delta_y**2)
                # reset angle before performing turning
                await robot.turn_in_place(degrees(0)).wait_for_completed()
                # performing the turn
                await robot.turn_in_place(degrees(0)).wait_for_completed()
                # call detecting function again in case there are some obstacles in front of the robot
                update_cmap, goal_center, marked = await detect_cube_and_update_cmap(robot, marked, robot_pos)
                if update_cmap:
                    cmap.reset_paths()
                    # proceed to next loop
                    continue
                # drive straight
                await robot.drive_straight(distance_mm(dist), speed_mmps(50)).wait_for_completed()
            # reset angle to 0 after going through all path
            await robot.turn_in_place(degrees(0)).wait_for_completed()    
        else:
            # if the cmap is not solved
            goal_list = cmap.get_goals()
            print(goal_list)
            goal_num = len(cmap.get_goals())
            if len(goal_list) == 0 and goal_num == 0:
                # no goals found!
                if index == 1:
                    
                    # drive to center first
                    dx = globalWidth / 2 - startX
                    dy = globalHeight / 2 - startY
                    distance = math.sqrt(dx**2 + dy**2)
                    alpha = math.atan2(dy, dx) * 57.2958
                    print(alpha)
                    await robot.turn_in_place(degrees(alpha)).wait_for_completed()
                    await robot.drive_straight(distance_mm(distance), speed_mmps(50)).wait_for_completed()
                    await robot.set_head_angle(degrees(0)).wait_for_completed()
                else:
                    # if i > 0
                    # turn at the center to observe new obstacles and goals
                    await robot.turn_in_place(degrees(30)).wait_for_completed()
            else:
                cmap.set_start(robot_pos)
                RRT(cmap, cmap.get_start())
                if cmap.is_solved():
                    path = cmap.get_smooth_path()
                index = 0
    
                
    

    


def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    # TODO: please enter your code below.
    new_node = None
    x = node.x
    y = node.y
    new_x = (x * math.cos(local_angle)) - (y * math.sin(local_angle)) + local_origin.x
    new_y = (y * math.cos(local_angle)) + (x * math.sin(local_angle)) + local_origin.y
    new_node = Node((new_x, new_y))
    return new_node


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """
    global cmap

    # Padding of objects and the robot for C-Space
    cube_padding = 40.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue

        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:

            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos

        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center, marked


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("maps/map6.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
