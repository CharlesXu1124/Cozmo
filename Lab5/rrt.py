import cozmo
import math
import sys
import time

from cmap import *
from gui import *
from utils import *
import random

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
    if get_dist(node0,node1) < limit:
        return node1
    else:
        limit_distance = (limit / get_dist(node0,node1))
        nodex = node0.x * (1 - limit_distance) + node1.x * limit_distance
        nodey = node0.y * (1 - limit_distance) + node1.y * limit_distance
        node = Node((nodex,nodey))
        return node
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object

    #5% change to get to the goal
    if random.random() < 0.05:
        goal_node = cmap.get_goals()[0]
        return Node((goal_node.x, goal_node.y))

    while rand_node == None or (not cmap.is_inbound(rand_node)) or cmap.is_inside_obstacles(rand_node):
        rand_node = Node((random.random() * cmap.width, random.random() * cmap.height))
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
        rand_node = None
        nearest_node = None



        rand_node = cmap.get_random_valid_node()
        dist = 0
        for curnode in cmap.get_nodes():
            curdist = get_dist(rand_node, curnode)
            if nearest_node is None or curdist < dist:
                dist = curdist
                nearest_node = curnode
        # print(type(rand_node))
        # print(rand_node)
        # print(type(nearest_node))
        # print(nearest_node)
        node = step_from_to(nearest_node,rand_node)
        # print(type(node))
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, node)
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


async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions

    #the start position of the robot
    xstart = 50
    ystart = 35
    angle = 0.0
    cozmo_angle = 0.0
    marklist = {}
    #set the angle back to 0 degree
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    #the width and height of the map
    widthofmap, heightofmap = cmap.get_size()
    counter = 0
    #the main loop of the robot running
    finding_path = True
    #the index of the path node
    index = 0
    while finding_path:
        counter += 1

        curpos = Node((50 + robot.pose.position.x, 35 + robot.pose.position.y))
        #let the cozmo to start at current position
        cmap.set_start(curpos)
        mapchangeflag, goal_center, _ = await detect_cube_and_update_cmap(robot, marklist, curpos)
        #if we change the map,ex:add new obstacles, we need to recompute the path
        if mapchangeflag:
            cmap.reset_paths()
        #go to the goal if cmap is solved
        if cmap.is_solved():
           #keep going until we reach the last node
            if index < len(path)-1:
                curnode = path[index]
                nextnode = path[index + 1]
                #calculate the angle
                offsetx = nextnode.x - curnode.x
                offsety = nextnode.y - curnode.y
                angle = math.atan2(offsety,offsetx)
                positionoffsetx = nextnode.x - xstart
                positionoffsety = nextnode.y - ystart
                positionangle = cozmo.util.Angle(angle)
                position = cozmo.util.Pose(positionoffsetx,positionoffsety,0,angle_z = positionangle)
                await robot.go_to_pose(position).wait_for_completed()
                index+=1
        #if cmap not solved
        else:
            numofgoal = len(cmap.get_goals())
            if goal_center == None and numofgoal == 0:
                offsetx = widthofmap/2 - xstart
                offsety = heightofmap/2 - ystart
                positionangle = cozmo.util.Angle((counter%18) * 20)
                position = cozmo.util.Pose(offsetx,offsety,0, angle_z = positionangle)

                #go to the nextposition and start to observe the world again
                await robot.go_to_pose(position).wait_for_completed()
                continue
            #we have goal in this case
            if numofgoal > 0:
                cmap.set_start(curpos)
                #generate the RRT
                RRT(cmap, cmap.get_start())
                #if the cmap is solved,smooth the path and reset the index
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
    x = node[0] * math.cos(local_angle) + node[1] * -math.sin(local_angle) + local_origin[0]
    y = node[0] * math.sin(local_angle) + node[1] * math.cos(local_angle) + local_origin[1]
    new_node = Node((x, y))
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
    cube_padding = 60.
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
        cmap = CozMap("maps/map2.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
