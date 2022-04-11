#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
# import dubins
import tf.transformations
import random
import math

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def map_cb(self, msg):
        self.map = msg
        if (self.map != None and self.start != None and self.goal != None):
            self.plan_path()

    def odom_cb(self, msg):
        x = int(msg.pose.pose.position.x)
        y = int(msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        self.start = [x, y]
        # reinitialize traj and other stuff and check if other things exist and call path plan
        if (self.map != None and self.start != None and self.goal != None):
            self.plan_path()


    def goal_cb(self, msg):
        pos = msg.pose.position
        quat = msg.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        self.goal = [int(pos.x), int(pos.y)]
        if (self.map != None and self.start != None and self.goal != None):
            self.plan_path()

    def plan_path(self):
        ## CODE FOR PATH PLANNING ##

        
        path = self.astar()

        # publish trajectory
        self.traj_pub.publish(path.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def astar(self):
        start_node = Node(None, self.start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, self.goal)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            # angle = current_node.position[2] # TODO: Only drive forward
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # # Make sure within range
                # if node_position[0] > self.map.info.height or node_position[0] < 0 or node_position[1] > (len(self.map[len(self.map)-1]) -1) or node_position[1] < 0:
                #     continue

                # Make sure walkable terrain
                if self.map.data[node_position[0]][node_position[1]] > 80:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                for closed_child in closed_list:
                    if child == closed_child:
                        continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()