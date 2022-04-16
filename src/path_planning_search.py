#!/usr/bin/env python

from threading import local
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

import rospkg
import time, os
from utils import LineTrajectory
# import dubins
import tf.transformations
import random
import math
from skimage import morphology

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
        self.debug_pub = rospy.Publisher("/debug_point_search", Marker, queue_size=1)


    def map_cb(self, msg):
        self.map = msg
        self.map_width = self.map.info.width
        self.map_height = self.map.info.height
        self.map_resolution = self.map.info.resolution
        self.map_origin_x = self.map.info.origin.position.x
        self.map_origin_y = self.map.info.origin.position.y

        quat = self.map.info.origin.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (origin_roll, origin_pitch, self.origin_yaw) = tf.transformations.euler_from_quaternion(explicit_quat)
        data = np.array([self.map.data]).reshape((self.map_height, self.map_width))
        self.map_dilated = morphology.dilation(data, selem=np.ones((8,8)))

        # if (self.map != None and self.start != None and self.goal != None):
        #     self.plan_path()

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # if (self.start is None and self.map is not None and self.goal is not None):
        #     self.start = [x, y]
        #     rospy.loginfo("got start and will plan")
        #     rospy.loginfo(self.start)
        #     self.plan_path()
        if self.map is None:
            return

        self.start = [x,y]

    def goal_cb(self, msg):
        pos = msg.pose.position

        self.goal = [pos.x, pos.y]
        rospy.loginfo("got new goal ")
        rospy.loginfo(self.goal)
        if (self.map is not None and self.start is not None):
            self.plan_path()
        # if (self.map != None and self.start != None and self.goal != None):
        #     self.plan_path()

    def plan_path(self):
        ## CODE FOR PATH PLANNING ##
        path = self.astar()
        self.trajectory.points = path

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.visualize = True
        self.trajectory.publish_viz()

    def astar(self):
        start_map = self.real_to_map(self.start)
        start_x = start_map[0]
        start_y = start_map[1]

        #occupancy grid -> (height -> x , width -> y)
        goal_map = self.real_to_map(self.goal)
        # pixel frame
        start_node = Node(None, [start_x, start_y])
        start_node.g = start_node.h = start_node.f = 0

        end_node = Node(None, goal_map)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end or run out of time
        time = 0
        # while len(open_list) > 0 and time <= 1000:
        while len(open_list) > 0:
            time += 1

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # rospy.loginfo("current node")
            # rospy.loginfo(current_node.position)
            # rospy.loginfo("its f value")
            # rospy.loginfo(current_node.f)

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.append(current_node)

            # for closed_node in closed_list:
            #     # open_node_map = self.real_to_map(open_node.position)
            #     # child_map = self.real_to_map(child.position)
            #     if (current_node.position[0] == closed_node.position[0]) and (current_node.position[1] == closed_node.position[1]):
            #         continue

            # Found the goal
            if (current_node.position[0] == end_node.position[0]) and (current_node.position[1] == end_node.position[1]):
                # traj_lol = PoseArray()
                # traj_lol.header.frame_id = "/map"
                # traj_lol.header.stamp = rospy.Time.now()
                # traj_lol.poses = []
                path = []
                current = current_node
                while current is not None:
                    #grid to node 
                    pos_x = current.position[0]
                    pos_y = current.position[1]
                    real_x, real_y = self.map_to_real([pos_x, pos_y])
                    # thankunext = Pose()
                    # thankunext.position.x = real_x
                    # thankunext.position.y = real_y
                    # path.append(thankunext)
                    path.append([real_x, real_y])
                    current = current.parent
                rospy.loginfo("hello cna anyone hear me")
                # traj_lol.poses = path[::-1]
                # self.traj_pub.publish(traj_lol)
                return path[::-1] # Return reversed path

            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()

            marker.type = 2
            marker.id = 0

            marker.scale.x = .2
            marker.scale.y = .2
            marker.scale.z = .05

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.83
            marker.color.a = 1.0

            marker.pose.position.x = self.map_to_real(current_node.position)[0]
            marker.pose.position.y = self.map_to_real(current_node.position)[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            self.debug_pub.publish(marker)      
            # Generate children
            
            children = []
            # angle = current_node.position[2] # TODO: Only drive forward
            # for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            child_offsets = []
            for i in range(-5, 6):
                for j in range(-5, 6):
                    child_offsets.append((i,j))

            # for new_position in [(0, -3), (0, 3), (-3, 0), (3, 0)]:
            for new_position in child_offsets:
                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
                local_x, local_y = node_position

                # # Make sure within range
                # if local_x > self.map_height-1 or local_x < 0 or local_y > self.map_width-1 or local_y < 0:
                if local_x > self.map_width-1 or local_x < 0 or local_y > self.map_height-1 or local_y < 0:
                    rospy.loginfo("out of bounds, continuing")
                    continue

                # Make sure walkable terrain
                # val = self.map.data[local_x*self.map_width+local_y]
                val = self.map_dilated[local_y, local_x]
                if val > 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                exists = False
                for closed_child in closed_list:
                    if child.position[0] == closed_child.position[0] and child.position[1] == closed_child.position[1]:
                        exists = True
                        break
                if exists:
                    continue
                exists = False

                # Create the f, g, and h values
                # child.g = current_node.g + 1
                # next_cost = current_node.g + math.sqrt((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                # next_cost = current_node.g + (child.position[0] - current_node.position[0])**2 + (child.position[1] - current_node.position[1])**2

                # this does work below
                # next_cost = current_node.g + 1
                # child.g = next_cost
                # priority = next_cost + np.abs(child.position[0] - end_node.position[0]) + np.abs(child.position[1] - end_node.position[1])

                next_cost = current_node.g + np.abs(child.position[0] - current_node.position[0]) + np.abs(child.position[1] - current_node.position[1])
                child.g = next_cost
                priority = next_cost + (child.position[0] - end_node.position[0]) ** 2 + (child.position[1] - end_node.position[1]) ** 2
                # child.g = current_node.g + ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                # child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                # child.h = np.abs(child.position[0] - end_node.position[0]) + np.abs(child.position[1] - end_node.position[1])
                # child.f = child.g + child.h
                child.f = priority

                # Child is already in the open list
                for open_node in open_list:
                    # if (child.position[0] == open_node.position[0]) and (child.position[1] == open_node.position[1]) and child.g > open_node.g:
                    #     continue
                    if (child.position[0] == open_node.position[0]) and (child.position[1] == open_node.position[1]):
                        exists = True
                        break
                if exists:
                    continue

                # Add the child to the open list
                open_list.append(child)
        rospy.loginfo("time?")
        rospy.loginfo(time)

    def real_to_map(self, p):
        px_x = p[0]
        px_y = p[1]

        new_pose = np.zeros(3)
        new_pose[0] = (px_x - self.map_origin_x) / self.map_resolution
        new_pose[1] = (px_y - self.map_origin_y) / self.map_resolution

        rot = np.array([[np.cos(self.origin_yaw), -np.sin(self.origin_yaw), 0],
                        [np.sin(self.origin_yaw), np.cos(self.origin_yaw), 0],
                        [0, 0, 1]])

        new_pose = np.int64(np.matmul(rot, new_pose))
        return (new_pose[0], new_pose[1])

    def map_to_real(self, p):
        px_x = p[0]
        px_y = p[1]

        new_pose = np.zeros(3)
        new_pose[0] = px_x
        new_pose[1] = px_y

        rot = np.array([[np.cos(self.origin_yaw), np.sin(self.origin_yaw), 0],
                        [-np.sin(self.origin_yaw), np.cos(self.origin_yaw), 0],
                        [0, 0, 1]])
        new_pose = np.matmul(rot, new_pose)
        new_pose[0] = new_pose[0] * self.map_resolution + self.map_origin_x
        new_pose[1] = new_pose[1] * self.map_resolution + self.map_origin_y
        return (new_pose[0], new_pose[1])

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
