#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

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
        self.debug_pub = rospy.Publisher("/debug_point_search", Marker, queue_size=1)


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
        # rospy.loginfo([self.map])

        path = self.astar()
        self.trajectory.points = path

        #rospy.loginfo([path])

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.visualize = True
        self.trajectory.publish_viz()

    def astar(self):
        #convert robot position to occpancy map position
        map_width = self.map.info.width# 1730
        map_height = self.map.info.height# 1300
        map_resolution = self.map.info.resolution # 0.0504000000656
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y

        #     position: 
        #       x: 25.9
        #       y: 48.5
        #       z: 0.0
        # 

        start_x = int(self.start[0])#int((self.start[0] - map_origin_x)/map_resolution - 0.5)
        start_y = int(self.start[1])#int((self.start[1] - map_origin_y)/map_resolution - 0.5)
        #start_x = int((self.start[0] - map_origin_x)/map_resolution - 0.5)
        #start_y = int((self.start[1] - map_origin_y)/map_resolution - 0.5)

        #occupancy grid -> (height -> x , width -> y)

        rospy.loginfo(["start grid pos", [start_x, start_y]])


        start_node = Node(None, [start_x, start_y])
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, self.goal)
        end_node.g = end_node.h = end_node.f = 0

        local_x, local_y = self.real_to_map((end_node.position[0], end_node.position[1]))
        
        val = self.map.data[local_x*map_height+local_y]
        rospy.loginfo([local_x, local_y, val])
        rospy.loginfo(["origin:", self.map.info.origin])

        # Initialize both open and closed list
        open_list = []
        closed_list = []

        # Add the start node
        open_list.append(start_node)
        rospy.loginfo(end_node.position)

        # Loop until you find the end or run out of time
        time = 0
        while len(open_list) > 0 and time <= 1000:
            time += 1
            #rospy.loginfo(len(open_list))

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
            #rospy.loginfo(current_node.position)
            if (current_node.position[0] == end_node.position[0]) and (current_node.position[1] == end_node.position[1]):
                path = []
                current = current_node
                while current is not None:
                    #grid to node 
                    pos_x = current.position[0]#map_origin_x + (current.position[0] + 0.5) * map_resolution
                    pos_y = current.position[1]#map_origin_y + (current.position[1] + 0.5) * map_resolution
                    path.append([pos_x, pos_y])
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()

            marker.type = 2
            marker.id = 0

            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.83
            marker.color.a = 1.0

            marker.pose.position.x = current_node.position[0]
            marker.pose.position.y = current_node.position[1]
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            self.debug_pub.publish(marker)
            children = []
            # angle = current_node.position[2] # TODO: Only drive forward
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # # Make sure within range
                # if node_position[0] > self.map.info.height or node_position[0] < 0 or node_position[1] > (len(self.map[len(self.map)-1]) -1) or node_position[1] < 0:
                #     continue

                # Make sure walkable terrain

                #local_x = int((node_position[0] - map_origin_x)/map_resolution - 0.5)
                #local_y = int((node_position[1] - map_origin_y)/map_resolution - 0.5)

                local_x, local_y = self.real_to_map(node_position)

                val = self.map.data[local_x*map_height+local_y]
                #rospy.loginfo(val)
                #rospy.loginfo(np.argwhere([np.array(self.map.data) >= 0]))
                if val < 80 and val >= 0:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                # Child is on the closed list
                if child in closed_list:
                    continue
                #for closed_child in closed_list:
                    #if child == closed_child:
                        #continue

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
        rospy.loginfo(time)

    def real_to_map(self, p):
        quat = self.map.info.origin.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (origin_roll, origin_pitch, origin_yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        px_x = p[0]
        px_y = p[1]
        local_x = px_x * np.cos(-origin_yaw) - px_y * np.sin(-origin_yaw)
        local_y = px_y * np.cos(-origin_yaw) + px_x * np.sin(-origin_yaw)
        local_x -= int(self.map.info.origin.position.x)
        local_y -= int(self.map.info.origin.position.y)
        local_x /= self.map.info.resolution
        local_y /= self.map.info.resolution

        return (-int(local_x), -int(local_y))

    def map_to_real(self, p):
        quat = self.map.info.origin.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (origin_roll, origin_pitch, origin_yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        px_x = p[0]
        px_y = p[1]
        px_x *= self.map.info.resolution
        px_y *= self.map.info.resolution
        px_x += int(self.map.info.origin.position.x)
        px_y += int(self.map.info.origin.position.y)
        local_x = px_x * np.cos(origin_yaw) - px_y * np.sin(origin_yaw)
        local_y = px_y * np.cos(origin_yaw) + px_x * np.sin(origin_yaw)

        return (local_x, local_y)

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
