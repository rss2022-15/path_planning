#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
import dubins
import tf.transformations
import random
import math


class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.parent = None
        self.cost = 0.0
        self.path_x = []
        self.path_y = []
        self.path_yaw = []

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        # in pixels index in as [v,u] (swapped indices)
        self.map = None
        self.start = None
        self.goal = None
        self.x_range = None
        self.y_range = None

        self.ITER_MAX = 300    # no fukin clu my doods

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    # node should have
    # x, y, yaw position
    # parent
    # cost
    # path x, y, yaw
    def map_cb(self, msg):
        self.map = msg

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        self.start = [x, y, yaw]
        # reinitialize traj and other stuff and check if other things exist and call path plan


    def goal_cb(self, msg):
        pos = msg.pose.position
        quat = msg.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        self.goal = [pos.x, pos.y, yaw]
        # reinitialize traj and other stuff and check if other things exist and call path plan

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        self.start_node = Node(self.start[0], self.start[1], self.start[2])
        self.goal_node = Node(self.goal[0], self.goal[1], self.goal[2])

        self.turn_radius = 1.0      # TODO: what the fuk
        self.step_size = 0.5        # TODO: also what the fuk
        self.step_len = 30.0        # TODO: wtf
        self.search_radius = 50.0   # TODO: also wtf

        self.V = [self.start_node]
        self.path = None

        for i in range(self.ITER_MAX):
            print("iter: ", i, " number of nodes: ", len(self.V))
            rand_node = self.sample()
            near_node = self.nearest(rand_node)
            new_node = self.steer(near_node, rand_node)

            if new_node and not self.is_collision(new_node):
                near_indices = self.Near(new_node)
                new_node = self.choose_parent(new_node, near_indices)

                if new_node:
                    self.V.append(new_node)
                    self.rewire(new_node, near_indices)

        last_index = self.best_goal()
        path = self.generate_final(last_index)

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def sample(self):
        if random.random() > self.goal_sample_rate:
            return Node(random.uniform(0, self.x_range), random.uniform(0, self.y_range), random.uniform(-math.pi, math.pi))
        else:
            return self.goal_node

    def nearest(self, rand):
        return self.V[int(np.argmin([(nd.x - rand.x)**2 + (nd.y - rand.y)**2 for nd in self.V]))]

    def steer(self, near, rand):
        q0 = (near.x, near.y, near.yaw)
        q1 = (rand.x, rand.y, rand.yaw)

        path = dubins.shortest_path(q0, q0, self.turn_radius)
        configurations, _ = path.sample_many(self.step_size)

        if len(configurations) <= 1:
            return None

        new_node = Node(configurations[-1][0], configurations[-1][1], configurations[-1][2])
        new_node.path_x = [configurations[i][0] for i in range(len(configurations))]
        new_node.path_y = [configurations[i][1] for i in range(len(configurations))]
        new_node.path_yaw = [configurations[i][2] for i in range(len(configurations))]
        new_path_len = path.path_length()
        new_node.cost = near.cost + new_path_len
        new_node.parent = near
        return new_node

    def near(self, node):
        n = len(self.V) + 1
        r = min(self.search_radius * math.sqrt((math.log(n)) / n), self.step_len)
        dist_table = [(nd.x - node.x)**2 + (nd.y - node.y)**2 for nd in self.V]
        node_near_ind = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r**2]
        return node_near_ind

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None

        costs = []
        for i in near_inds:
            near_node = self.V[i]
            t_node = self.steer(near_node, new_node)
            if t_node and not self.is_collision(t_node):
                costs.append(near_node.cost + math.hypot((new_node.x - near_node.x), (new_node.y - near_node.y)))
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        return self.steer(self.V[min_ind], new_node)

    def propagate(self, nd):
        for node in self.V:
            if node.parent == nd:
                node.cost = nd.cost + math.hypot((nd.x - node.x),(nd.y - node.y))
                self.propagate(node)

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.V[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = new_node.cost + math.hypot((new_node.x - near_node.x), (new_node.y - near_node.y))

            if not self.is_collision(edge_node) and near_node.cost > edge_node.cost:
                self.V[i] = edge_node
                self.propagate(new_node)

    def best_goal(self):
        dist_to_goals = [math.hypot((nd.x - self.goal_node.x),(nd.y - self.goal_node.y)) for nd in self.V]
        goal_inds = [dist_to_goals.index(i) for i in dist_to_goals if i <= self.step_len]

        good_goal_inds = []
        for i in goal_inds:
            t_node = self.steer(self.V[i], self.goal_node)
            if t_node and not self.is_collision(t_node):
                good_goal_inds.append(i)

        if not good_goal_inds:
            return None

        min_cost = min([self.V[i].cost for i in good_goal_inds])
        for i in good_goal_inds:
            if self.V[i].cost == min_cost:
                return i

        return None

    def generate_final(self, goal_index):
        path = [[self.goal_node.x, self.goal_node.y]]
        node = self.V[goal_index]
        while node.parent:
            for (ix, iy) in zip(reversed(node.path_x), reversed(node.path_y)):
                path.append([ix, iy])
            node = node.parent
        path.append([self.start_node.x, self.start_node.y])
        return path

    def is_collision(self, node):
        pass


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
