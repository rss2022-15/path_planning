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
from skimage import morphology


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
        self.map_dilated = None

        self.turn_radius = 1.0      # TODO: what the fuk
        self.step_size = 1        # TODO: also what the fuk
        self.step_len = 10.0        # TODO: wtf
        self.search_radius = 50.0   # TODO: also wtf

        self.ITER_MAX = 1400    # no fukin clu my doods

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
        self.map_width = self.map.info.width
        self.map_height = self.map.info.height
        self.map_resolution = self.map.info.resolution
        self.map_origin_x = self.map.info.origin.position.x
        self.map_origin_y = self.map.info.origin.position.y

        data = np.array([self.map.data]).reshape((self.map_height, self.map_width))
        self.map_dilated = morphology.dilation(data, selem=np.ones((8,8)))

        self.x_range = self.map_width
        self.y_range = self.map_height
        if (self.map != None and self.start != None and self.goal != None):
            self.plan_path()

    def odom_cb(self, msg):
        x = int(msg.pose.pose.position.x)
        y = int(msg.pose.pose.position.y)
        quat = msg.pose.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        if (self.start is None and self.map is not None and self.goal is not None):
            self.start = [x, y, yaw]
            rospy.loginfo("got start and will plan")
            rospy.loginfo(self.start)
            self.plan_path()
        # reinitialize traj and other stuff and check if other things exist and call path plan
        # if (self.map != None and self.start != None and self.goal != None):
        #     self.plan_path()


    def goal_cb(self, msg):
        pos = msg.pose.position
        quat = msg.pose.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        self.goal = [int(pos.x), int(pos.y), yaw]
        rospy.loginfo("got new goal ")
        rospy.loginfo(self.goal)

    def plan_path(self):
        rospy.loginfo([self.map])
        ## CODE FOR PATH PLANNING ##
        start_map = self.real_to_map(self.start)
        goal_map = self.real_to_map(self.goal)
        self.start_node = Node(int(start_map[0]), int(start_map[1]), self.start[2])
        self.goal_node = Node(int(goal_map[0]), int(goal_map[1]), self.goal[2])

        

        # start_map = self.real_to_map(self.start)

        self.V = [self.start_node]
        self.path = None

        for i in range(self.ITER_MAX):
            print("iter: ", i, " number of nodes: ", len(self.V))
            rand_node = self.sample()
            near_node = self.nearest(rand_node)
            new_node = self.steer(near_node, rand_node)
            print("checking for is_collision")
            if new_node and not self.is_collision(new_node):
                print("no collision")

                near_indices = self.near(new_node)
                print("near incicies", near_indices)
                new_node = self.choose_parent(new_node, near_indices)
                print("is new node", new_node)

                if new_node:
                    self.V.append(new_node)
                    self.rewire(new_node, near_indices)

        last_index = self.best_goal()
        path = self.generate_final(last_index)
        self.trajectory.points = path

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def sample(self):
        if random.random() > 0.1: #self.goal_sample_rate:
            return Node(random.uniform(0, self.x_range), random.uniform(0, self.y_range), random.uniform(-math.pi, math.pi))
        else:
            return self.goal_node

    def nearest(self, rand):
        return self.V[int(np.argmin([(nd.x - rand.x)**2 + (nd.y - rand.y)**2 for nd in self.V]))]

    def steer(self, near, rand):
        q0 = (near.x, near.y, near.yaw)
        q1 = (rand.x, rand.y, rand.yaw)
        print(q0)

        path = dubins.shortest_path(q0, q1, self.turn_radius)
        configurations, _ = path.sample_many(self.step_size)

        if len(configurations) <= 1:
            print("config < 1")
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
            print("not near ins")
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
            print("min cost is inf")
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
        for i in range(len(node.path_x)) :
            
            pos = [node.path_x[i], node.path_y[i]]
 
            # map_pos = self.real_to_map(pos)
            print("pos", pos)

            if pos[0] > self.map_width-1 or pos[0] < 0 or pos[1] > self.map_height-1 or pos[1] < 0:
                print("out of bounds, continuing")
                return True

            val = self.map_dilated[int(pos[0]), int(pos[1])]
            if val > 0:
                print("hit wall, continuing")
                return True
        print("clear")
        return False



    def real_to_map(self, p):
        quat = self.map.info.origin.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (origin_roll, origin_pitch, origin_yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        px_x = p[0]
        px_y = p[1]
        new_pose = np.zeros(3)
        new_pose[0] = int((px_x - self.map.info.origin.position.x) / self.map.info.resolution)
        new_pose[1] = int((px_y - self.map.info.origin.position.y) / self.map.info.resolution)
        rot = np.array([[np.cos(origin_yaw), -np.sin(origin_yaw), 0],
                        [np.sin(origin_yaw), np.cos(origin_yaw), 0],
                        [0, 0, 1]])

        new_pose = np.int64(np.matmul(rot, new_pose))

        # local_x = px_x * np.cos(-origin_yaw) - px_y * np.sin(-origin_yaw)
        # local_y = px_y * np.cos(-origin_yaw) + px_x * np.sin(-origin_yaw)
        # local_x -= int(self.map.info.origin.position.x)
        # local_y -= int(self.map.info.origin.position.y)
        # local_x /= self.map.info.resolution
        # local_y /= self.map.info.resolution

        # return (-int(local_x), -int(local_y))
        return (new_pose[0], new_pose[1])

    def map_to_real(self, p):
        quat = self.map.info.origin.orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w]
        (origin_roll, origin_pitch, origin_yaw) = tf.transformations.euler_from_quaternion(explicit_quat)

        px_x = p[0]
        px_y = p[1]
        new_pose = np.zeros(3)
        new_pose[0] = px_x
        new_pose[1] = px_y
        rot = np.array([[np.cos(origin_yaw), np.sin(origin_yaw), 0],
                        [-np.sin(origin_yaw), np.cos(origin_yaw), 0],
                        [0, 0, 1]])
        new_pose = np.matmul(rot, new_pose)
        new_pose[0] = new_pose[0] * self.map.info.resolution + self.map.info.origin.position.x
        new_pose[1] = new_pose[1] * self.map.info.resolution + self.map.info.origin.position.y
        return (new_pose[0], new_pose[1])
    # def raytrace(A, B):
        # """ Return all cells of the unit grid crossed by the line segment between
        #     A and B.
        # """

        # (xA, yA) = 
        # (xB, yB) = B
        # (dx, dy) = (xB - xA, yB - yA)
        # (sx, sy) = (sign(dx), sign(dy))

        # grid_A = (floor(A[0]), floor(A[1]))
        # grid_B = (floor(B[0]), floor(B[1]))
        # (x, y) = grid_A
        # traversed=[grid_A]

        # tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
        # tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")

        # while (x,y) != grid_B:
        #     # NB if tIx == tIy we increment both x and y
        #     (movx, movy) = (tIx <= tIy, tIy <= tIx)

        #     if movx:
        #         # intersection is at (x + sx, yA + tIx / dx^2)
        #         x += sx
        #         tIx = dy * (x + sx - xA)

        #     if movy:
        #         # intersection is at (xA + tIy / dy^2, y + sy)
        #         y += sy
        #         tIy = dx * (y + sy - yA)

        #     traversed.append( (x,y) )

        # return traversed


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
