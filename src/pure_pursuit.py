#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
#import tf2_ros
#import tf2_geometry_msgs

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.speed            = 10. # FILL IN #
        self.lookahead        = 1.#.*np.log2(self.speed+1) # FILL IN #
        #self.wrap             = # FILL IN #
        self.wheelbase_length = 1. # FILL IN #
        self.last_angle_error = 0
        self.current_odom     = None
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.car_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.debug_pub = rospy.Publisher("/debug_point", Marker, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def odom_callback(self, data):
        #rospy.loginfo("odom called back :)")
        self.current_odom = data
        self.get_nearest_point()

    def get_nearest_point(self):
        if self.trajectory is None or len(self.trajectory.points) == 0:
            return
        #v_dist = np.vectorize(self.calc_dist, excluded=["carp"])
        #distances = v_dist(self.trajectory.points, np.roll(self.trajectory.points, -1), carp=(self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y))

        # carp = Position of car in world frame
        carp = (self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y)
        car_quaternion_ = [self.current_odom.pose.pose.orientation.x, self.current_odom.pose.pose.orientation.y, self.current_odom.pose.pose.orientation.z, self.current_odom.pose.pose.orientation.w]
        # caro = Orientation of car in world frame
        caro = tf.transformations.euler_from_quaternion(car_quaternion_)[2]

        distances = np.zeros(len(self.trajectory.points)-1)
        # TODO: Vectorize this segment so it runs faster
        i=0
        #for p1, p2 in zip(self.trajectory.points[:-1], np.roll(self.trajectory.points, -1)[:-1]):
        while i < len(distances):
            p1 = self.trajectory.points[i]
            p2 = self.trajectory.points[i+1]
            #if i == 0:
                #rospy.loginfo(["points", p1, p2])
            distances[i] = self.calc_dist(p1, p2, carp)
            i += 1

        i_shortest_distance = np.argmin(distances)
        #length_of_traj = self.trajectory.distances[i+1]

        #rospy.loginfo(["distances: ", distances])

        target = None
        #i = i_shortest_distance
        # Loop through each trajectory segment to find intersection with circle around car
        for i in range(i_shortest_distance, len(distances)):
            intersection = self.line_circle_intersection(self.trajectory.points[i], self.trajectory.points[i+1], carp, self.lookahead)
            if intersection is not None and len(intersection) != 0:
                j = 0
                targets = []
                while len(targets) < 2:
                    if intersection is not None and len(intersection) != 0:
                        for point in intersection:
                            point = (point[0], point[1]+j)
                            targets.append(point)
                    j += 1
                    if i+j <= len(distances)-1:
                        intersection = self.line_circle_intersection(self.trajectory.points[i+j], self.trajectory.points[i+j+1], carp, self.lookahead)
                    else:
                        break
                target = max(targets, key=lambda x: x[1])[0]
                break

        else:  # No intersection found
            # p1, p2 = line segment, p3 = car position, x = closets point on line segment to x
            i = i_shortest_distance
            x1, y1 = self.trajectory.points[i]
            x2, y2 = self.trajectory.points[i+1]
            x3, y3 = carp
            dx, dy = x2-x1, y2-y1
            det = dx*dx + dy*dy
            a = (dy*(y3-y1)+dx*(x3-x1))/float(det)
            x =  [x1+a*dx, y1+a*dy]
            #rospy.loginfo(["point on line  :", x, "could not find intersection"])
            target = x

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()

        marker.type = 2
        marker.id = 0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.83
        marker.color.a = 1.0

        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.debug_pub.publish(marker)

        # Implement drive command 
        # Get relative position of target in world frame
        relative_x = (target[0] - carp[0])
        relative_y = (target[1] - carp[1])
        drive_cmd = AckermannDriveStamped()
        distance_to_point = np.sqrt((target[1]-carp[1])**2 + (target[0]-carp[0])**2)
        #angle_to_point = np.arctan(float(relative_y) / relative_x) - caro

        # Convert world frame target to local frame of car by rotation
        local_x = relative_x * np.cos(-caro) - relative_y * np.sin(-caro)
        local_y = relative_y * np.cos(-caro) + relative_x * np.sin(-caro)

        # Find the angle error of the car to the point
        angle_to_point = np.arctan(local_y / local_x)
        #rospy.loginfo(["car orientation:", caro, "angle to point: ", angle_to_point])
        angle_error = angle_to_point #np.arctan(2.*self.wheelbase_length*np.sin(angle_to_point) / self.lookahead)

        if local_x < 0:
            angle_error = -angle_error

        angle_deriv = angle_error - self.last_angle_error
        self.last_angle_error = angle_error

        k_1 = 1
        k_2 = 0.2
        angle = k_1 * angle_error + k_2 * angle_deriv

        drive_cmd.drive.speed = self.speed
        drive_cmd.drive.steering_angle = angle

        self.drive_pub.publish(drive_cmd)
        #rospy.loginfo(["drive_angle: ", angle, "angle to target", angle_to_point])

    def line_circle_intersection(self, p1, p2, pc, r):
        """
        Calculates the intersection between a circle centered at pc and that has radius r
        with a line segment defined by (p1, p2)

        Returns the point where this intersection occurs, or None if there is no intersection.
        """

        (p1x, p1y), (p2x, p2y), (cx, cy) = p1, p2, pc
        (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
        dx, dy = float(x2 - x1), float(y2 - y1)
        dr = (dx**2 + dy**2)**(0.5)
        big_d = x1 * y2 - x2 * y1
        discriminant = r**2 * dr**2 - big_d**2

        if discriminant < 0: # No intersection
            return None
        else:
            intersections = [
                ( cx + (big_d*dy + sign * (-1 if dy < 0 else 1) * dx * discriminant**0.5) / (dr**2)
                , cy + (-big_d*dx + sign * abs(dy) * discriminant**0.5) / dr**2)
                for sign in ((1, -1) if dy < 0 else (-1, 1))]
            frac_along_segment = [(xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy for xi, yi in intersections]
            intersections = [(pt, frac) for pt, frac in zip(intersections, frac_along_segment) if 0 <= frac <= 1]
            return intersections

    def calc_dist(self, p1, p2, carp):
        """
        Calculate distance from carp(osition) (xp, yp) to line segment defined by p1(x1, y1), p2(x2, y2)
        """
        #rospy.loginfo((p1, p2, carp))
        x1, y1 = p1
        x2, y2 = p2
        xp, yp = carp
        dx = x2-x1
        dy = y2-y1

        norm = dx**2 + dy**2

        u = ((xp - x1) * dx + (yp - y1) * dy) / float(norm)

        if u > 1:
            u = 1
        elif u < 0:
            u = 0

        x = x1 + u * dx
        y = y1 + u * dy

        distx = x - xp
        disty = y - yp

        dist = (distx**2 + disty**2)**0.5
        return dist#((x,y), dist)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
