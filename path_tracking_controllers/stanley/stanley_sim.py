#!/usr/bin/env python3

# Python Headers
import math
import numpy as np
from numpy import linalg as la
from threading import Lock

# ROS Headers
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Gazebo Headers
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

from gem_manager.msg import WaypointsBatch, Waypoint


class Stanley(object):

    def __init__(self):

        self.rate = rospy.Rate(20)
        self.wheelbase = 1.75  # meters

        self.path_points_x = []
        self.path_points_y = []
        self.path_points_yaw = []
        self.data_lock = Lock()

        self.waypoint_counter = 0
        self.initial_k = 0.05  # Lower K for the start
        self.regular_k = 0.45

        # Track the time when the last waypoint batch was received
        self.last_waypoint_time = rospy.Time.now()

        # Timeout threshold in seconds
        self.waypoint_timeout = rospy.Duration(2.0)

        self.ackermann_msg = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration = 0.0
        self.ackermann_msg.jerk = 0.0
        self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0

        self.ackermann_pub = rospy.Publisher(
            "/gem/ackermann_cmd", AckermannDrive, queue_size=1
        )

        self.waypoints_sub = rospy.Subscriber(
            "/gem_manager/waypoints", WaypointsBatch, self.waypoints_callback
        )

    def waypoints_callback(self, msg):
        new_waypoints = {
            (float(wp.x), float(wp.y), float(wp.yaw)) for wp in msg.waypoints
        }

        with self.data_lock:
            existing_waypoints = set(
                zip(self.path_points_x, self.path_points_y, self.path_points_yaw)
            )

            unique_waypoints = new_waypoints - existing_waypoints

            for x, y, yaw in unique_waypoints:
                self.path_points_x.append(x)
                self.path_points_y.append(y)
                self.path_points_yaw.append(yaw)

        # Update the time of the last waypoint reception
        self.last_waypoint_time = rospy.Time.now()

    def get_gem_state(self):
        rospy.wait_for_service("/gazebo/get_model_state")

        try:
            service_response = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            model_state = service_response(model_name="gem")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q = model_state.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x_dot = model_state.twist.linear.x
        y_dot = model_state.twist.linear.y

        return round(x, 4), round(y, 4), round(x_dot, 4), round(y_dot, 4), round(yaw, 4)

    def pi_2_pi(self, angle):
        if angle > math.pi:
            return angle - 2.0 * math.pi
        if angle < -math.pi:
            return angle + 2.0 * math.pi

        return angle

    def stop_vehicle(self):
        # Gradually stop the vehicle
        if self.ackermann_msg.speed > 0.1:
            self.ackermann_msg.speed *= 0.9
        else:
            self.ackermann_msg.speed = 0.0
        self.ackermann_msg.steering_angle = 0.0
        self.ackermann_pub.publish(self.ackermann_msg)

    def start_stanley(self):
        while not rospy.is_shutdown():

            # Check if waypoints have timed out
            if rospy.Time.now() - self.last_waypoint_time > self.waypoint_timeout:
                rospy.logwarn("No waypoints received recently. Stopping the vehicle.")
                self.stop_vehicle()
                self.rate.sleep()
                continue

            # get current position and orientation in the world frame
            # reference point is located at the center of rear axle
            curr_x, curr_y, curr_x_dot, curr_y_dot, curr_yaw = self.get_gem_state()

            # reference point is located at the center of frontal axle
            front_x = self.wheelbase * np.cos(curr_yaw) + curr_x
            front_y = self.wheelbase * np.sin(curr_yaw) + curr_y

            with self.data_lock:
                # Check if waypoints exist
                if (
                    not self.path_points_x
                    or not self.path_points_y
                    or not self.path_points_yaw
                ):
                    rospy.logwarn("No waypoints available. Stopping the vehicle.")
                    self.stop_vehicle()
                    self.rate.sleep()
                    continue

                # find the closest point
                dx = [front_x - x for x in self.path_points_x]
                dy = [front_y - y for y in self.path_points_y]

            # find the index of closest point
            target_index = int(np.argmin(np.hypot(dx, dy)))

            # Check if target_index+1 is within bounds
            with self.data_lock:
                if target_index + 1 >= len(self.path_points_x):
                    rospy.logwarn("Not enough waypoints ahead. Stopping the vehicle.")
                    self.stop_vehicle()
                    self.rate.sleep()
                    continue

            self.waypoint_counter += 1

            front_axle_vec_rot_90 = np.array(
                [
                    [math.cos(curr_yaw - math.pi / 2.0)],
                    [math.sin(curr_yaw - math.pi / 2.0)],
                ]
            )

            vec_target_2_front = np.array([[dx[target_index]], [dy[target_index]]])

            # crosstrack error
            ef = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)
            ef = float(np.squeeze(ef))

            # vehicle heading
            theta = curr_yaw

            with self.data_lock:
                # approximate heading of path at (path_x, path_y)
                path_x = self.path_points_x[target_index]
                path_y = self.path_points_y[target_index]
                path_x_next = self.path_points_x[target_index + 1]
                path_y_next = self.path_points_y[target_index + 1]
            theta_p = np.arctan2(path_y_next - path_y, path_x_next - path_x)

            # theta_e is the heading error
            theta_e = self.pi_2_pi(theta_p - theta)

            f_vel = round(np.sqrt(curr_x_dot**2 + curr_y_dot**2), 3)

            if self.waypoint_counter < 700:
                K = self.initial_k
            else:
                K = self.regular_k

            # Steering angle calculation with dynamic K
            delta = round(theta_e + math.atan2(K * ef, f_vel), 3)

            theta_e = round(np.degrees(theta_e), 1)

            ef = round(ef, 3)
            print("Crosstrack Error: " + str(ef) + ", Heading Error: " + str(theta_e))

            # implement constant pure pursuit controller
            self.ackermann_msg.speed = 2.8
            self.ackermann_msg.steering_angle = delta
            self.ackermann_pub.publish(self.ackermann_msg)

            self.rate.sleep()


def stanley():
    rospy.init_node("stanley_sim_node", anonymous=True)
    sl = Stanley()

    try:
        sl.start_stanley()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    stanley()
