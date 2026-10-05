#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1C of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*/*
*****************************************************************************************
'''

# Team ID:          1198
# Author List:      Nithin
# Filename:         task1c_1198.py
# Functions:        [ __init__, pathcb, odomcb, scancb, process_navigation, main ]
# Nodes:
#                   Publishing Topics  - [ /cmd_vel ]
#                   Subscribing Topics - [ /ebot_path, /odom, /scan ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


##################### TASK CONSTANTS #######################

# The route is published here ONCE, before your node starts, and held for late
# subscribers. An ordinary subscription waits forever for a message already sent.
path_topic = '/ebot_path'

odom_topic = '/odom'
scan_topic = '/scan'
cmd_topic = '/cmd_vel'

# What /cmd_vel accepts. The base clips anything beyond these, and clipping one of the
# two changes their ratio - which is the arc the base actually drives.
max_linear_mps = 0.5
max_angular_rps = 1.0

qos_profile_path = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )


##################### CLASS DEFINITION #######################

class ebot_nav(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the eBot along the route
                    published on /ebot_path, through its waypoints, in order.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ebot_nav
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'ebot_nav_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic SUBSCRIPTIONS ############

        # ->  NOTE: The subscription to 'path_topic' below uses the default settings and will
        #           NEVER receive the route - see the INSTRUCTIONS in 'pathcb' and fix it there.
        self.path_sub = self.create_subscription(Path, path_topic, self.pathcb, qos_profile_path)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odomcb, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scancb, qos_profile_sensor_data)

        ############ Topic PUBLISHERS ############

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)                      # the eBot drives on what you publish here

        ############ Constructor VARIABLES/OBJECTS ############

        control_rate = 0.05                                                             # rate of time to run one control cycle (seconds)
        self.timer = self.create_timer(control_rate, self.process_navigation)           # creating a timer based function which gets called on every 0.05 seconds (as defined by 'control_rate' variable)

        self.route = None                                                               # the route to drive (from pathcb())
        self.odom = None                                                                # where the base is (from odomcb())
        self.scan = None                                                                # what the lidar sees (from scancb())

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your controller needs to keep between cycles.
        #       ->  HINT: How far along the route you have got, for a start.

        self.curr_wp, self.curr_y = None, None
        self.curr_yaw = None
        self.wp_th = 0.25
        self.curr_wp = 0
        ############################################


    def pathcb(self, data):
        '''
        Description:    Callback function for the route topic.
                        Use this function to receive the waypoints the eBot has to drive.

        Args:
            data (Path):    The route, as a sequence of poses

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  This callback will not fire as written. The route is published once, before
        #       this node starts, and held for late subscribers, so the subscription has to
        #       match the publisher's QoS-
        #           QoSProfile(depth=1, history=..., reliability=..., durability=...)
        #       ->  HINT: 'ros2 topic info /ebot_path -v' says what the publisher offers. The
        #                 policies are in 'rclpy.qos'; a held message is TRANSIENT_LOCAL.

        #   ->  Store the waypoints, and log how many arrived and in which frame.
        #       ->  HINT: data.poses[i].pose.position

        #   ->  Take the route once. Replacing it mid-run resets whatever you keep with it.

        ############################################
        if self.route is None:
            self.route = data.poses
            self.get_logger().info(f"Received route with {len(self.route)} waypoints in frame '{data.header.frame_id}'")    


    def odomcb(self, data):
        '''
        Description:    Callback function for the odometry topic.
                        Use this function to receive where the base currently is.

        Args:
            data (Odometry):    Pose and velocity of the base

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the position and the heading of the base.
        #       ->  HINT: data.pose.pose.position, data.pose.pose.orientation (a quaternion)
        #                     yaw = atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        #                 or 'euler_from_quaternion' from tf_transformations.

        ############################################
        self.odom = data
        self.curr_x = data.pose.pose.position.x
        self.curr_y = data.pose.pose.position.y
        
        q = data.pose.pose.orientation
        self.curr_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))


    def scancb(self, data):
        '''
        Description:    Callback function for the lidar topic.
                        Use this function to receive what the lidar currently sees.

        Args:
            data (LaserScan):    One lidar sweep

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the scan. Read the message definition first
        #       -> https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html

        #   ->  Work out the direction of each beam-
        #           angle_i = angle_min + i * angle_increment      # 0 is straight ahead

        #   ->  Keep only the entries that are measurements-
        #           math.isfinite(r) and range_min <= r <= range_max

        #   ->  The ranges are measured from the LIDAR, not from the centre of the base.
        #       ->  HINT: /tf carries where it is mounted. Do not type in an offset.

        ############################################
        self.scan = data


    def process_navigation(self):
        '''
        Description:    Timer function used to drive the eBot along the route.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Return early until route, odometry and scan have all arrived.

        #   ->  Steer towards the waypoint you are heading for and publish a Twist on
        #       self.cmd_pub. Only linear.x and angular.z do anything-
        #           distance = hypot(tx - x, ty - y)
        #           heading  = wrap(atan2(ty - y, tx - x) - yaw)
        #           angular  = Kp * heading
        #       ->  HINT: wrap(e) is atan2(sin(e), cos(e)); without it the base turns the
        #                 long way round. Drive slower the worse the heading error is.

        #   ->  Keep the command inside the limits above yourself.

        #   ->  Decide when a waypoint is done and the next becomes the target. Too early and
        #       you never went there; too late and you have already driven on.
        #       ->  NOTE: The waypoints must be taken IN THE ORDER the route lists them.

        #   ->  Get around the props. A controller that follows the line and ignores the scan
        #       will not get through.
        #       ->  HINT: Somewhere to start: take the nearest valid range in a cone ahead,
        #                 and compare the room on each side to pick a way round.
        #       ->  NOTE: The eBot's geometry is in its URDF, and its widest part is not its
        #                 chassis.

        #   ->  This is a four-wheel SKID STEER. How cleanly it turns on the spot is
        #       something to measure, not to assume.

        #   ->  Check what the base did on /odom, not on the command you sent.

        #   ->  Stop the base once the route is done. Think about what "done" means for a
        #       base that sails past the last waypoint without ever being close to it.

        #   ->  Nothing here should raise. An uncaught exception kills the node and leaves
        #       the base coasting.

        ############################################
        # 1. Gatekeeper: Ensure all sensor data and route have arrived
        if self.odom is None or self.scan is None or self.route is None:
            return

        # 2. Check if all waypoints have been visited
        if self.curr_wp >= len(self.route):
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            return

        # 3. Retrieve target waypoint coordinates
        target_pt = self.route[self.curr_wp].pose.position
        tx, ty = target_pt.x, target_pt.y
        dx = tx - self.curr_x
        dy = ty - self.curr_y

        dist_to_goal = math.hypot(dx, dy)

        # 4. Check if current waypoint is reached
        if dist_to_goal < self.wp_th:
            self.get_logger().info(f"Reached waypoint {self.curr_wp}")
            self.curr_wp += 1
            return

        # 5. Base heading towards waypoint
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - self.curr_yaw),
            math.cos(target_heading - self.curr_yaw)
        )

        # 6. LiDAR obstacle check (Cone in front and Left/Right clearance)
        cone_angle = 0.40          # ~23 deg cone on each side
        obstacle_dist_limit = 0.8  # Safety threshold in meters

        min_front_dist = float('inf')
        left_clearances = []
        right_clearances = []

        for i, r in enumerate(self.scan.ranges):
            if math.isfinite(r) and self.scan.range_min <= r <= self.scan.range_max:
                angle = self.scan.angle_min + (i * self.scan.angle_increment)

                # Front cone
                if abs(angle) < cone_angle:
                    if r < min_front_dist:
                        min_front_dist = r

                # Sector clearances (Side with higher average distance is clearer)
                if 0.0 < angle <= math.pi / 2:
                    left_clearances.append(r)
                elif -math.pi / 2 <= angle < 0.0:
                    right_clearances.append(r)

        # 7. Obstacle avoidance bias
        if min_front_dist < obstacle_dist_limit:
            avg_left = sum(left_clearances) / len(left_clearances) if left_clearances else 0.0
            avg_right = sum(right_clearances) / len(right_clearances) if right_clearances else 0.0

            # Turn towards the side with GREATER clearance
            if avg_left >= avg_right:
                heading_error += 0.6  # Steer left
            else:
                heading_error -= 0.6  # Steer right

            # Re-normalize heading error
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 8. Compute motion commands
        cmd = Twist()
        kp_yaw = 1.2

        cmd.angular.z = kp_yaw * heading_error

        # Slow down linear velocity if heading error is large
        cmd.linear.x = max_linear_mps * max(0.0, math.cos(heading_error))
        if min_front_dist < 0.5:
            cmd.linear.x *= 0.5

        # 9. Clip velocities to task limits
        cmd.linear.x = max(-max_linear_mps, min(max_linear_mps, cmd.linear.x))
        cmd.angular.z = max(-max_angular_rps, min(max_angular_rps, cmd.angular.z))

        # 10. Publish
        self.cmd_pub.publish(cmd)


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    ebot_nav class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ebot_nav_process')                    # creating ROS node

    node.get_logger().info('Node created: eBot navigation process') # logging information

    ebot_nav_class = ebot_nav()                                     # creating a new object for class 'ebot_nav'

    rclpy.spin(ebot_nav_class)                                      # spining on the object to make it alive in ROS 2 DDS

    ebot_nav_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':

    main()
