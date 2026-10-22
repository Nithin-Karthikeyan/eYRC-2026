#!/usr/bin/env python3

'''
*****************************************************************************************
*
-               ===============================================
-                           StrataCobot (SC) Theme (eYRC 2026-27)
-               ===============================================
*
-  This script should be used to implement Task 1B of StrataCobot (SC) Theme (eYRC 2026-27).
*
-  This software is made available on an "AS IS WHERE IS BASIS".
-  Licensee/end user indemnifies and will keep e-Yantra indemnified from
-  any and all claim(s) that emanate from the use of the Software or
-  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          1198
# Author List:      Nithin
# Filename:         task2A_manipulation.py
# Functions:
#                   [ Comma separated list of functions in this file ]
# Nodes:            Add your publishing and subscribing node
#                   Example:
#                   Publishing Topics  - [ /delta_twist_cmds, /delta_joint_cmds ]
#                   Subscribing Topics - [ /tcp_pose_raw, /joint_states, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import math
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import re # needed to parse ore data from tf buffer and assign numbers to each ore type
import yaml # Convert the data from tf buffer into yaml format to parse using re
import tf2_ros # To convert camera ore position to base_link ore position
from std_srvs.srv import SetBool # Used to set True/False thru service
from std_msgs.msg import Int32, Float32 # Store


##################### TASK CONSTANTS #######################

# Tool positions in base_link, in metres, in the order they must be reached. The tool
# stops at each one and holds it for at least two seconds. Copy the signs as they are:
# base_link is the UR7e's own frame, not the Gazebo world's.

#* Pack this since for 2A the waypoints are the ore centers from base link given by perception node
# waypoints = [
#     (-0.4085, -0.5379, 0.1967),   # 1
#     (-0.8000, -0.0005, 0.3967),   # 2
#     (-0.7430,  0.5280, 0.1967),   # 3
#     (-0.4097,  0.5280, 0.1967),   # 4
#     (-0.0763,  0.5280, 0.1967),   # 5
# ]

# The two command interfaces. Only ONE is active at a time; messages to the other are
# accepted and ignored.
servo_ns = '/ur_arm_controller'
twist_controller = 'delta_twist_controller'
joint_controller = 'delta_joint_controller'

# The only frame a twist may be stamped with; any other is refused, not converted. Note
# 'base' is base_link turned through 180 degrees, not another name for it.
base_frame = 'base_link'

# JointJog velocities are matched to these names, in this order.
joint_names = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

# What the servo accepts. A command above one of these is dropped WHOLE, not clamped.
cap_linear_mps = 0.15     # magnitude of a twist's linear part
cap_angular_rps = 0.35    # magnitude of its angular part
cap_joint_rps = 0.35      # per joint

# Dead-man switch: the arm stops this long after the last message it received.
command_timeout_s = 0.15


##################### CLASS DEFINITION #######################

class arm_waypoints(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to drive the UR7e's tool through the
                    given waypoints using the arm's velocity command interfaces.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class arm_waypoints
        '''

        # use_sim_time is set here, not on the command line, so this node runs on the
        # simulation clock however it is started.
        super().__init__(                                                               # registering node
            'arm_waypoints_node',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        ############ Topic PUBLISHERS ############

        self.twist_pub = self.create_publisher(TwistStamped, '/delta_twist_cmds', 10)    # end-effector velocity, in base_link
        self.joint_pub = self.create_publisher(JointJog, '/delta_joint_cmds', 10)        # per-joint velocity

        ############ Topic SUBSCRIPTIONS ############

        self.tcp_sub = self.create_subscription(PoseStamped, '/tcp_pose_raw', self.tcpposecb, 20)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.jointstatecb, 50)
        self.status_sub = self.create_subscription(Int32, '/arm_status', self.armstatuscb, 10)
        self.force_sub = self.create_subscription(Float32, '/ur7e/end_effector/force_magnitude', self.forcecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        self.control_rate = 0.05                                                        # rate of time to run one control cycle (seconds)
        self.switch_cli = self.create_client(                                           # client used to pick which command topic is live
            SwitchController, f'{servo_ns}/switch_controller')
        self.magnet_cli = self.create_client(SetBool, '/magnet')
        self.timer = self.create_timer(self.control_rate, self.process_waypoints)       # creating a timer based function which gets called every 0.05 seconds

        self.tcp_pose = None                                                            # tool pose variable (from tcpposecb())
        self.joint_angles = None                                                        # joint feedback variable (from jointstatecb())
        self.arm_status = None                                                          # arm state code variable (from armstatuscb())

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #   ->  Add any variable your motion needs to keep between cycles.
        #       ->  HINT: Which waypoint you are on, and what the arm is doing about it.
        self.switch_future = None
        self.active_controller = None
        self.target_controller = None
        self.curr_wp = 0 # Indicates the current wp
        self.next_wp = self.curr_wp + 1 # next wp to go to. Is always one ahead of curr_wp

        self.joint_vel_cap = 0.34
        self.zero_vel_lin = [0.0]*3
        self.zero_vel_ang = [0.0]*3

        self.home_angles = [-1.57, -1.57, -1.57, -1.57, 1.57, 1.57] # Sets the unfold target angles for each joint
        self.unfolded = False # Check if the arm is unfolded before moving using twist messages

        # Cartesian PID state (XYZ position error -> XYZ linear velocity)
        self.pid_integral = [0.0, 0.0, 0.0]
        self.pid_prev_error = None
        self.pid_prev_time = None

        # Error bookkeeping. Each entry stores the MAX TCP position error seen during
        # the successful 2-second hold at that waypoint.
        # self.errors = [None] * len(waypoints)
        self.hold_max_error = None
        self.hold_start = None
        self.hold_duration = 2.0
        self.summary_logged = False
        self.debug_counter = 0
        
        # PID Constants
        self.Kp = 2.0
        self.Ki = 0.0
        self.Kd = 0.5
        self.integral_limit = 0.5  # <-- ADDED TO FIX UNDEFINED VARIABLE
        self.joint_kp = 2.0
        self.pose_tol = 0.03

        # Transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State machine
        #* Valid states = ['HOME', 'SWITCH_TO_TWIST', 'WAIT_FOR_ORE', 'MOVE_ABOVE_ORE', 'MAGNET_ON', 'DESCEND_TO_ORE', 'LIFT_ORE', 'VERIFY_PICK', 'MOVE_ABOVE_DROP', 'DESCEND_TO_DROP', 'MAGNET_OFF', 'RETREAT', 'NEXT_ORE', 'RECOVER_STOP', 'DONE']
        self.state = 'HOME' # Start the node by moving arm to HOME position. 

        self.current_ore = None
        self.current_ore_position = None # Lock this once ore transform is done to make sure arm moving doesn't mean the position of ore isnt readjusted
        self.processed_ores = set()

        # Magnet stuff
        self.force_mag = None
        self.magnet_future = None
        self.magnet_state = False
        self.hover_z = None


        ############################################
    
    def reset_pid(self):
        '''Reset Cartesian PID memory when changing/holding waypoints.'''
        self.pid_integral = [0.0, 0.0, 0.0]
        self.pid_prev_error = None
        self.pid_prev_time = None


    def publish_twist(self, linear, angular):
        '''
        Description:    Function to publish a TwistStamped message on the /delta_twist_cmds topic.

        Args:
            linear (list):  Linear velocity vector [x, y, z] in m/s
            angular (list): Angular velocity vector [x, y, z] in rad/s

        Returns:
        '''

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.twist.linear.x = linear[0]
        msg.twist.linear.y = linear[1]
        msg.twist.linear.z = linear[2]
        msg.twist.angular.x = angular[0]
        msg.twist.angular.y = angular[1]
        msg.twist.angular.z = angular[2]

        self.twist_pub.publish(msg)

    def publish_joint_jog(self, velocities):
        '''
        Description:    Function to publish a JointJog message on the /delta_joint_cmds topic.

        Args:
            velocities (list):  List of joint velocities [v1, v2, v3, v4, v5, v6] in rad/s

        Returns:
        '''

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = joint_names
        msg.velocities = velocities

        self.joint_pub.publish(msg)


    def tcpposecb(self, data):
        '''
        Description:    Callback function for the tool pose topic.
                        Use this function to receive where the tool currently is.

        Args:
            data (PoseStamped):    Pose of the tool, reported in base_link

        Returns:
        '''

        self.tcp_pose = data.pose


    def jointstatecb(self, data):
        '''
        Description:    Callback function for the joint states topic.
                        Use this function to receive the current angle of each joint.

        Args:
            data (JointState):    Joint feedback published by the arm

        Returns:
        '''

        self.joint_angles = [data.position[data.name.index(j)] for j in joint_names]


    def armstatuscb(self, data):
        '''
        Description:    Callback function for the arm status topic.
                        Use this function to receive the arm's current state code.

        Args:
            data (Int32):    One state code describing what the arm is doing

        Returns:
        '''

        self.arm_status = data.data

    def forcecb(self, data):
        '''
        Description:    Callback function for the magnet force topic.
                        Use this function to receive the force acting on the magnet tool.

        Args:
            data (Float32):    Force in Newtons acting on the magnet

        Returns:
        '''
        self.force_mag = data.data


    def switch_controller(self, controller):
        '''
        Description:    Function to make one of the arm's two command interfaces the active
                        one, so that commands published to it are acted on.

        Args:
            controller  (str):      Name of the controller to activate, either
                                    'twist_controller' or 'joint_controller'

        Returns:
            success     (bool):     Whether the controller was activated
        '''

        # Check if controller is valid
        if controller not in (twist_controller, joint_controller):
            self.get_logger().error(f"{controller} is an invalid controller name. Only use 'delta_joint_controller' or 'delta_twist_controller'")
            return False

        # Wait for the service server
        if not self.switch_cli.service_is_ready():
            self.get_logger().error("SwitchController service unavailable")
            return False
        # If the service responded:
        # Create a Request Instance and set the controller
        req = SwitchController.Request()
        req.activate_controllers = [controller]
        req.deactivate_controllers = [twist_controller if controller == joint_controller else joint_controller]
        req.strictness = SwitchController.Request.STRICT

        self.switch_future = self.switch_cli.call_async(req) # Calls service asynchronously to keep the thread from hanging

    def get_tf_position(self, frame_name):
        """
        Returns frame position relative to base_link.

        Example:
        get_tf_position("azurite_ore_1")
        -> [x, y, z]

        Returns None if TF isn't available yet.
        """

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',          # target frame
                frame_name,           # frame we want position of
                rclpy.time.Time()     # latest available TF
            )

            return [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

        except Exception:
            return None


    def get_available_ore_frames(self):
        """
        Find all ore frames currently present in the TF tree.

        Example output:
        [
            'azurite_ore_1',
            'azurite_ore_2',
            'malachite_ore_1',
            'vanadinite_ore_1'
        ]
        """

        # Get all frames currently known by this node's TF buffer
        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames = yaml.safe_load(frames_yaml) or {}

        # Match:
        # azurite_ore_<number>
        # malachite_ore_<number>
        # vanadinite_ore_<number>
        ore_pattern = re.compile(
            r'^(azurite|malachite|vanadinite)_ore_\d+$'
        )

        ore_frames = [
            frame_name
            for frame_name in frames.keys()
            if ore_pattern.match(frame_name)
        ]

        return sorted(ore_frames)

    def move_to_point(self, point, max_speed=cap_linear_mps, tol=None):
        if tol is None:
            tol = self.pose_tol

        curr_pose = self.tcp_pose.position
        target_pose = point

        # Cartesian position error in base_link
        pos_error = [
            target_pose[0] - curr_pose.x,
            target_pose[1] - curr_pose.y,
            target_pose[2] - curr_pose.z,
        ]
        error = math.sqrt(sum(e * e for e in pos_error))

        if error < tol:
            self.get_logger().info(f"Target {[round(x, 3) for x in target_pose]} reached (err={error*1000:.1f} mm).")
            self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
            self.reset_pid()  # <-- Crucial so next state starts with clean derivative/integral
            return True

        # --- Cartesian PID: XYZ position error -> XYZ TCP linear velocity ---
        now = self.get_clock().now()
        if self.pid_prev_time is None or self.pid_prev_error is None:
            dt = self.control_rate
            derivative = [0.0, 0.0, 0.0]
        else:
            dt = (now - self.pid_prev_time).nanoseconds / 1e9
            if dt <= 1e-6 or dt > 0.5:
                dt = self.control_rate
                derivative = [0.0, 0.0, 0.0]
            else:
                derivative = [
                    (pos_error[i] - self.pid_prev_error[i]) / dt
                    for i in range(3)
                ]

        for i in range(3):
            self.pid_integral[i] += pos_error[i] * dt
            self.pid_integral[i] = max(
                -self.integral_limit,
                min(self.pid_integral[i], self.integral_limit)
            )

        p_term = [self.Kp * e for e in pos_error]
        i_term = [self.Ki * e for e in self.pid_integral]
        d_term = [self.Kd * e for e in derivative]

        linear_velocity = [
            p_term[i] + i_term[i] + d_term[i]
            for i in range(3)
        ]

        self.pid_prev_error = pos_error.copy()
        self.pid_prev_time = now

        # Cap the magnitude of the WHOLE Cartesian velocity vector
        speed = math.sqrt(sum(v * v for v in linear_velocity))
        safe_cap = min(0.99 * cap_linear_mps, max_speed)
        if speed > safe_cap and speed > 0.0:
            scale = safe_cap / speed
            linear_velocity = [v * scale for v in linear_velocity]
            speed = safe_cap

        self.publish_twist(linear_velocity, self.zero_vel_ang)

        self.debug_counter += 1
        if self.debug_counter % 10 == 0:
            self.get_logger().info(
                f"err={error*1000:.1f}mm | cmd_speed={speed:.4f} m/s | cmd={['%.3f' % v for v in linear_velocity]}"
            )
        return False


    def set_magnet(self, state: bool):
        if not self.magnet_cli.service_is_ready():
            self.get_logger().warn("Magnet service not ready...")
            return False
        self.magnet_state = state
        req = SetBool.Request()
        req.data = self.magnet_state
        self.magnet_future = self.magnet_cli.call_async(request=req)
        return True
    
    def process_waypoints(self):
        '''
        Description:    Timer function used to drive the tool through the waypoints.

        Args:
        Returns:
        '''

        # 1. Check if all required data has arrived
        if self.tcp_pose is None or self.joint_angles is None or self.arm_status is None:
            self.get_logger().info("Waiting for all data to arrive...")
            return

        # 2. Handle Controller Switching
        if self.switch_future is not None:
            self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
            # 2.1 Return if the future is not complete yet
            if not self.switch_future.done():
                return 

            # 2.2 If Future is complete, then get result and check
            res = self.switch_future.result()
            self.switch_future = None # Set future back to none for the next service call to come into this loop

            if res is not None and res.ok:
                self.get_logger().info(f"Switched to {self.target_controller}")
                self.active_controller = self.target_controller
            else:
                self.get_logger().error(f"Switch to {self.target_controller} Failed. Active controller is {self.active_controller}")

        # Implement the state machine:
        #! state = HOME
        if self.state == "HOME":
            # 3.1 Check if controller is joint
            if self.active_controller != joint_controller:
                self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
                self.target_controller = joint_controller
                self.switch_controller(self.target_controller)
                self.get_logger().info(f"Switch to {self.target_controller} Initiated...")
                return 

            joint_Kp = self.joint_kp
            joint_errors = [target - current for target, current in zip(self.home_angles, self.joint_angles)]
            joint_velocity = [max(-self.joint_vel_cap, min(error * joint_Kp, self.joint_vel_cap)) for error in joint_errors]
            self.publish_joint_jog(joint_velocity)
            joint_tolerance = 0.02
            self.get_logger().info(f"Joint angles: {[round(q, 3) for q in self.joint_angles]}")
            if all(abs(e) < joint_tolerance for e in joint_errors):
                self.publish_joint_jog([0.0] * 6)
                self.state = "SWITCH_TO_TWIST"
                return

        elif self.state == "SWITCH_TO_TWIST":
            if self.active_controller != twist_controller:
                self.publish_joint_jog([0] * 6)
                self.target_controller = twist_controller
                self.switch_controller(self.target_controller)
                self.get_logger().info(f"Switch to {self.target_controller} Initiated...")
                return
            self.get_logger().info(f"{self.target_controller} Active. Moving to next State")
            self.state = "WAIT_FOR_ORE"

        elif self.state == "WAIT_FOR_ORE":
            self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
            ore_frames = self.get_available_ore_frames()

            for ore_name in ore_frames:
                if ore_name in self.processed_ores:
                    continue
                ore_position = self.get_tf_position(ore_name)
                if ore_position is None:
                    continue

                self.current_ore = ore_name
                self.current_ore_position = ore_position
                # Keep current Z height so MOVE_ABOVE_ORE only moves horizontally in X and Y
                self.hover_z = self.tcp_pose.position.z

                self.get_logger().info(
                    f"Selected {ore_name} at {[round(x, 3) for x in ore_position]} | Hover Z={self.hover_z:.3f}"
                )
                self.reset_pid()
                self.state = "MOVE_ABOVE_ORE"
                return

        elif self.state == "MOVE_ABOVE_ORE":
            # Target has ore's X and Y, but keeps TCP's hover_z
            above_target = [
                self.current_ore_position[0],
                self.current_ore_position[1],
                self.hover_z
            ]
            if self.move_to_point(above_target) is True:
                self.get_logger().info(f"Directly above {self.current_ore}! Switching to DESCEND_TO_ORE")
                self.state = "MAGNET_ON"
                return

        elif self.state == "MAGNET_ON":
            if self.magnet_state is not True:
                self.set_magnet(state=True) # The function itself updates the self.magnet_state var
            # Handle magnet state setting to True or False
            if self.magnet_future is not None:
                self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
                self.get_logger().warn(f"Magnet Future not done yet...")
                if not self.magnet_future.done():
                    return
                res = self.magnet_future.result()
                self.magnet_future = None
                if res is not None:
                    self.get_logger().info(f"Switched Magnet state to {self.magnet_state}")
                else:
                    self.get_logger().error(f"Switch in magnet state failed. Active state is {self.magnet_state}")
            else:
                self.state = "DESCEND_TO_ORE"

        elif self.state == "DESCEND_TO_ORE": #TODO The TCP angle seems to change slightly, will have to check if thats an issue
            # Target now uses the ore's Z coordinate (with slow approach speed so it doesn't crash)
            descend_target = [
                self.current_ore_position[0],
                self.current_ore_position[1],
                self.current_ore_position[2]
            ]
            if self.move_to_point(descend_target, max_speed=0.25, tol=0.015) is True:
                self.get_logger().info(f"Reached {self.current_ore} top face!")
                self.state = "LIFT_ORE"
                return

        elif self.state == "LIFT_ORE":
            if self.force_mag < 1.4:
                self.get_logger().warn(f"Force on magnet is {self.force_mag}, Object not picked")
                return 
            ascend_target = [
                            self.current_ore_position[0],
                            self.current_ore_position[1],
                            self.hover_z
                        ]
            if self.move_to_point(ascend_target, max_speed=0.25, tol=0.015) is True:
                self.get_logger().info(f"{self.force_mag}")
                self.state = "VERIFY_PICKUP"

        








        


            
        # # 4. If unfolded is true, then send twist commands
        # else:
        #     # Check if Active controller is twist, if not change it:
        #     if self.active_controller != twist_controller:
        #         self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)
        #         self.target_controller = twist_controller
        #         self.switch_controller(self.target_controller)
        #         self.get_logger().info(f"Switch to {self.target_controller} Initiated...")
        #         return 

        #     # 5. Put all way point reached logic here, and increment self.curr_wp when reached.
        #     if self.curr_wp >= len(waypoints):
        #         self.publish_twist(self.zero_vel_lin, self.zero_vel_ang)

        #         if not self.summary_logged:
        #             self.get_logger().info("All waypoints reached.")
        #             summary = " | ".join(
        #                 f"WP{i + 1}: {err:.5f} m ({err * 1000.0:.1f} mm)"
        #                 if err is not None else f"WP{i + 1}: not recorded"
        #                 for i, err in enumerate(self.errors)
        #             )
        #             self.get_logger().info(f"Max error during each successful 2 s hold -> {summary}")
        #             self.summary_logged = True
        #         return
            # # get current pose
            # curr_pose = self.tcp_pose.position
            # target_pose = waypoints[self.curr_wp]
            # # Cartesian position error in base_link
            # pos_error = [
            #     target_pose[0] - curr_pose.x,
            #     target_pose[1] - curr_pose.y,
            #     target_pose[2] - curr_pose.z,
            # ]
            # error = math.sqrt(sum(e * e for e in pos_error))
            # waypoint_tolerance = self.wp_tol

            # self.debug_counter += 1
            # if self.debug_counter % 10 == 0:  # ~2 Hz at a 20 Hz control loop
            #     self.get_logger().info(
            #         f"WP {self.curr_wp + 1} error: {error:.5f} m ({error * 1000.0:.1f} mm) | "
            #         f"ex={pos_error[0]:+.4f}, ey={pos_error[1]:+.4f}, ez={pos_error[2]:+.4f}"
            #     )

            # if error < waypoint_tolerance:
            #     self.get_logger().info(f"Waypoint {self.curr_wp + 1} reached.")
            #     # Keep explicitly commanding zero velocity while holding
            #     self.publish_twist(
            #         self.zero_vel_lin,
            #         self.zero_vel_ang
            #     )

        #         # Start hold timer when waypoint is first reached
        #         if self.hold_start is None:
        #             self.hold_start = self.get_clock().now()
        #             self.hold_max_error = error
        #             self.reset_pid()

        #             self.get_logger().info(
        #                 f"Waypoint {self.curr_wp + 1} reached at {error * 1000.0:.1f} mm error. Starting 2 second hold..."
        #             )

        #             return

        #         # Calculate how long we have been holding
        #         elapsed = (
        #             self.get_clock().now() - self.hold_start
        #         ).nanoseconds / 1e9

        #         self.hold_max_error = max(self.hold_max_error, error)

        #         if self.debug_counter % 10 == 0:
        #             self.get_logger().info(
        #                 f"Holding WP {self.curr_wp + 1}: {elapsed:.2f} s | "
        #                 f"current error={error * 1000.0:.1f} mm | "
        #                 f"max hold error={self.hold_max_error * 1000.0:.1f} mm"
        #             )

        #         # Hold complete
        #         if elapsed >= self.hold_duration:

        #             self.errors[self.curr_wp] = self.hold_max_error
        #             self.get_logger().info(
        #                 f"Waypoint {self.curr_wp + 1} held successfully for 2 seconds. "
        #                 f"Max hold error = {self.hold_max_error:.5f} m ({self.hold_max_error * 1000.0:.1f} mm)"
        #             )

        #             self.curr_wp += 1
        #             self.hold_start = None
        #             self.hold_max_error = None
        #             self.reset_pid()
                    
        #         return
        #     else:
        #         # If the tool left the tolerance region before completing the hold,
        #         # restart the hold next time it enters.
        #         if self.hold_start is not None:
        #             self.get_logger().info(
        #                 f"WP {self.curr_wp + 1} left tolerance at {error * 1000.0:.1f} mm; restarting hold timer."
        #             )
        #             self.reset_pid()
        #         self.hold_start = None
        #         self.hold_max_error = None

        #         # --- Cartesian PID: XYZ position error -> XYZ TCP linear velocity ---
        #         now = self.get_clock().now()
        #         if self.pid_prev_time is None or self.pid_prev_error is None:
        #             dt = self.control_rate
        #             derivative = [0.0, 0.0, 0.0]
        #         else:
        #             dt = (now - self.pid_prev_time).nanoseconds / 1e9
        #             # Protect against zero/negative dt or a simulation time jump/reset.
        #             if dt <= 1e-6 or dt > 0.5:
        #                 dt = self.control_rate
        #                 derivative = [0.0, 0.0, 0.0]
        #             else:
        #                 derivative = [
        #                     (pos_error[i] - self.pid_prev_error[i]) / dt
        #                     for i in range(3)
        #                 ]

        #         for i in range(3):
        #             self.pid_integral[i] += pos_error[i] * dt
        #             self.pid_integral[i] = max(
        #                 -self.integral_limit,
        #                 min(self.pid_integral[i], self.integral_limit)
        #             )

        #         p_term = [self.Kp * e for e in pos_error]
        #         i_term = [self.Ki * e for e in self.pid_integral]
        #         d_term = [self.Kd * e for e in derivative]

        #         linear_velocity = [
        #             p_term[i] + i_term[i] + d_term[i]
        #             for i in range(3)
        #         ]

        #         self.pid_prev_error = pos_error.copy()
        #         self.pid_prev_time = now

        #         # Cap the magnitude of the WHOLE Cartesian velocity vector.
        #         speed = math.sqrt(sum(v * v for v in linear_velocity))
        #         safe_cap = 0.99 * cap_linear_mps
        #         if speed > safe_cap and speed > 0.0:
        #             scale = safe_cap / speed
        #             linear_velocity = [v * scale for v in linear_velocity]
        #             speed = safe_cap

        #         angular_velocity = [0.0, 0.0, 0.0]
        #         self.publish_twist(linear_velocity, angular_velocity)

        #         if self.debug_counter % 10 == 0:
        #             self.get_logger().info(
        #                 f"PID gains Kp={self.Kp:.3f}, Ki={self.Ki:.3f}, Kd={self.Kd:.3f} | "
        #                 f"cmd_speed={speed:.4f} m/s | cmd={['%.4f' % v for v in linear_velocity]}"
        #             )


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the
                    arm_waypoints class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation
    node = rclpy.create_node('arm_waypoints_process')               # creating ROS node
    node.get_logger().info('Node created: Arm waypoints process')   # logging information
    arm_waypoints_class = arm_waypoints()                           # creating a new object for class 'arm_waypoints'
    rclpy.spin(arm_waypoints_class)                                 # spining on the object to make it alive in ROS 2 DDS
    arm_waypoints_class.destroy_node()                              # destroy node after spin ends
    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()