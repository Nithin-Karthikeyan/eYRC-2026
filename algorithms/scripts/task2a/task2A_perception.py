#!/usr/bin/env python3


'''
*****************************************************************************************
*
*        		===============================================
*           		        StrataCobot (SC) Theme (eYRC 2026-27)
*        		===============================================
*
*  This script should be used to implement Task 1A of StrataCobot (SC) Theme (eYRC 2026-27).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          1198
# Author List:		Ananth Raghav Pai
# Filename:		    ore_detector.py
# Functions:        detect_ores, ore_tf.process_image
#			        [ Comma separated list of functions in this file ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/camera/color/image_raw, 
#                                          /camera/camera/aligned_depth_to_color/image_raw, 
#                                          /camera/camera/color/camera_info ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, PointStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point


##################### TASK CONSTANTS #######################

# Two ores of each type are spawned - six in all - told apart by an id of 1 or 2.
ore_types = ['azurite_ore', 'malachite_ore', 'vanadinite_ore']

# The RealSense topics. The depth image is ALIGNED to the colour image.
color_topic = '/camera/camera/color/image_raw'
depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
camera_info_topic = '/camera/camera/color/camera_info'

# The parent frame every ore transform is published against.
base_frame = 'base_link'


##################### FUNCTION DEFINITIONS #######################

def detect_ores(image):
    '''
    Description:    Function to detect the ores present in a colour image frame and
                    return the pixel location and the type of each one found.

    Args:
        image                   (Image):    Input colour image frame received from the camera topic

    Returns:
        center_ore_list         (list):     Center pixel (cX, cY) of every ore detected in the frame
        ore_type_list           (list):     Type of each ore detected, taken from 'ore_types'
    '''

    ############ Function VARIABLES ############

    # ->  You can remove these variables if needed. These are just for suggestions to let you get started

    center_ore_list = []
    ore_type_list = []

    ############ ADD YOUR CODE HERE ############

    # INSTRUCTIONS & HELP :

    #	->  Detect the ores by COLOUR, and return a center pixel and a type for each.
    #       ->  HINT: hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #                 mask = cv2.inRange(hsv, lower, upper)       # one pair per type
    #                 H is 0-179, S and V are 0-255. The frame is BGR, not RGB.

    #   ->  Read the bounds off a saved frame rather than copying them from a tutorial.

    #   ->  Clean the mask, and drop anything too small to be an ore.
    #       ->  HINT: cv2.morphologyEx (MORPH_OPEN, MORPH_CLOSE), cv2.contourArea

    #   ->  Reduce each region to one center pixel.
    #       ->  HINT: M  = cv2.moments(contour)                   # guard against m00 == 0
    #                 cX = int(M['m10'] / M['m00'])
    #                 cY = int(M['m01'] / M['m00'])

    #   ->  Draw your detections on the frame while you are developing.
    #       ->  HINT: cv2.circle, cv2.putText

    ############################################
    # Convert the BGR image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the upper and lower hsv bounds for each of the ores
    azurite_lower = np.array([100, 100, 100])
    azurite_upper = np.array([130, 255, 255])
    malachite_lower = np.array([35, 100, 100])
    malachite_upper = np.array([85, 255, 255])
    vanadinite_lower = np.array([8, 200, 200])
    vanadinite_upper = np.array([18, 255, 255])

    lower = [azurite_lower, malachite_lower, vanadinite_lower]
    upper = [azurite_upper, malachite_upper, vanadinite_upper]

    # Define BGR color for the ores
    azurite_color = (255, 0, 0)
    malachite_color = (0, 255, 0)
    vanadinite_color = (0, 0, 255)
    colors = [azurite_color, malachite_color, vanadinite_color]

    # Iterate over the three ore types
    for i in range(3):
        mask = cv2.inRange(hsv, lower[i], upper[i])                                                 # Mask for the hsv range of the ore
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)            # Detect contours in the mask image  
        valid_contours = [contour for contour in contours if 1000 > cv2.contourArea(contour) > 100] # Discard contours that have too large or too small of an area

        # Calculate the moments to get the center of the contour
        for contour in valid_contours:
            M = cv2.moments(contour)
            cX = int(M['m10'] / (M['m00'] + 1e-6))
            cY = int(M['m01'] / (M['m00'] + 1e-6))

            # Append the ore center and name of detection to the lists
            center_ore_list.append((cX, cY))
            ore_type_list.append(ore_types[i])

        # Draw the contours for visualization
        cv2.drawContours(image, valid_contours, -1, colors[i], 2)

    # Visualization of the ore center and ore type
    for i in range(len(center_ore_list)):
        ore_name = ore_type_list[i]
        center = center_ore_list[i]
        cv2.circle(img=image, center=center, radius=2, color=(0, 0, 0), thickness=-1)
        cv2.putText(image, f'{ore_name}_{i % 2 + 1}', center, cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255))

    # Store the annotated image
    # cv2.imwrite('SC#1198_task1A_detection.png', image)
    #cv2.imshow("Image", image)
    #cv2.waitKey(1)

    return center_ore_list, ore_type_list

def detect_drop(image):
    '''
    This function is to get the transform of the drop locations from the base_link
    It does so by:
    - First cropping the image to the left bottom corner of the image 
    - Applying hsv mask to filter out the required box
    - Detecting contours on morphologically cleaned mask
    - Using approxPolyDP to approximate as a trapezium
    - Calculating the centers with moments
    '''

    # Cropping the image
    x_end = 200
    y_start = 200
    roi = image[y_start:, :x_end]

    # Create mask for the drop box color
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
  
    lower_gray = np.array([0, 30, 100])
    upper_gray = np.array([25, 90, 230])
    mask = cv2.inRange(hsv, lower_gray, upper_gray)

    # Clean the mask with morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # Contour detection 
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # # Create display image
    # display_roi = roi.copy()
    box_centers = []

    min_area = 1500 

    for cnt in contours:
        if cv2.contourArea(cnt) > min_area:
            # Get contour perimeter
            peri = cv2.arcLength(cnt, True)
            
            # Get trapezium shape with approxPolyDP
            approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)

            # Check if approximated polygon is a 4-sided quadrilateral/trapezium
            # cv2.drawContours(display_roi, [cnt], -1, (120, 0, 0), 2)
            if len(approx) == 4:
                # Compute centroid of the trapezium using moments
                M = cv2.moments(approx)
                if M["m00"] != 0:
                    cx_roi = int(M["m10"] / (M["m00"] + 1e-6))
                    cy_roi = int(M["m01"] / (M["m00"] + 1e-6))

                    # Map ROI coordinates back to full image space
                    cx_full = cx_roi
                    cy_full = cy_roi + y_start
                    box_centers.append([cx_full, cy_full])

                    # Draw green trapezium contour & red center dot
                    # For debugging 
                    # cv2.drawContours(display_roi, [approx], -1, (0, 255, 0), 2)
                    # cv2.circle(display_roi, (cx_roi, cy_roi), 5, (0, 0, 255), -1)

    # Sort the centers based on y coordinate
    # It is now in the order [Azurite, Malachite, Vanadinite]
    sorted_centers = sorted(box_centers, key=lambda center: center[1])

    # cv2.imshow("Mask", closed)
    # cv2.waitKey(1)
    # cv2.imshow("Masked", display_roi)
    # cv2.waitKey(1)
    return sorted_centers


##################### CLASS DEFINITION #######################

class ore_tf(Node):
    '''
    ___CLASS___

    Description:    Class which serves the purpose to detect the ores in the cell and
                    broadcast a transform for each one.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class ore_tf
        '''
        
        super().__init__('ore_tf_publisher')                                            # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, color_topic, self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, depth_topic, self.depthimagecb, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.caminfocb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)

        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())
        self.cam_info = None                                                            # camera intrinsics variable (from caminfocb())

        ############ ADD YOUR CODE HERE ############
        self.k = None
        self.drop_height_offset = 0.1
        self.base_from_cam_transform = None

        # INSTRUCTIONS & HELP :

        #	->  Add any variable your detection needs to keep between frames.
        #       ->  HINT: The two ores of a type must keep their ids for the whole run, and
        #                 'detect_ores' returns them unordered.

        ############################################


    def depthimagecb(self, data):
        '''
        Description:    Callback function for the aligned depth camera topic.
                        Use this function to receive the depth image and convert it to a CV2 image.

        Args:
            data (Image):    Input depth image frame received from the aligned depth camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        raw_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')     # Desired encoding of 32FC1 to get the distance in meters and float
        raw_depth_image[~np.isfinite(raw_depth_image) | (raw_depth_image < 0.0)] = 0.0  # Removing the NaN and negative depth values 
        self.depth_image = raw_depth_image                                              # Saving to member variable

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        #   ->  Get the units right. Print 'depth.dtype'.
        #       ->  HINT: 32FC1 is METRES; 16UC1 is MILLIMETRES, so divide by 1000.

        #   ->  Drop the pixels that carry no reading.
        #       ->  HINT: depth[np.isfinite(depth) & (depth > 0.0)]

        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for the colour camera raw topic.
                        Use this function to receive the raw image and convert it to a CV2 image.

        Args:
            data (Image):    Input coloured raw image frame received from the image_raw camera topic

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')    # Desired encoding of 'bgr8' for color image and CV2 format is BGR.

        # INSTRUCTIONS & HELP :

        #	->  Convert the ROS Image message to a CV2 image and store it.
        #       ->  HINT: self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        ############################################


    def caminfocb(self, data: CameraInfo):
        '''
        Description:    Callback function for the camera info topic.
                        Use this function to receive the camera's intrinsic parameters.

        Args:
            data (CameraInfo):    Camera calibration published by the camera

        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Store the focal lengths and the principal point. Read them from the topic;
        #       never hard-code them.
        #       ->  HINT: 'k' is the pinhole matrix flattened row by row-
        #                     k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

        ############################################
        self.k = data.k.ravel()         # Converts the 2D K matrix to 1D row-wise.


    def process_image(self):
        '''
        Description:    Timer function used to detect the ores and publish a transform for
                        each one on its estimated position.

        Args:
        Returns:
        '''

        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP :

        #	->  Return early until both images and the camera info have arrived.

        #   ->  Get the ore centers and their types from 'detect_ores' defined above

        #   ->  Read the depth at each center pixel. Depth is ALIGNED to colour.
        #       ->  HINT: Take the MEDIAN of a small window, not the one pixel-
        #                     patch = self.depth_image[cY-4:cY+5, cX-4:cX+5]
        #                     z     = np.median(patch[np.isfinite(patch) & (patch > 0.0)])

        #   ->  Deproject the center pixel (u, v) and its depth z into a 3D point-
        #           x = (u - cx) * z / fx
        #           y = (v - cy) * z / fy
        #           z = z
        #       ->  HINT: That point is in the camera's OPTICAL frame - 'data.header.frame_id'.

        #   ->  Transform it into 'base_frame'-
        #           tf = self.tf_buffer.lookup_transform(
        #                    base_frame, <optical frame>, rclpy.time.Time())
        #           p  = do_transform_point(point_in_camera, tf)
        #       ->  HINT: PointStamped and tf2_geometry_msgs are not imported above, and the
        #                 lookup raises until the tree has filled in.

        #   ->  Correct for what you measured: the camera sees the ore's top face, and the
        #       ore is named by its middle.

        #   ->  Give each ore an id and keep it for the whole run.
        #       ->  HINT: Match a detection to the nearest ore of that type already named,
        #                 on the pixel rather than the depth.

        #   ->  Publish one transform per ore, using Geometry Message - TransformStamped
        #       Use the following frame_id-
        #           frame_id = 'base_link'
        #           child_frame_id = '<ore_type>_<id>'      Ex: azurite_ore_1, where azurite_ore
        #                                                   is one of 'ore_types' and the id is 1 or 2
        #       ->  HINT: t.header.stamp, t.header.frame_id, t.child_frame_id,
        #                 t.transform.translation.x/.y/.z, t.transform.rotation.w = 1.0,
        #                 then self.br.sendTransform(t). Every number must be a float.
        #       ->  NOTE: Only the translation is read; the names are matched exactly.

        #   ->  Broadcast every ore on every cycle, not once.

        #   ->  Show the frame with your detections drawn on it using 'cv2.imshow'.

        ############################################
        # Return until the cv image and depth image are acquired
        if self.cv_image is None or self.depth_image is None or self.k is None:
            self.get_logger().warn("Waiting for CV image and depth image....")
            return

        # Get the centers and ore types detection using the detect_ores function
        center_ore_list, ore_type_list = detect_ores(self.cv_image)
        if len(center_ore_list) == 0 or len(ore_type_list) == 0:
            self.get_logger().warn("No ores detected.....")
            return

        try:
            # Lookup the transform from the camera_color_optical_frame to base_link
            self.base_from_cam_transform = self.tf_buffer.lookup_transform(base_frame, 'camera_color_optical_frame', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")
            return

        # Create the ore transforms from base_link
        transforms = []

        for i, center in enumerate(center_ore_list):
            # Get the point in 3D space
            point_in_base_frame = self.get_point(center)
            
            # Create the Transform message to be broadcasted
            ore_tf_from_base = self.create_tf(
                f'{ore_type_list[i]}_{i % 2 + 1}', 
                [point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z]
            )

            transforms.append(ore_tf_from_base)

        # Get the centers of drop boxes
        box_centers = detect_drop(self.cv_image)

        for i, center in enumerate(box_centers):
            point_in_base_frame = self.get_point(center)

            drop_tf_from_base = self.create_tf(
                f'{ore_types[i]}_drop',
                [point_in_base_frame.point.x, point_in_base_frame.point.y, point_in_base_frame.point.z + self.drop_height_offset]
            )

            transforms.append(drop_tf_from_base)

        # Broadcast all the ore transforms at once, if not empty
        if len(transforms) != 0:
            self.br.sendTransform(transforms)

        # Send the transforms of the dropping areas
        detect_drop(self.cv_image)


    def get_point(self, center):
        # Get the median depth of a patch as the depth of the ore from the camera
        cX, cY = center
        patch = self.depth_image[cY - 5: cY + 5, cX - 5 : cX + 5]
        z = np.median(patch[patch > 0.0])

        # Calculate the x and y coordinates of the 3D point using projection
        # k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        x = (cX - self.k[2]) * z / self.k[0]
        y = (cY - self.k[5]) * z / self.k[4]
        # Transform the point in camera_color_optical_frame to base_link
        point_in_cam_frame = PointStamped()
        point_in_cam_frame.header.frame_id = 'camera_color_optical_frame'
        point_in_cam_frame.header.stamp = self.base_from_cam_transform.header.stamp
        point_in_cam_frame.point.x = x
        point_in_cam_frame.point.y = y
        point_in_cam_frame.point.z = z

        point_in_base_frame = do_transform_point(point_in_cam_frame, self.base_from_cam_transform)
        return point_in_base_frame
    

    def create_tf(self, child_frame_id, position, stamp=None):
        '''
        Description:    Helper function to create a TransformStamped message against base_link.

        Args:
            child_frame_id (str): Name of the child frame (e.g., 'azurite_ore_1', 'azurite_drop')
            position (list/tuple): [x, y, z] coordinates in meters
            stamp (Time, optional): ROS timestamp. Defaults to current node time if None.
        '''
        t = TransformStamped()
        t.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        t.header.frame_id = base_frame
        t.child_frame_id = str(child_frame_id)

        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        return t 
                


##################### FUNCTION DEFINITION #######################

def main():
    '''
    Description:    Main function which creates a ROS node and spins around for the ore_tf
                    class to perform its task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('ore_tf_process')                      # creating ROS node

    node.get_logger().info('Node created: Ore tf process')          # logging information

    ore_tf_class = ore_tf()                                         # creating a new object for class 'ore_tf'

    rclpy.spin(ore_tf_class)                                        # spining on the object to make it alive in ROS 2 DDS

    ore_tf_class.destroy_node()                                     # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process
    


if __name__ == '__main__':

    main()