#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*                ===============================================
*                      Logistic coBot (LB) Theme (eYRC 2024-25)
*                ===============================================
*
*  This script should be used to implement Task 1B of Logistic coBot (LB) Theme (eYRC 2024-25).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:          [ Team-ID ]
# Author List:      [ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:         task1b_boiler_plate.py
# Functions:
#                   [ Comma separated list of functions in this file ]
# Nodes:            Add your publishing and subscribing node
#                   Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]


################### IMPORT MODULES #######################

import rclpy
import sys
import cv2
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
import tf_transformations


##################### FUNCTION DEFINITIONS #######################

def calculate_rectangle_area(coordinates):
    '''
    Description:    Function to calculate area or detected aruco

    Args:
        coordinates (list):     coordinates of detected aruco (4 set of (x,y) coordinates)

    Returns:
        area        (float):    area of detected aruco
        width       (float):    width of detected aruco
    '''
    ############ Function VARIABLES ############

    area = None
    width = None

    ############ ADD YOUR CODE HERE ############

    # Extract the corner points
    (top_left, top_right, bottom_right, bottom_left) = coordinates[0]
    width = np.linalg.norm(top_right - top_left)
    height = np.linalg.norm(top_right - bottom_right)
    area = width * height

    ############################################

    return area, width


def detect_aruco(image):
    '''
    Description:    Function to perform aruco detection and return each detail of aruco detected 
                    such as marker ID, distance, angle, width, center point location, etc.

    Args:
        image                   (Image):    Input image frame received from respective camera topic

    Returns:
        center_aruco_list       (list):     Center points of all aruco markers detected
        distance_from_rgb_list  (list):     Distance value of each aruco markers detected from RGB camera
        angle_aruco_list        (list):     Angle of all pose estimated for aruco marker
        width_aruco_list        (list):     Width of all detected aruco markers
        ids                     (list):     List of all aruco marker IDs detected in a single frame 
    '''
    ############ Function VARIABLES ############

    aruco_area_threshold = 1500

    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])

    size_of_aruco_m = 0.15

    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    ############ ADD YOUR CODE HERE ############

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # Check if ids is a numpy array and flatten it only if it is
        if isinstance(ids, np.ndarray):
            ids = ids.flatten().tolist()
        else:
            ids = list(ids)  # Ensure ids is a list

        for corner, marker_id in zip(corners, ids):
            area, width = calculate_rectangle_area(corner)
            if area > aruco_area_threshold:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, size_of_aruco_m, cam_mat, dist_mat)
                center = np.mean(corner, axis=1)[0]

                center_aruco_list.append(center)
                distance_from_rgb_list.append(np.linalg.norm(tvec))
                angle_aruco_list.append(rvec[0])
                width_aruco_list.append(width)

                cv2.aruco.drawDetectedMarkers(image, [corner])
                cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, 0.1)
    ############################################

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


##################### CLASS DEFINITION #######################

class aruco_tf(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to define process for detecting aruco marker and publishing tf on pose estimated.
    '''

    def __init__(self):
        '''
        Description:    Initialization of class aruco_tf
                        All classes have a function called __init__(), which is always executed when the class is being initiated.
                        The __init__() function is called automatically every time the class is being used to create a new object.
                        You can find more on this topic here -> https://www.w3schools.com/python/python_classes.asp
        '''

        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())


    def depthimagecb(self, data):
        '''
        Description:    Callback function for aligned depth camera topic. 
                        Use this function to receive image depth data and convert to CV2 image

        Args:
            data (Image):    Input depth image frame received from aligned depth camera topic

        Returns:
        '''
        ############ ADD YOUR CODE HERE ############
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='32FC1')
        ############################################


    def colorimagecb(self, data):
        '''
        Description:    Callback function for colour camera raw topic.
                        Use this function to receive raw image data and convert to CV2 image

        Args:
            data (Image):    Input coloured raw image frame received from image_raw camera topic

        Returns:
        '''
        ############ ADD YOUR CODE HERE ############
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        ############################################


    def process_image(self):
        '''
        Description:    Timer function used to detect aruco markers and publish tf on estimated poses.

        Args:
        Returns:
        '''
        ############ Function VARIABLES ############
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375
        centerX = None
        centerY = None

        ############ ADD YOUR CODE HERE ############
        if self.cv_image is None or self.depth_image is None:
            return

        center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids = detect_aruco(self.cv_image)

        if ids is not None:
            for i, marker_id in enumerate(ids):
                centerX, centerY = center_aruco_list[i]
                distance = distance_from_rgb_list[i]
                angle = angle_aruco_list[i]
                width = width_aruco_list[i]
                
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_frame"
                t.child_frame_id = f"aruco_{marker_id}"
                t.transform.translation.x = (centerX - centerCamX) * distance / focalX
                t.transform.translation.y = (centerY - centerCamY) * distance / focalY
                t.transform.translation.z = distance
                print("Angle before conversion:", angle)

    # Ensure angle is a scalar by extracting the first element
                if isinstance(angle, (np.ndarray)) and angle.ndim > 1:
                    angle = angle[0]  # Take the first row of the array

    # Assuming angle is now a 1D array or a list
                if isinstance(angle, (list, np.ndarray)):
                    if len(angle) == 3:  # Ensure it has three angles for Euler conversion
                        q = tf_transformations.quaternion_from_euler(
                            0, 0, np.deg2rad(angle[2])  # Use the third element for rotation around Z-axis
                            )
                    else:
                        print("Error: Angle must have three components.")
                        return  # Early exit or handle error
                else:
                    print("Error: Angle must be a list or ndarray.")
                    return  # Early exit or handle error
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                
                self.br.sendTransform(t)
        ############################################


def main(args = None):
    '''
    Description:    main() function for the node
                    It initialises the ros2 communication, creates object of the class and keeps the node alive.

    Args:
    Returns:
    '''

    ############ ADD YOUR CODE HERE ############
    rclpy.init(args=args)
    aruco_node = aruco_tf()
    rclpy.spin(aruco_node)
    aruco_node.destroy_node()
    rclpy.shutdown()
    ############################################


if __name__ == '__main__':
    main()
