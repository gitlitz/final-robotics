"""
This module contains useful utilities for moving the robot in the environment
"""
import consts
import random
import time
import math
import logging
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

CAMERA_FOV = 1.0472


def fuzz():
    """
    Move the robot randomly. This function will never end!
    """
    velocity_publisher = rospy.Publisher(consts.VEL_TOPIC, Twist, queue_size=10)

    vel_msg = Twist()

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while True:
        vel_msg.linear.x = (random.random() - 0.5) * 0.25 * 0.3
        vel_msg.angular.z = random.random() - 0.5

        velocity_publisher.publish(vel_msg)


def move_forward():
    """
    move forward until the robot a wall is 20cm in front of it.
    """
    pub = rospy.Publisher(consts.VEL_TOPIC, Twist, queue_size=10)
    while distance_to_front() > 0.2:
        msg = Twist()
        msg.linear.x = 0.1
        pub.publish(msg)

    msg = Twist()
    msg.linear.x = 0.0
    pub.publish(msg)


def turn_robot(angle):
    """
    turn the robot by a given angle in degrees
    """
    angular_speed = 0.2
    relative_angle = angle * 2 * math.pi / 360

    velocity_publisher = rospy.Publisher(consts.VEL_TOPIC, Twist, queue_size=10)
    vel_msg = Twist()
    # We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    vel_msg.angular.z = angular_speed
    if angle > 0:
        vel_msg.angular.z = -angular_speed
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while abs(current_angle) < abs(relative_angle):
        velocity_publisher.publish(vel_msg)
        time.sleep(0.01)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)

    # stop the robot
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def angle_to_block():
    """
    :return: the angle to the center of an path blocking object. None if no object is available for the camera
    """
    print(consts.CAMERA_TOPIC)
    image = rospy.wait_for_message(consts.CAMERA_TOPIC, Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
    # store the original image for debugging purposes
    cv2.imwrite("/tmp/image.jpg", cv_image)

    hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    frame_threshed = cv2.inRange(hsv_img, consts.COLOR_THRESHOLDS[0], consts.COLOR_THRESHOLDS[1])

    moment = cv2.moments(frame_threshed)

    if not moment["m00"]:
        return None  # object in the given color was not found

    x = int(moment["m10"] / moment["m00"])

    # x is where the object is stored. find the angle
    image_width = cv_image.shape[1]
    l = image_width / 2 / math.tan(CAMERA_FOV / 2)
    angle = math.atan((x - image_width / 2) / l)

    # convert to degrees
    return angle * 180 / math.pi


def distance_to_front():
    """
    :return:  how far the robot is from the object in front of it, in meters
    """
    data = rospy.wait_for_message(consts.SCAN_TOPIC, LaserScan)
    center = min(data.ranges[::10] + data.ranges[-10::])
    return center
