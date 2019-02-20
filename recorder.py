"""
This script is used to handle recordings.
The idea behind a recording is to first move the robot using the keyboard, record the keypress
and then to make the same movement actions by the recording. It is done by splitting the time
it takes for the robot to make the action into small segment, and chose for each segment a
single movement command: forward, backward, right or left.

Running this script will let the user create a recording. Then a user may run the recording by
importing it and running the "run_recording" function.
"""
import argparse

import consts
import rospy
import keyboard
import time
import logging
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
import cv2

from consts import SEGMENT_SIZE

__all__ = [
    'run_recording'
]


def run_recording(path):
    """
    Run a recording

    :param path: path to a stored recording
    """
    publisher = rospy.Publisher(consts.VEL_TOPIC, Twist, queue_size=10)
    commands = open(path, 'r').read()

    msgs = build_msgs()

    for i in commands:
        publisher.publish(msgs[i])
        print(repr(i))
        time.sleep(0.2)


def build_msgs():
    """
    :return: A dictionary where the keys are the key pressed, and the values are the matching
    twist messages
    """
    forward = Twist()
    forward.linear.x = 0.2
    forward.linear.y = 0
    forward.linear.z = 0
    forward.angular.x = 0
    forward.angular.y = 0
    forward.angular.z = 0

    backward = Twist()
    backward.linear.x = -0.2
    backward.linear.y = 0
    backward.linear.z = 0
    backward.angular.x = 0
    backward.angular.y = 0
    backward.angular.z = 0

    left = Twist()
    left.linear.x = 0
    left.linear.y = 0
    left.linear.z = 0
    left.angular.x = 0
    left.angular.y = 0
    left.angular.z = 0.5

    right = Twist()
    right.linear.x = 0
    right.linear.y = 0
    right.linear.z = 0
    right.angular.x = 0
    right.angular.y = 0
    right.angular.z = -0.5

    stop = Twist()
    stop.linear.x = 0
    stop.linear.y = 0
    stop.linear.z = 0
    stop.angular.x = 0
    stop.angular.y = 0
    stop.angular.z = 0

    return {
        'w': forward,
        's': backward,
        'a': left,
        'd': right,
        ' ': stop
    }


def parse_args():
    """
    Parse the program args

    :return: fd to the output file
    """
    parser = argparse.ArgumentParser(description="Create a recording by using the wasd keys")
    parser.add_argument("path", help="path to store the recording", type=argparse.FileType('w'))
    return parser.parse_args().path


def main():
    record = parse_args()
    rospy.init_node('cnc', anonymous=True)
    time.sleep(1)
    publisher = rospy.Publisher(consts.VEL_TOPIC, Twist, queue_size=10)

    msgs = build_msgs()

    print("start to log key presses")

    while not keyboard.is_pressed('escape'):  # stop when esc key is pressed
        chosen = ' '
        for i in msgs:
            if keyboard.is_pressed(i):
                chosen = i
                break

        publisher.publish(msgs[chosen])
        record.write(chosen)
        print(repr(chosen))
        time.sleep(SEGMENT_SIZE)

    record.flush()


if __name__ == '__main__':
    main()
