"""
An example script for solving a maze. In general, it should always end with sensing.fuzz
"""
import rospy
import recorder
import time

import sensing


def main():
    rospy.init_node('cnc', anonymous=True)
    time.sleep(1)
    sensing.move_forward()
    sensing.turn_robot(90)
    sensing.move_forward()
    angle = sensing.angle_to_block()
    if angle is not None:
        sensing.turn_robot(angle)
        sensing.move_forward()
        recorder.run_recording("records/push_box.rec")

    sensing.fuzz()


if __name__ == '__main__':
    main()
