import numpy as np

VEL_TOPIC = "turtle1/cmd_vel"
CAMERA_TOPIC = "/usb_cam/image_raw"
SCAN_TOPIC = "/scan"

# Thresholds for detecting the block
COLOR_THRESHOLDS = [np.array([100, 50, 50]), np.array([110, 200, 200])]

# The size of the segments of the recorder
SEGMENT_SIZE = 0.2
