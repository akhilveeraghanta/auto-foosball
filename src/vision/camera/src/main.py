#!/usr/bin/env python

"""
Node responsible for taking the camera input from a webcam
and publishing it as a ROS sensor_image

"""

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#######################################################################
#                                Main                                 #
#######################################################################
if __name__ == "__main__":

    # start the node
    rospy.init_node("camera")

    gst_pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), "+\
                   "width=(int)1280, height=(int)720, format=(string)NV12, "+\
                   "framerate=(fraction)30/1 ! nvvidconv flip-method=2 ! video/x-raw,"+\
                   "format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

    # acquire camera and setup topic
    CAM = cv2.VideoCapture(gst_pipeline)
    CAMERA_TOPIC = rospy.Publisher('/camera', Image, queue_size=1)

    # publish frames
    while True:
        ret, frame = CAM.read()
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        CAMERA_TOPIC.publish(image_message)

        if not ret:
            break

    CAM.release()
