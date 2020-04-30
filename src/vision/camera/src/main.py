#!/usr/bin/env python

"""
Node responsible for taking the camera input from a webcam
and publishing it as a ROS sensor_image

NOTE: This is an interm file to get video data until we setup
DeepStream/GStreamer once the Jetson Nano arrives
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

    # acquire camera and setup topic
    CAM = cv2.VideoCapture(0)
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
