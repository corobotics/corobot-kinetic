#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageViewer():
    def __init__( self ):
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/camera/rgb/raw", Image, self.image_call)
	self.images = []

    def image_call(self, data):
	images.append(data)


if __name__ == "__main__":
    ImageViewer()
