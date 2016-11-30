#!/usr/bin/env python

"""
    Calculates the position of another robot based on kinect input and current odometry
"""
import rospy
import cv2
import socket
from sensor_msgs.msg import Image, LaserScan
from corobot_common.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np

"""
    Calculates the distance between two points in n space
    a: A vector describing 1 point in n space
    b: A vector describing another point in n space
    return: the euclidean distance between the two points
"""
def dist( a, b ):
    summ = 0
    for (x1, x2) in zip(a, b):
        summ += (x1 - x2)**2
    return np.sqrt(summ)

class ImageViewer():
    """
	Initializes the image viewing node
    """
    def __init__( self ):
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_call)
	self.pos_sub = rospy.Subscriber("/pose", Pose, self.odom_call)
	self.scan = rospy.Subscriber("/scan", LaserScan, self.scan_call)
	self.images = []
	self.pos = []
	self.scan = []
	self.seq = "RRGRBRKGGBGKBBKKR"
	self.colorRecord = open('colors.txt', 'a+')

    """
	Collects all of the input images
    """
    def image_call(self, data):
	self.images.append(data)

    """
	Collects all of the past odometry information
    """
    def odom_call(self, data):
	self.pos.append( data )

    """
	Collects all of the past laserscan data
    """
    def scan_call(self, data):
	self.scan.append( data )

    """
	Processes the most recent information to generate a position estimate if it can
    """
    def processImage(self, junk):
	while not rospy.is_shutdown():
	    if len(self.images) == 0 or len(self.pos) == 0 or len(self.scan) == 0:
		continue
	    data = self.images.pop()
	    scan = self.scan.pop()
	    pos = self.pos.pop()
	    self.images = []
	    self.scan = []
	    self.pos = []
	    try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")[360:]
	    except CvBridgeError as e:
		print e
	    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	    edges = cv2.Canny(gray_image, 300, 600)
	    # Try to find the circles in an image
	    circles = cv2.HoughCircles( edges, cv2.HOUGH_GRADIENT, 1, 20, 100, 50, 5, 10 )
	    output = cv_image.copy()
	    if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		circles = filter((lambda x: x[2] < 30), circles)[:min(10, len(circles))]
		for i in range(len(circles)):
		     robot = []
                     robot.append(circles[i])
		     for j in range(i + 1, len(circles)):
			for (rx, ry, rr) in robot:
			    if circles[j][0] >= rx - 2.5*rr and circles[j][0] <= rx + 2.5*rr and \
			        circles[j][1] >= ry - rr and circles[j][1] <= ry + rr:
				robot.append(circles[j])
				break
		     if len(robot) >= 3:
			colorstring = ""
			robot = sorted(robot[:3], key = lambda x: x[0] )
			for (rx, ry, rr) in robot:
		            cv2.circle(output, (rx,ry), rr, (0, 255, 0), 1)
		            cv2.circle(output, (rx,ry), 1, (255, 0, 0), 1)
			    #cv2.putText(cv_image, self.getColor(rx, ry, rr, cv_image), (rx, ry), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
			    colorstring += self.getColor(rx, ry, rr, cv_image)
			if "W" in colorstring:
			    break
			phi_obs = self.seq.index(colorstring[:2]) * np.pi / 8.0
			print phi_obs, colorstring[:2]
			d = scan.ranges[max(len(scan.ranges) - robot[0][0] - 20, 0): min(len(scan.ranges) - robot[0][0]+20, len(scan.ranges))]
			d = np.nanmean(d)
			if np.isnan(d):
			    print( 'No Object Observed in this direction' )
			    break
			camang = scan.angle_min + (len(scan.ranges) - robot[0][0]) * scan.angle_increment
			px = pos.x
			py = pos.y
			ptheta = pos.theta
			print("ME:")
			print(px, py, ptheta)
			
			ox = px + d * np.cos( ptheta + camang )
			oy = py + d * np.sin( ptheta + camang )
			otheta = (np.pi - phi_obs) + (ptheta + camang)
			print ("THEM: ")
			print (ox, oy, otheta)
			print ( scan.header.stamp )
			rospy.loginfo( "Time: %s\t( %s, %s ) at %s radians from ( %s, %s )", str(scan.header.stamp), str(ox), str(oy), str(otheta), str(px), str(py) )
			break	
	    cv2.imshow( "Test", output)
	    cv2.waitKey(3)

    def getColor(self, x, y, radius, image ):
	img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)

	mask = np.zeros(image.shape[:2], dtype="uint8")
	cv2.circle( mask, (x, y), 5, 255, -1)
	col = cv2.mean(img_lab, mask = mask)[:3]
        if col[1] >= 140:
	    return 'R'
	elif col[1] <= 125:
	    return 'G'
	elif col[2] <= 120:
	    return 'B'
        elif col[0] <= 40:
	    return 'K'
	else:
	    return 'W'

if __name__ == "__main__":
    iv = ImageViewer()
    rospy.init_node('image_viewer', anonymous = True )
    rospy.Timer( rospy.Duration(0.1), iv.processImage, oneshot= True )
    try:
	rospy.spin()
    except KeyboardInterrupt:
	pass
    cv2.destroyAllWindows()
	
