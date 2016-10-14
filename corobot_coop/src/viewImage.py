#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np

def dist( a, b ):
    summ = 0
    for (x1, x2) in zip(a, b):
        summ += (x1 - x2)**2
    return np.sqrt(summ)

class ImageViewer():
    def __init__( self ):
	self.bridge = CvBridge()
	self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_call)
	self.pos_sub = rospy.Subscriber("/odom", Odometry, self.odom_call)
	self.scan = rospy.Subscriber("/scan", LaserScan, self.scan_call)
	self.images = []
	self.pos = []
	self.scan = []
	self.seq = "RRGRBRKGGBGKBBKKR"

    def image_call(self, data):
	self.images.append(data)

    def odom_call(self, data):
	self.pos.append( data.pose.pose )

    def scan_call(self, data):
	self.scan.append( data )

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
	    edges = cv2.Canny(gray_image, 100, 200)
	    circles = cv2.HoughCircles( edges, cv2.HOUGH_GRADIENT, 1, 10, 100, 50, 1, 5 )
	    if circles is not None:
		circles = np.round(circles[0, :]).astype("int")[:7]
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
		            #cv2.circle(cv_image, (rx,ry), rr, (0, 255, 0), 1)
		            #cv2.circle(cv_image, (rx,ry), 1, (255, 0, 0), 1)
			    #cv2.putText(cv_image, self.getColor(rx, ry, rr, cv_image), (rx, ry), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
			    colorstring += self.getColor(rx, ry, rr, cv_image)
			if "W" in colorstring:
			    break
			phi_obs = self.seq.index(colorstring[:2]) * np.pi / 8.0
			
			d = scan.ranges[max(len(scan.ranges) - robot[0][0] - 10, 0): min(len(scan.ranges) - robot[0][0]+10, len(scan.ranges))]
			d = np.nanmean(d)
			if np.isnan(d):
			    break
			camang = scan.angle_min + (len(scan.ranges) - robot[0][0]) * scan.angle_increment
			px = pos.position.x
			py = pos.position.y
			qw = pos.orientation.w
			qz = pos.orientation.z
			ptheta = np.arctan2(2 * qw * qz, 1 - 2 * qz * qz)
			print("ME:")
			print(px, py, ptheta)
			
			ox = px + d * np.cos( ptheta + camang )
			oy = py + d * np.sin( ptheta + camang )
			otheta = (np.pi - phi_obs) + (ptheta + camang)
			print ("THEM: ")
			print (ox, oy, otheta)
			break	
	
	    cv2.imshow( "Test", cv_image)
	    cv2.waitKey(3)

    def getColor(self, x, y, radius, image ):
	img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
	colors = np.zeros((5, 1, 3), dtype = "uint8")
	colors[0] = [100, 30, 20] #Red
	colors[1] = [30, 40, 30] #Green
	colors[2] = [30, 50, 80] #Blue
	colors[3] = [20, 20, 20] #Black
	colors[4] = [128, 128, 128] #White

	lab = cv2.cvtColor(colors, cv2.COLOR_RGB2LAB)
	minDist = (np.inf, "?")

	mask = np.zeros(image.shape[:2], dtype="uint8")
	cv2.circle( mask, (x, y), 5, 255, -1)
	col = cv2.mean(img_lab, mask = mask)[:3]
	for i in range(len(lab)):
            dis = dist(col, lab[i][0].astype('double'))
	    if minDist[0] > dis:
		if i == 0:
		    minDist = (dis, "R")
		elif i == 1:
		    minDist = (dis, "G")
		elif i == 2:
		    minDist = (dis, "B")
		elif i == 3:
		    minDist = (dis, "K")
		else:
		    minDist = (dis, "W")

	return minDist[1]

if __name__ == "__main__":
    iv = ImageViewer()
    rospy.init_node('image_viewer', anonymous = True )
    rospy.Timer( rospy.Duration(0.1), iv.processImage, oneshot= True )
    try:
	rospy.spin()
    except KeyboardInterrupt:
	pass
    cv2.destroyAllWindows()
	
