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
	self.colorRecord = open('colors.txt', 'a+')


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
	    circles = cv2.HoughCircles( edges, cv2.HOUGH_GRADIENT, 1, 20, 100, 50, 5, 10 )
	    output = cv_image.copy()
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
		            cv2.circle(output, (rx,ry), rr, (0, 255, 0), 1)
		            cv2.circle(output, (rx,ry), 1, (255, 0, 0), 1)
			    #cv2.putText(cv_image, self.getColor(rx, ry, rr, cv_image), (rx, ry), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
			    colorstring += self.getColor(rx, ry, rr, cv_image)
			if "W" in colorstring:
			    break
			phi_obs = self.seq.index(colorstring[:2]) * np.pi / 8.0
			print phi_obs, colorstring[:2]
			d = scan.ranges[max(len(scan.ranges) - robot[0][0] - 10, 0): min(len(scan.ranges) - robot[0][0]+10, len(scan.ranges))]
			d = np.nanmean(d)
			if np.isnan(d):
			    print( 'No Object Observed in this direction' )
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
	    cv2.imshow( "Test", output)
	    cv2.waitKey(3)

    def getColor(self, x, y, radius, image ):
	img_lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
	colors = np.zeros((20, 1, 3), dtype = "uint8")
	colors[0] = [92, 171, 154] #Red
	colors[1] = [156, 144, 141] #Red
	colors[2] = [109, 165, 149] #Red
	colors[3] = [137, 152, 142] #Red
	colors[4] = [66, 119, 135] #Green
	colors[5] = [78, 118, 136] #Green
	colors[6] = [72, 130, 111] #Blue
	colors[7] = [98, 130, 110] #Blue
	colors[8] = [37, 129, 129] #Black
	colors[9] = [173, 130, 135] #White
	colors[10] =  [108, 129, 131] #White
	colors[11] = [150, 129, 140] #White?
	colors[12] = [218, 128, 131] #White
	colors[13] = [138, 128, 134] #White
	colors[14] = [91, 126, 133] #White?
	colors[15] = [199, 129, 133] #White
	colors[16] = [65, 131, 130] #White? Could be black
	colors[17] = [159, 128, 133]
	colors[18] = [188, 130, 132]
	colors[19] = [123, 128, 133]

	minDist = (np.inf, "?")
	lab = colors
	mask = np.zeros(image.shape[:2], dtype="uint8")
	cv2.circle( mask, (x, y), 5, 255, -1)
	col = cv2.mean(img_lab, mask = mask)[:3]
	self.colorRecord.write( str(col[0]) + " " + str(col[1]) + " "+ str(col[2]) + "\n" );
	for i in range(len(lab)):
            dis = dist(col, lab[i][0].astype('double'))
	    if minDist[0] > dis:
		if i >= 0 and i<= 3:
		    minDist = (dis, "R")
		elif i == 4 or i == 5:
		    minDist = (dis, "G")
		elif i == 6 or i == 7:
		    minDist = (dis, "B")
		elif i == 8:
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
	
