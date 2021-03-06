#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_odometry_logging")
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from corobot_common.msg import Pose

FEATURES = {
    'odom': None,
    'cmd_vel': None,
    'qr_pose': Pose(),
    'barcode_loc': Pose(),
}
LOG_HERTZ = 10

def odom_callback(odom):
    global FEATURES
    FEATURES['odom'] = odom

def cmd_vel_callback(vel):
    global FEATURES
    FEATURES['cmd_vel'] = vel

def qrcode_pose_callback(qr_pose):
    global FEATURES
    FEATURES['qr_pose'] = qr_pose

def barcode_loc_callback(barcode_loc):
    global FEATURES
    FEATURES['barcode_loc'] = barcode_loc

def get_feature_vector():
    try:
        return [
            FEATURES['odom'].pose.pose.position.x,
            FEATURES['odom'].pose.pose.position.y,
            FEATURES['odom'].pose.pose.orientation.z,
            FEATURES['odom'].pose.pose.orientation.w,
            FEATURES['cmd_vel'].linear.x,
            FEATURES['cmd_vel'].linear.y,
            FEATURES['cmd_vel'].angular.z,
            FEATURES['qr_pose'].x,
            FEATURES['qr_pose'].y,
            FEATURES['qr_pose'].theta,
	    FEATURES['barcode_loc'].x,
            FEATURES['barcode_loc'].y,
        ]
    except Exception as e:
        print(e)
        return None

def log_status():
    vector = get_feature_vector()
    if vector:
        rospy.loginfo(vector)

def main():
    rospy.init_node("odometry_logging")
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/commands/velocity", Twist, cmd_vel_callback)
    rospy.Subscriber("qrcode_pose", Pose, qrcode_pose_callback)
    rospy.Subscriber("barcode_location", Pose, barcode_loc_callback)

    sleep_rate = rospy.Rate(LOG_HERTZ)
    while not rospy.is_shutdown():
        log_status()
        sleep_rate.sleep()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
