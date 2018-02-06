#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_odometry_logging")
import rospy

FEATURES = {
    'odom': None;
}

def odom_callback(odom):
    global FEATURES
    FEATURES['odom'] = odom

def get_feature_vector():
    if not FEATURES['odom']:
        return None
    return [
        FEATURES['odom'].pose.pose.position.x,
        FEATURES['odom'].pose.pose.position.y,
        FEATURES['odom'].pose.pose.orientation.z,
        FEATURES['odom'].pose.pose.orientation.w
    ]

def log_status():
    rospy.loginfo(

def main():
    rospy.init_node("odometry_logging")
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Timer(rospy.Duration(0.01), log_status)
    rospy.spin()
