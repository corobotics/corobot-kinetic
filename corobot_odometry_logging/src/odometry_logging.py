#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_odometry_logging")
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

FEATURES = {
    'odom': None,
    'cmd_vel': None
}

def odom_callback(odom):
    global FEATURES
    FEATURES['odom'] = odom

def cmd_vel_callback(vel):
    global FEATURES
    print(vel)
    FEATURES['cmd_vel'] = vel

def get_feature_vector():
    if not FEATURES['odom'] or not FEATURES['cmd_vel']:
        return None
    return [
        FEATURES['odom'].pose.pose.position.x,
        FEATURES['odom'].pose.pose.position.y,
        FEATURES['odom'].pose.pose.orientation.z,
        FEATURES['odom'].pose.pose.orientation.w,
        FEATURES['cmd_vel'].linear.x,
        FEATURES['cmd_vel'].linear.y,
        FEATURES['cmd_vel'].angular.z
    ]

def log_status():
    rospy.loginfo(get_feature_vector())

def main():
    rospy.init_node("odometry_logging")
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("/mobile_base/commands/velocity", Twist, cmd_vel_callback)

    while not rospy.is_shutdown():
        log_status()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
