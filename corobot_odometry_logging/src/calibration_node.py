#!/usr/bin/env python

import roslib; roslib.load_manifest("corobot_calibration_node")
import rospy
from nav_msgs.msg import Odometry
from corobot_common.msg import Pose

import math

LAST_ODOM_MEASUREMENT = None
CURRENT_POSE = Pose()
CALIBRATION_PUBLISHER = None
LOG_HERTZ = 10

LINEAR_CALIBRATION = 1.0
ANGULAR_CALIBRATION = 1.0
LIN_ANGULAR_CALIBRATION = 1.0

def odom_callback(odom):
    global LAST_ODOM_MEASUREMENT
    global CURRENT_POSE

    reported_pose = odom_to_pose(odom)
    delta_x = reported_pose.x - LAST_ODOM_MEASUREMENT.x
    delta_y = reported_pose.y - LAST_ODOM_MEASUREMENT.y
    delta_theta = reported_pose.theta - LAST_ODOM_MEASUREMENT.theta
    reported_linear_distance = euclidean_distance(0, 0, delta_x, delta_y)

    calibrated_linear_distance = LINEAR_CALIBRATION * reported_linear_distance
    calibrated_theta = ANGULAR_CALIBRATION * delta_theta \
                        + (LIN_ANGULAR_CALIBRATION * reported_linear_distance)
    angle_average = 0.5 * (CURRENT_POSE.theta + calibrated_theta)
    calibrated_x = calibrated_linear_distance * math.cos(angle_average)
    calibrated_y = calibrated_linear_distance * math.sin(angle_average)
    calibrated_pose = Pose(CURRENT_POSE.x + calibrated_x, CURRENT_POSE.y + calibrated_y,
                           CURRENT_POSE.theta + calibrated_theta)

    LAST_ODOM_MEASUREMENT = reported_pose
    CURRENT_POSE = calibrated_pose
    CALIBRATION_PUBLISHER.publish(CURRENT_POSE)

def odom_to_pose(odom):
    pose = Pose()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y,
    pose.theta = quaternion_to_angle(odom.pose.pose.orientation.z,
                                    odom.pose.pose.orientation.w)
    return pose

def euclidean_distance(p1_x, p1_y, p2_x, p2_y):
    return math.sqrt(math.pow(p1_x - p2_x, 2) + math.pow(p1_y + p2_y, 2))

def quaternion_to_angle(qw, qz):
    return math.atan2(2 * qw * qz, 1 - 2 * qz * qz)

def main():
    global CALIBRATION_PUBLISHER
    rospy.init_node("corobot_calibration_node")
    rospy.Subscriber("odom", Odometry, odom_callback)
    CALIBRATION_PUBLISHER = rospy.Publisher('corobot_calibration_node', Odometry, queue_size=1)

    sleep_rate = rospy.Rate(LOG_HERTZ)
    while not rospy.is_shutdown():
        sleep_rate.sleep()
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
