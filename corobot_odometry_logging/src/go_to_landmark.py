import rospy
import sys
from corobot_common.srv import GetLandmark
from geometry_msgs.msg import Point

"""
WInter,74.1936,37.392
VendDoor,80.688,37.5888

Office3519,269,61.008,
Office3509,70.5856,38.4088

ICL1,56.6128,35.916
ICL3,34.8336,35.916
"""

def main():
    if len(sys.argv < 2) or len(sys.argv > 3):
        print('Usage: go_to_landmark LOC or go_to_landmark x y')
    else:
        rospy.init_node("go_to_landmark")

        if len(sys.argv == 2):
            rospy.wait_for_service("get_landmark")
            get_landmark = rospy.ServiceProxy("get_landmark", GetLandmark)
            landmark = get_landmark(sys.argv[1])
            x = landmark.wp.x
            y = landmark.wp.y
        else:
            x = float(sys.argv[1])
            y = float(sys.argv[2])

        # possible topics are goals and goals-nav
        goals_nav_pub = rospy.Publisher("goals", Point, queue_size=10)
        goals_nav_pub.publish(x=x, y=y)
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass