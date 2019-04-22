# #!/usr/bin/env python

# Imports are here.
import numpy
import math
import random
import rospy
import copy

# From ... import ... are here.
from corobot_common.msg import Pose
from corobot_common.msg import Target
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ekf_slam import EKFSLAM

from corobot_slam.src.montecarlo import MonteCarlo
from corobot_localization.src.ekf import EKF
from utils import odom_to_pose

# Global variables are here.
ekf = EKF()
smart_slam = EKFSLAM()
traditional_slam = EKFSLAM()
seen_lms = dict()
accepted_lms = dict()
laser_landmark = 0
robot_poses = list()
goals = list()
scale = 30
accept_capacity = 50

global current_pose
global smart_lm


# Class definitions are here.
class LandmarkRL(MonteCarlo):
    """
    Implements the paper version RL algorithm for handling landmarks.
    """
    __slots__ = {"sar", "states", "actions", "kis", "epsilon", "prior_reward", "k"}

    def __init__(self, k):
        """
        Constructor of landmark choosing algorithm.
        """
        self.sar = dict()
        self.states = list()
        self.actions = ["Accept", "Reject"]
        self.kis = [random.uniform(0, 0.5), random.uniform(0.5, 1)]
        self.epsilon = 1 / (len(self.sar) + 1)
        self.prior_reward = 0
        self.k = k

        for each_action in self.actions:
            self.sar[each_action] = dict()

    def policy(self, state, params):
        """
        Choose action based on given state. Used KNN algorithm to find action.
        :param state: The new state based on which an action is to be taken.
        :param params: A list of other parameters
        :return action: The action to be taken.
        """
        landmark_cap, visible_lm = params[0], params[1]
        action_reward = dict()
        # Search K nearest neighboring states.
        for one_action in self.sar.keys():
            sr_dict = self.sar[one_action]
            action_reward[one_action] = self.get_neighbor_rewards(state, sr_dict)

        # Here is policy
        self.epsilon = 1 / (len(self.sar) + 1)
        if action_reward["Accept"] != action_reward["Reject"] and self.kis[0] <= 1 - self.epsilon:
            action = max(action_reward)
        elif (action_reward["Accept"] == action_reward["Reject"] or self.kis[0] <= self.epsilon) \
            and self.kis[1] < (landmark_cap / visible_lm):
            action = "Accept"
        else:
            action = "Reject"

        self.sar[action][state] = self.prior_reward

        return action

    def get_neighbor_rewards(self, new_state, sr_dict):
        """
        Based on given state-reward dictionary, calculate the average reward.
        :param new_state: The new state w.r.t. which needs to take an action
        :param sr_dict: Dictionary containing information of states and their corresponding rewards.
        :return avg_rewards:
        """
        neighbor_rewards = dict()
        if sr_dict:
            avg_reward = 0
            for each_state in sr_dict.keys():
                euc_dist = math.sqrt((each_state[0] - new_state[0]) ** 2
                                     + (each_state[1] - new_state[1]) ** 2
                                     + (each_state[2] - new_state[2]) ** 2
                                     + (each_state[3] - new_state[3]) ** 2
                                     + (each_state[4] - new_state[4]) ** 2)
                if len(neighbor_rewards) < self.k:
                    neighbor_rewards[each_state] = euc_dist
                else:
                    max_key = max(neighbor_rewards)
                    if euc_dist < neighbor_rewards[max_key]:
                        neighbor_rewards.pop(max_key)
                        neighbor_rewards[each_state] = euc_dist

            for each_neighbor in neighbor_rewards.keys():
                avg_reward += neighbor_rewards[each_neighbor] * (1 / self.k)
        else:
            avg_reward = self.prior_reward

        return avg_reward

    def update_reward(self, state_action, reward):
        state, action = state_action[0], state_action[1]
        self.sar[action][state] = reward


class Landmark:
    __slots__ = {"name", "x", "y", "orientation"}

    def __init__(self, name, pose):
        self.name = name
        self.x = pose[0]
        self.y = pose[1]
        # self.orientation = pose[2]

    def pose_update(self, new_pose):
        self.x = new_pose[0]
        self.y = new_pose[1]
        # self.orientation = new_pose[2]


# Functions are here.
def calc_lmpose(robot_pose, dist, angle, camera_angle):
    """
    Calculate the pose of a landmark, given robot's pose and its camera readings.
    :param robot_pose: Robot pose.
    :param dist: Range reading given by robot camera.
    :param angle: Angle reading given by robot camera.
    :param camera_angle: The angle between camera's orientation and robot's orientation. -pi / 2 or pi / 2.
    :return: Pose of a landmark.
    """
    lm_pose = Pose()
    lm_pose.header.frame_id = "landmark"
    robot_lm_angle = robot_pose.theta + camera_angle + angle
    lm_pose.x = math.cos(robot_lm_angle) * dist
    lm_pose.y = math.sin(robot_lm_angle) * dist
    lm_pose.theta = 0
    lm_pose.theta = 0

    return lm_pose


def calc_states(robot_pose, lm_pose, yaw_angle):
    global seen_lms
    states = list()

    if goals:
        subgoal = goals.pop(0)
        goal_dist = math.sqrt((subgoal[0] - robot_pose.x) ** 2 + (subgoal[1] - robot_pose.y) ** 2)
    else:
        dist = 0

    # First dimension of states. Distance to subgoal.
    states.append(goal_dist)
    # Second dimension of states. Number of accepted landmarks.
    states.append(len(accepted_lms))
    # Third dimension of states. Yaw angle of new landmark.
    states.append(yaw_angle)
    # Fourth dimension of states. Distance of new landmark to closest landmark.
    temp_dist_dict = dict()
    for each_landmark in seen_lms.keys():
        one_landmark = seen_lms[each_landmark]
        distance = math.sqrt((one_landmark.x - robot_pose.x) ** 2 + (one_landmark.y - robot_pose.y) ** 2)
        temp_dist_dict[each_landmark] = distance

    closest_lm = seen_lms[min(temp_dist_dict)]
    dim_four = math.sqrt((closest_lm.x - lm_pose.x) ** 2 + (closest_lm.y - lm_pose.y) ** 2)
    states.append(dim_four)
    # Fifth dimension of states. Uncertainty of robot pose in terms of entropy
    dim_five = math.log(math.sqrt((2 * math.pi * math.e) ** 3 * numpy.linalg.det(robot_pose.cov)))
    states.append(dim_five)

    return states


def landmark_callback(qrcode_info):
    global laser_landmark, current_pose, smart_lm

    robot_pose = copy.deepcopy(current_pose)
    landmark_name = qrcode_info.name
    landmark_dist = qrcode_info.dist
    landmark_angle = qrcode_info.angle
    if qrcode_info.camera_id == 0:
        camera_angle = math.pi / 2
    else:
        camera_angle = -math.pi / 2
    landmark_pose = calc_lmpose(robot_pose, landmark_dist, landmark_angle, camera_angle)

    new_lm = Landmark(landmark_name, landmark_pose)

    seen_lms[new_lm.name] = new_lm
    yaw_angle = camera_angle + landmark_angle
    new_lm_states = calc_states(robot_pose, landmark_pose, yaw_angle)

    # Use RL to decide whether to take this landmark.
    action = smart_lm.policy(new_lm_states, [accept_capacity, len(accepted_lms)])
    if action == "Accept":
        accepted_lms.append(new_lm)


def update_map(pose):
    global ekf

    ekf.update_pos(pose)
    current_pose = ekf.get_pose()


def odom_callback(odometry):
    """
    Handles the odometry messages. Once receiving an odometry, calculates the current pose of robot.
    :param odometry: The odometry message.
    :return: None
    """
    global ekf, current_pose, smart_slam, traditional_slam

    odom_pose = odom_to_pose(odometry)
    ekf.predict(odom_pose)
    # current_pose is set as the latest known pose of robot.
    current_pose = ekf.get_pose()
    # robot_poses records the trajectory of the robot.
    last_pose = current_pose -
    robot_poses.append(current_pose)
    last_pose =

    smart_slam.predict()


def corobot_smart_slam():
    """
    The running of using SLAM algorithm with Reinforcement Learning.
    :return:
    """
    pass


def run_experiment():
    """
    Main function for running experiment. Robot will run smart SLAM and traditional SLAM at the same time.
    :return:
    """
    global goals, smart_lm

    rospy.init_node("smart_slam_exp")

    goal_file = "/home/oralas/Documents/Capstone/Goal Info"
    goals_info = open(goal_file)

    for lines in goals_info:
        x, y = int(lines.split(",")[0]), int(lines.split(",")[1])
        goals.append([x, y])

    smart_lm = LandmarkRL(15)

    rospy.Subscriber("landmark_info", Target, landmark_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    binary_model = True


if __name__ == "__main__":
    try:
        corobot_smart_slam()
    except rospy.ROSInterruptException:
        pass
