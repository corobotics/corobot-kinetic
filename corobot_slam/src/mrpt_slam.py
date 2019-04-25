# #!/usr/bin/env python

# Imports are here.
import numpy
import math
import random
import rospy
import copy

# From ... import ... are here.
from corobot_common.msg import Target
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from ekf_slam import EKFSLAM
from corobot_slam.src.montecarlo import MonteCarlo
from corobot_localization.src.ekf import EKF
from collections import namedtuple

# Global variables are here.
ekf = EKF()
smart_slam = EKFSLAM()
traditional_slam = EKFSLAM()
goals = list()
tolerance = 0.2
batch = 1

global landmark_choosing


# Class definitions are here.
class Landmark:
    __slots__ = {"name", "x", "y", "cov"}

    def __init__(self, name, x, y):
        self.name = name
        self.x = x
        self.y = y
        self.cov = numpy.identity(2, numpy.int16)


class LandmarkChoice(MonteCarlo):
    """
    Implements the paper version RL algorithm for handling landmarks.
    """
    __slots__ = {"sar", "states", "actions", "kis", "epsilon", "prior_reward", "k"}

    def __init__(self, k):
        """
        Constructor of landmark choosing algorithm.
        """
        self.sar = dict()   # sar holds the rewards for state-action pairs.
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
        :param state: The new state based on which an action is to be taken. It's a tuple.
        :param params: A list containing [capacity of integrated landmarks, total number of landmarks]
        :return action: The action to be taken.
        """
        integrate_capacity, number_lms = params[0], params[1]
        action_reward = dict()
        # Search K nearest neighboring states.
        for one_action in self.sar.keys():
            sr_dict = self.sar[one_action]  # sr_dict takes state as keys and rewards as values.
            action_reward[one_action] = self.get_neighbor_rewards(state, sr_dict)

        # Here is policy
        self.epsilon = 1 / (len(self.sar) + 1)
        if action_reward["Accept"] != action_reward["Reject"] and self.kis[0] <= 1 - self.epsilon:
            action = max(action_reward)
        elif (action_reward["Accept"] == action_reward["Reject"] or self.kis[0] <= self.epsilon) and \
                self.kis[1] < (integrate_capacity / number_lms):
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


# Functions are here.
def calc_lmpose(robot_pose, observation):
    """
    Calculate the pose of a landmark, given robot's pose and its camera readings.
    :param robot_pose: Robot pose.
    :param observation: Range reading given by robot camera.
    :return: Pose of a landmark.
    """
    lm_x = observation.dist * math.cos(observation.angle + robot_pose[2]) + robot_pose[0]
    lm_y = observation.dist * math.sin(observation.angle + robot_pose[2]) + robot_pose[1]
    observed_lm_loc = numpy.matrix([lm_x, lm_y]).transpose()

    return observed_lm_loc


def calc_states(slam, observation):
    """
    Calculate the states for reinforcement learning.
    :param slam: The SLAM algorithm being used here.
    :param observation: Observation of an unintegrated landmark. Contains name, dist, angle, and cov.
    :return:
    """
    state_list = list()

    robot_info = slam.get_robot_pose()
    robot_pose, robot_cov = robot_info[0], robot_info[1]

    if goals:
        subgoal = goals[0]
        goal_dist = math.sqrt((subgoal[0] - robot_pose[0].item()) ** 2 + (subgoal[1] - robot_pose[1].item()) ** 2)
    else:
        goal_dist = 0

    # First dimension of states. Distance to subgoal.
    state_list.append(goal_dist)
    # Second dimension of states. Number of accepted landmarks.
    state_list.append(len(slam.get_integrated_lm()))
    # Third dimension of states. Yaw angle of new landmark.
    state_list.append(observation.angle)
    # Fourth dimension of states. Distance of new landmark to closest landmark.
    potential_lm_loc = calc_lmpose(robot_pose, observation)
    first_lm_loc = slam.get_landmark_loc(min(slam.get_integrated_lm()))[0]
    shortest_dist = get_distance(potential_lm_loc, first_lm_loc)
    for one_lm in slam.get_integrated_lm().keys():
        landmark_loc = slam.get_landmark_loc(one_lm)[0]
        if get_distance(potential_lm_loc, landmark_loc) < shortest_dist:
            shortest_dist = get_distance(potential_lm_loc, landmark_loc)
    state_list.append(shortest_dist)
    # Fifth dimension of states. Uncertainty of robot pose in terms of entropy
    dim_five = math.log(math.sqrt((2 * math.pi * math.e) ** 3 * numpy.linalg.det(robot_cov)))
    state_list.append(dim_five)

    state = tuple(state_list)
    return state


def get_distance(point_1, point_2):
    """
    Calculate the Euclidean distance between point_1 and point_2
    :param point_1: Coordinates of point_1 in form of a column vector.
    :param point_2: Coordinates of point_2 in form of a column vector.
    :return: The comptuted Euclidean distance.
    """
    delta = point_1 - point_2
    distance = math.sqrt(numpy.dot(delta.transpose(), delta).item())

    return distance


def mobilize_robot(robot_pose):
    global tolerance, goals

    twist = Twist()
    # twist.linear = Vector3()
    twist.linear.x = numpy.float64(0.0)
    twist.linear.y = numpy.float64(0.0)
    twist.linear.z = numpy.float64(0.0)
    # twist.angular = Vector3()
    twist.angular.x = numpy.float64(0.0)
    twist.angular.y = numpy.float64(0.0)
    twist.angular.z = numpy.float64(0.0)

    next_goal = goals[0]
    distance = math.sqrt((robot_pose.x - next_goal[0]) ** 2 + (robot_pose.y - next_goal[1]) ** 2)
    if distance <= tolerance:
        goals.pop(0)
    else:
        # Calculate rotation based on destination
        target_angle = math.atan2((next_goal[1] - robot_pose.y), (next_goal[0] - robot_pose.x))
        rotation = target_angle - robot_pose.theta
        # twist.linear.x = numpy.float64(math.cos(target_angle) * 0.5)
        # twist.linear.y = numpy.float64(math.sin(target_angle) * 0.5)
        twist.linear.x = numpy.float64(0.1)
        # twist.linear.y = numpy.float64(math.sin(target_angle) * 0.5)
        if rotation != 0:
            twist.angular.z = numpy.float64(math.pi * math.copysign(0.1, rotation))

    return twist


def log_info(batch_number):
    t_log_name = "traditional_slam_log.txt"
    s_log_name = "smart_slam_log.txt"

    tradition_log = open(t_log_name, "a")
    smart_log = open(s_log_name, "a")

    tradition_log.write("Batch %d:\n" % batch_number)
    smart_log.write("Batch %d:\n" % batch_number)

    t_r_pose = traditional_slam.get_robot_pose()[0]
    tradition_log.write("Robot pose:\n" + str(t_r_pose) + "\n")
    s_r_pose = smart_slam.get_robot_pose()[0]
    smart_log.write("Robot pose:\n" + str(s_r_pose) + "\n")

    for each_lm in traditional_slam.get_integrated_lm().keys():
        lm_name = each_lm
        if lm_name in smart_slam.get_integrated_lm():
            t_lm_loc = traditional_slam.get_landmark_loc(lm_name)[0]
            s_lm_loc = smart_slam.get_landmark_loc(lm_name)[0]

            tradition_log.write("Landmark " + str(lm_name) + " location: \n" + str(t_lm_loc) + "\n")
            smart_log.write("Landmark " + str(lm_name) + " location: \n" + str(s_lm_loc) + "\n")


def landmark_callback(qrcode_info):
    global landmark_choosing, traditional_slam, smart_slam, batch

    lm_readings = namedtuple("LMReading", "name, dist, angle, cov")
    lm_readings.name = qrcode_info.name
    lm_readings.dist = qrcode_info.dist
    if qrcode_info.camera_id == 0:
        lm_readings.angle = qrcode_info.angle + math.pi / 2
    else:
        lm_readings.angle = qrcode_info.angle - math.pi / 2
    lm_readings.cov = numpy.identity(2, numpy.int16)

    # t_robot_pose = copy.deepcopy(traditional_slam.get_robot_pose()[0])

    # Traditional SLAM will update with the landmark info no matter what.
    traditional_slam.update(lm_readings)

    # Smart SLAM will evaluate to see whether to use this landmark or not.
    s_robot_info = copy.deepcopy(smart_slam.get_robot_pose())
    s_robot_pose, s_robot_cov = s_robot_info[0], s_robot_info[1]
    number_integrated_lm = len(smart_slam.get_integrated_lm())

    calc_states(smart_slam, lm_readings)

    # Use RL to decide whether to take this landmark.
    if lm_readings.name in smart_slam.get_integrated_lm:
        smart_slam.update(lm_readings)
    else:
        new_lm_states = calc_states(s_robot_pose, lm_readings)
        integrate_capacity = 20
        landmark_capacity = 50
        action = landmark_choosing.policy(new_lm_states, [integrate_capacity, landmark_capacity])
        if action == "Accept" and number_integrated_lm < integrate_capacity:
            smart_slam.update(lm_readings)

    log_info(batch)
    batch += 1


def odom_callback(odometry):
    """
    Handles the odometry messages. Once receiving an odometry, computes velocity for robot to move,
    and use it as control vector for slam prediction.
    :param odometry: The odometry message.
    :return: None
    """
    global ekf, smart_slam, traditional_slam

    robot_pose = namedtuple("Pose", "x, y, theta")
    robot_pose.x = odometry.pose.pose.position.x
    robot_pose.y = odometry.pose.pose.position.y
    robot_pose.theta = 2 * math.atan2(odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
    velo = mobilize_robot(robot_pose)
    velocity_pub = rospy.Publisher("/r1/cmd_vel", Twist)

    control_vector = numpy.matrix([velo.linear.x, velo.linear.y, velo.angular.z]).transpose()
    traditional_slam.predict(control_vector)
    smart_slam.predict(control_vector)

    velocity_pub.publish(velo)


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
    global goals, landmark_choosing

    rospy.init_node("smart_slam_exp")

    goal_file = "/home/oralas/Documents/Capstone/Goal Info"
    goals_info = open(goal_file)

    for line in goals_info:
        one_dest = line.split(",")
        x, y = float(one_dest[0]), float(one_dest[1])
        goals.append([x, y])

    landmark_choosing = LandmarkChoice(10)

    if len(goals) > 0:
        rospy.Subscriber("landmark_info", Target, landmark_callback)
        rospy.Subscriber("odom", Odometry, odom_callback)


if __name__ == "__main__":
    try:
        corobot_smart_slam()
    except rospy.ROSInterruptException:
        pass
