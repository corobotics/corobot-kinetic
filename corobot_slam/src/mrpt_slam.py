#!/usr/bin/env python

# Imports are here.
import roslib
roslib.load.manifest("corobot_slam")
import numpy
import math
import random
import rospy
import copy

# From ... import ... are here.
from corobot_common.msg import Target
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ekf_slam import EKFSLAM
# from montecarlo import MonteCarlo
from collections import namedtuple

# Global variables are here.
smart_slam = None
traditional_slam = None
goals = list()
tolerance = 0.01
m_batch = 1
l_batch = 1
landmark_choosing = None
velocity_pub = None
landmark_truth = dict()
rotation = 0


# class LandmarkChoice(MonteCarlo):
class LandmarkChoice:
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
        self.kis = None
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
        self.kis = [random.uniform(0, 0.5), random.uniform(0.5, 1)]
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
        Given a state, compute the average reward of its K nearest neighbor in a state-reward dictionary.
        :param new_state: The new state w.r.t. which an action needs to be decided.
        :param sr_dict: Dictionary containing information of past states and their corresponding rewards.
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
    lm_x = observation.dist * math.cos(observation.angle + robot_pose[2].item()) + robot_pose[0].item()
    lm_y = observation.dist * math.sin(observation.angle + robot_pose[2].item()) + robot_pose[1].item()
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
    if len(slam.get_integrated_lm()) > 0:
        first_lm_loc = slam.get_landmark_loc(min(slam.get_integrated_lm()))[0]
        shortest_dist = get_distance(potential_lm_loc, first_lm_loc)
        for one_lm in slam.get_integrated_lm().keys():
            landmark_loc = slam.get_landmark_loc(one_lm)[0]
            if get_distance(potential_lm_loc, landmark_loc) < shortest_dist:
                shortest_dist = get_distance(potential_lm_loc, landmark_loc)
    else:
        shortest_dist = 0
    state_list.append(shortest_dist)
    # Fifth dimension of states. Uncertainty of robot pose in terms of entropy
    dim_five = math.log(math.sqrt((2 * math.pi * math.e) ** 3 * numpy.linalg.det(robot_cov)))
    state_list.append(dim_five)

    state = tuple(state_list)
    return state


def get_distance(point_1, point_2):
    """
    Calculate the Euclidean distance between point_1 and point_2
    :param point_1: Coordinates of point_1 in form of a COLUMN VECTOR.
    :param point_2: Coordinates of point_2 in form of a COLUMN VECTOR.
    :return: The comptuted Euclidean distance.
    """
    delta = point_1 - point_2
    distance = math.sqrt(numpy.dot(delta.transpose(), delta).item())

    return distance


def mobilize_robot(robot_pose=None):
    """
    Send velocity information to robot, so robot can move.
    :param robot_pose: Current robot pose, in form of a named tuple, containing (x, y, theta). Default as None.
    :return: If robot_pose is None, then robot stops. Else, robot will move along x with 0.1/s and 0.1 * pi/s
    """
    global tolerance, goals, rotation

    twist = Twist()
    twist.linear.x = numpy.float64(0.0)
    twist.linear.y = numpy.float64(0.0)
    twist.linear.z = numpy.float64(0.0)
    twist.angular.x = numpy.float64(0.0)
    twist.angular.y = numpy.float64(0.0)
    twist.angular.z = numpy.float64(0.0)

    if robot_pose is not None and len(goals) > 0:
        next_goal = goals[0]
        distance = get_distance(numpy.matrix([robot_pose.x, robot_pose.y]).transpose(),
                                numpy.matrix([next_goal[0], next_goal[1]]).transpose())
        if distance <= tolerance:
            goals.pop(0)
        else:
            # Calculate rotation based on destination
            target_angle = math.atan2((next_goal[1] - robot_pose.y), (next_goal[0] - robot_pose.x))
            rotation = target_angle - robot_pose.theta
            twist.linear.x = numpy.float64(0.075)
            if rotation != 0:
                twist.angular.z = numpy.float64(math.copysign(0.25, rotation))

    # if robot_pose is not None:
    #     next_goal = goals[0]
    #     distance = get_distance(numpy.matrix([robot_pose.x, robot_pose.y]).transpose(),
    #                             numpy.matrix([next_goal[0], next_goal[1]]).transpose())
    #     if distance <= tolerance:
    #         while rotation < math.pi * 20:
    #             twist.angular.z = numpy.float64(0.25)
    #             rotation += (twist.angular.z / 10)
    #         goals.pop(0)
    #     else:
    #         # Calculate rotation based on destination
    #         target_angle = math.atan2((next_goal[1] - robot_pose.y), (next_goal[0] - robot_pose.x))
    #         rotation = target_angle - robot_pose.theta
    #         twist.linear.x = numpy.float64(0.075)
    #         if rotation != 0:
    #             twist.angular.z = numpy.float64(math.copysign(0.25, rotation))

    return twist


def flat_tuple(mat):
    flat = str()
    for i in range(mat.shape[0]):
        flat += str(mat[i]).strip("[]") + ";"

    return flat[0:-1]


def log_info(batch_number, mapping):
    if mapping is True:
        t_log_name = "traditional_slam_log.txt"
        s_log_name = "smart_slam_log.txt"

        tradition_log = open(t_log_name, "a")
        smart_log = open(s_log_name, "a")

        batch_start = "==========================Batch %d Start==========================\n" % batch_number
        batch_end = "===========================Batch %d End===========================\n" % batch_number
        tradition_log.write(batch_start)
        smart_log.write(batch_start)

        [t_r_pose, t_r_cov] = traditional_slam.get_robot_pose()
        tradition_log.write("Robot pose:\t" + flat_tuple(t_r_pose) + "\n")
        tradition_log.write("Robot cov:\t" + flat_tuple(t_r_cov) + "\n")
        [s_r_pose, s_r_cov] = smart_slam.get_robot_pose()
        smart_log.write("Robot pose:\t" + flat_tuple(s_r_pose) + "\n")
        smart_log.write("Robot cov:\t" + flat_tuple(s_r_cov) + "\n")

        for each_lm in traditional_slam.get_integrated_lm().keys():
            [t_lm_loc, t_lm_cov] = traditional_slam.get_landmark_loc(each_lm)
            tradition_log.write("Landmark " + str(each_lm) + " location: \t" + flat_tuple(t_lm_loc) + "\n")
            tradition_log.write("Landmark " + str(each_lm) + " cov: \t" + flat_tuple(t_lm_cov) + "\n")

        for each_lm in smart_slam.get_integrated_lm().keys():
            [s_lm_loc, s_lm_cov] = smart_slam.get_landmark_loc(each_lm)
            smart_log.write("Landmark " + str(each_lm) + " location: \t" + flat_tuple(s_lm_loc) + "\n")
            smart_log.write("Landmark " + str(each_lm) + " cov: \t" + flat_tuple(s_lm_cov) + "\n")

        tradition_log.write(batch_end)
        smart_log.write(batch_end)
        tradition_log.close()
        smart_log.close()
    else:
        if batch_number % 10 == 1:
            pt_log_name = "pt_log.txt"

            pt_log = open(pt_log_name, "a")
            current_dest = numpy.matrix(goals[0])
            t_r_pose = traditional_slam.get_robot_pose()[0]
            s_r_pose = smart_slam.get_robot_pose()[0]
            pt_log.write("Current Destination:\t" + flat_tuple(current_dest) + "\n")
            pt_log.write("Tradi SLAM pose:\t" + flat_tuple(t_r_pose) + "\n"
                         + "Smart SLAM pose:\t" + flat_tuple(s_r_pose) + "\n")
            pt_log.write("---------------------------------------------------")
            pt_log.close()


def calc_reward(slam):
    """
    Calculate the reward after integrating a landmark.
    :param slam: The object of SLAM.
    :return: The reward of this.
    """
    avg_dist = 0
    count = len(slam.get_integrated_lm())

    if count == 0:
        avg_dist = 0
    else:
        for integrated_lm in slam.get_integrated_lm().keys():
            est_loc = slam.get_landmark_loc(integrated_lm)[0]
            tru_loc = landmark_truth[integrated_lm]
            avg_dist -= get_distance(est_loc, tru_loc)
        avg_dist /= count

    return avg_dist


def landmark_callback(qrcode_info):
    """
    Handles the camera readings of QR codes. The camera readings will be used for updating the SLAM model.
    Also it will log the SLAM models after their update.
    :param qrcode_info: Contains the .name, distance, angle, and camera_id. camera_id is used to identify whether the
    camera is mounted on left side or right side of robot.
    :return:
    """
    global landmark_choosing, traditional_slam, smart_slam, m_batch

    # print(qrcode_info.name)
    lm_readings = namedtuple("LMReading", "name, dist, angle")
    lm_readings.name = qrcode_info.name
    lm_readings.dist = qrcode_info.dist
    if qrcode_info.camera_id == 0:
        lm_readings.angle = qrcode_info.angle + math.pi / 2
    else:
        lm_readings.angle = qrcode_info.angle - math.pi / 2

    if lm_readings.dist < 0.7:
        camera_cov = numpy.matrix([[0.0001, 0], [0, 0.00001]])
    else:
        camera_cov = numpy.matrix([[0.005 * lm_readings.dist, 0], [0, 0.001 * lm_readings.dist]])
    # Traditional SLAM will update with the landmark info no matter what (It's expensive but not picky).
    traditional_slam.update(lm_readings, camera_cov)

    # Smart SLAM will update its model if the landmark is an integrated one, or consider whether to integrate it
    # if it's a new one.
    integrate_capacity, landmark_capacity = 5, 12
    if lm_readings.name in smart_slam.get_integrated_lm():
        # If the landmark is already integrated, update the model.
        smart_slam.update(lm_readings, camera_cov)
    elif len(smart_slam.get_integrated_lm()) < integrate_capacity:
        # Evaluate to decide whether to integrate this new landmark or not.
        smart_slam_copy = copy.deepcopy(smart_slam.get_robot_pose())
        new_lm_state = calc_states(smart_slam_copy, lm_readings)
        action = landmark_choosing.policy(new_lm_state, [integrate_capacity, landmark_capacity])
        if action == "Accept":
            smart_slam.update(lm_readings, camera_cov)

        state_action = tuple([new_lm_state, action])
        reward = calc_reward(smart_slam_copy)
        landmark_choosing.update_reward(state_action, reward)

    log_info(m_batch, True)
    m_batch += 1


def odom_callback(odometry):
    """
    Handles the odometry messages. Once receiving an odometry, computes velocity for robot to move,
    and use it as control vector for slam prediction.
    :param odometry: The odometry message.
    :return: None
    """
    global smart_slam, traditional_slam, velocity_pub, l_batch

    robot_pose = namedtuple("Pose", "x, y, theta")
    robot_pose.x = odometry.pose.pose.position.x
    robot_pose.y = odometry.pose.pose.position.y
    robot_pose.theta = 2 * math.atan2(odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w)
    # print("Robot at:", robot_pose.x, robot_pose.y, robot_pose.theta)
    velo = mobilize_robot(robot_pose)

    motion_vector = numpy.matrix([[velo.linear.x * math.cos(velo.angular.z) / 10],
                                  [velo.linear.x * math.sin(velo.angular.z) / 10],
                                  [velo.angular.z / 10]])
    motion_cov = numpy.matrix([[0.05 * velo.linear.x / 10, 0, 0],
                               [0, 0.1 * velo.linear.x / 10, 0],
                               [0, 0, 0.2 * velo.angular.z / 10]])
    traditional_slam.predict(motion_vector, motion_cov)
    smart_slam.predict(motion_vector, motion_cov)

    velocity_pub.publish(velo)
    log_info(l_batch, False)
    l_batch += 1


def corobot_smart_slam():
    """
    The running of using SLAM algorithm with Reinforcement Learning. It's currently set to pass as our experiment
    requires to run not only smart SLAM but traditional SLAM as well.
    :return:
    """
    pass


def run_experiment():
    """
    Main function for running experiment. Robot will run smart SLAM and traditional SLAM at the same time.
    :return:
    """
    global goals, landmark_choosing, velocity_pub, traditional_slam, smart_slam, landmark_truth

    rospy.init_node("smart_slam_exp")

    gt = open("lm_loc.txt")
    for line in gt:
        str_list = line.split(",")
        lm_name = str_list[0]
        lm_loc = numpy.matrix([float(str_list[1]), float(str_list[1])]).transpose()
        landmark_truth[lm_name] = lm_loc

    traditional_slam = EKFSLAM()
    smart_slam = EKFSLAM()
    velocity_pub = rospy.Publisher("mobile_base/commands/velocity", Twist)

    goal_file = "goals.txt"
    goals_info = open(goal_file)

    for line in goals_info:
        one_dest = line.split(",")
        x, y = float(one_dest[0]), float(one_dest[1])
        goals.append([x, y])
    # goals = [1, 0]
    landmark_choosing = LandmarkChoice(10)

    if len(goals) > 0:
        rospy.Subscriber("landmark_info", Target, landmark_callback)
        rospy.Subscriber("odom", Odometry, odom_callback)
    else:
        # If robot reaches the final destination, it'll stop.
        stop = mobilize_robot()
        velocity_pub.publish(stop)

    rospy.spin()


if __name__ == "__main__":
    try:
        run_experiment()
    except rospy.ROSInterruptException:
        pass
