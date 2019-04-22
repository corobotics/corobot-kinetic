import rospy
import math
import numpy

from corobot_common.msg import Pose
from utils import column_vector, coord_transform, get_offset, reduce_covariance


# class Landmark:
#     __slots__ = {"id", "x", "y", "cov"}
#
#     def __init__(self, id, pose):
#         self.id = id
#         self.x = pose.x
#         self.y = pose.y
#         self.cov = numpy.matrix([[1, 0, 0],
#                                  [0, 1, 0],
#                                  [0, 0, 0]])


class EKFSLAM:
    """
    This is class for robotics Extended Kalman Filter Simultaneous Localization and Mapping algorithm, with limited
    modifiability. (Because I'm not that smart, at least not at present.)
    """
    __slots__ = {"mean_mat", "cov_mat", "gain", "motion_cov", "camera_cov", "landmarks"}

    def __init__(self, mean_mat = numpy.zeros([3, 1]), cov_mat = numpy.zeros([3, 3]),
                 motion_cov = numpy.identity(3), camera_cov = numpy.identity(2)):
        self.mean_mat = mean_mat
        self.cov_mat = cov_mat
        self.gain = None
        self.motion_cov = motion_cov
        self.camera_cov = camera_cov
        self.landmarks = dict()

    def add_new_landmark(self, landmark):
        """
        When a new landmark is integrated into system, the matrices need to be updated correspondingly.
        :param landmark: Contains the information of landmark with its name, location of (x, y) and location covariance.
        :return:
        """
        lm_loc = numpy.matrix([landmark.x, landmark.y]).transpose()
        self.mean_mat = numpy.concatenate((self.mean_mat, lm_loc), 0)

        top_right = numpy.zeros([self.cov_mat.shape[0], 2])
        left_bottom = top_right.transpose()
        up = numpy.concatenate((self.cov_mat, top_right), 1)
        down = numpy.concatenate((left_bottom, landmark.cov), 1)
        self.cov_mat = numpy.concatenate((up, down), 0)

        self.landmarks[landmark.name] = len(self.landmarks)


    def predict(self, control_vector):
        """
        Calculate new mean matrix and covariance matrix with given control matrix.
        :param control_vector: The change of robot location, in form of (x, y, theta)
        :return:
        """
        zero_mat = numpy.zeros([3, 2 * len(self.landmarks)])
        f1_mat = numpy.concatenate((numpy.identity(3), zero_mat), 1)

        self.mean_mat = self.mean_mat + f1_mat.transpose() * control_vector

        control_mat = numpy.concatenate((numpy.zeros([3, 2]), control_vector), 1)
        control_mat[2, 2] = 0
        g_mat = numpy.identity(f1_mat.shape[1]) + f1_mat * control_mat * f1_mat.transpose()

        self.motion_cov = f1_mat.transpose() * self.motion_cov * f1_mat
        self.cov_mat = g_mat * self.cov_mat * g_mat.transpose() + f1_mat.transpose() * self.motion_cov * f1_mat

    def upate(self, lm_observation):
        """
        Responsible of updating the predicted values with Kalman Gain.
        :param lm_observation: Supposedly, this should be a vector of size 2 by 1
        :return:
        """
        landmark_idx = self.landmarks[lm_observation.name]
        mean_lm = self.mean_mat[3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2]
        cov_r_lm = self.cov_mat[0: 3, 3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2]
        cov_lm_r = self.cov_mat[3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2, 0: 3]
        lm_cov = self.cov_mat[3 + landmark_idx * 2: 3 + landmark_idx * 2 + 2, 3 + landmark_idx * 2: 3 + landmark_idx * 2 + 2]

        lm_x = lm_observation.dist * math.cos(lm_observation.angle + self.mean_mat[2]) + self.mean_mat[0]
        lm_y = lm_observation.dist * math.sin(lm_observation.angle + self.mean_mat[2]) + self.mean_mat[1]
        observed_loc = numpy.matrix([lm_x, lm_y]).transpose()

        r_lm_delta = mean_lm - self.mean_mat[0 : 2]
        q = (r_lm_delta.transpose() * r_lm_delta).item()
        delta_x, delta_y = r_lm_delta[0].item(), r_lm_delta[1].item()
        expected_reading = numpy.matrix([math.sqrt(q), math.atan2(r_lm_delta[1], r_lm_delta[0]) - self.mean_mat[2]]).transpose()

        low_jacobian = 1 / q * numpy.matrix([[-math.sqrt(q) * delta_x, -math.sqrt(q) * delta_y, 0, math.sqrt(q) * delta_x, math.sqrt(q) * delta_y],
                                             [delta_y, -delta_x, -q, -delta_y, delta_x]])

        up = numpy.concatenate((numpy.identity(3), numpy.zeros([3, 2 * len(self.landmarks)])), 1)
        down = numpy.zeros([2, 3 + 2 * len(self.landmarks)])
        down[0, 3 + 2 * landmark_idx], down[1, 3 + 2 * landmark_idx + 1] = 1, 1
        f2_mat = numpy.concatenate((up, down), 0)
        observe_jacobian = low_jacobian * f2_mat

        s_mat = numpy.dot(numpy.dot(observe_jacobian, self.cov_mat), observe_jacobian.transpose()) + self.camera_cov
        self.gain = numpy.dot(numpy.dot(self.cov_mat, observe_jacobian.transpose()), numpy.linalg.inv(s_mat))

        self.mean_mat = self.mean_mat + numpy.dot(self.gain, (lm_observation - expected_reading))
        self.cov_mat = numpy.dot((numpy.identity(3 + 2 * len(self.landmarks)) - numpy.dot(self.gain, observe_jacobian)), self.cov_mat)


def points_relation(point_a, point_b):
    """
    Given two points, calculate the distance between them and point_b's orientation to point_a in a 2d plane.
    :param point_a: First point in 2d plane.
    :param point_b: Second point in 2d plane.
    :return: The distance and orientation.
    """
    dist, angle = 0, 0

    return dist, angle