#!/usr/bin/env python

import rospy
import math
import numpy


class EKFSLAM:
    """
    This is class for robotics Extended Kalman Filter Simultaneous Localization and Mapping algorithm, with limited
    modifiability. (Because I'm not that smart, at least not at present.)
    """
    __slots__ = {"mean_mat", "cov_mat", "gain", "landmarks"}

    def __init__(self, mean_mat=numpy.zeros([3, 1], numpy.int16), cov_mat=numpy.identity(3, numpy.int16)):
        self.mean_mat = mean_mat
        self.cov_mat = cov_mat
        self.gain = None
        self.landmarks = dict()

    def predict(self, control_vector, motion_cov):
        """
        Calculate new mean matrix and covariance matrix with given control matrix.
        :param control_vector: The change of robot location, in form of column vector [x, y, theta]
        :param motion_cov: Covariance matrix of motion.
        :return:
        """
        zero_mat = numpy.zeros([3, 2 * len(self.landmarks)], numpy.int16)
        # f1_mat is a helper matrix of size 3 * (3 + 2N)
        f1_mat = numpy.concatenate((numpy.identity(3, numpy.int16), zero_mat), 1)

        self.mean_mat = self.mean_mat + numpy.dot(f1_mat.transpose(), control_vector)

        control_mat = numpy.concatenate((numpy.zeros([3, 2], numpy.int16), control_vector), 1)
        control_mat[2, 2] = 0
        jacobian_f = numpy.dot(numpy.dot(f1_mat.transpose(), control_mat), f1_mat)
        i_size = jacobian_f.shape[0]
        jacobian_f = numpy.identity(i_size, numpy.int16) + jacobian_f

        resized_motion_cov = numpy.dot(numpy.dot(f1_mat.transpose(), motion_cov), f1_mat)
        self.cov_mat = numpy.dot(numpy.dot(jacobian_f, self.cov_mat), jacobian_f.transpose()) + resized_motion_cov

    def update(self, observation, camera_cov):
        """
        Responsible of updating the predicted values with Kalman Gain. This is divided into two parts.
        If observed landmark is a new one, then integrate it; Else, update the model.
        :param observation: Contains information in form of .name, .dist, .angle, and .cov.
        :param camera_cov: Covariance of observation.
        :return:
        """
        # Observation of a unintegrated landmark and we will integrate it now.
        if observation.name not in self.landmarks:
            lm_x = observation.dist * math.cos(observation.angle + self.mean_mat[2].item()) + self.mean_mat[0].item()
            lm_y = observation.dist * math.sin(observation.angle + self.mean_mat[2].item()) + self.mean_mat[1].item()
            lm_loc = numpy.matrix([lm_x, lm_y]).transpose()
            self.mean_mat = numpy.concatenate((self.mean_mat, lm_loc), 0)

            top_right = numpy.zeros([self.cov_mat.shape[0], 2], numpy.int16)
            left_bottom = top_right.transpose()
            top = numpy.concatenate((self.cov_mat, top_right), 1)
            bottom = numpy.concatenate((left_bottom, camera_cov), 1)
            self.cov_mat = numpy.concatenate((top, bottom), 0)

            self.landmarks[observation.name] = len(self.landmarks)
        # Update model when seeing an integrated landmark.
        else:
            landmark_idx = self.landmarks[observation.name]
            estimated_lm_loc = self.mean_mat[3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2]

            estimated_lm_r_delta = estimated_lm_loc - self.mean_mat[0 : 2]  # Predicted [dx, dy] between robot and lm.
            # q_val is a helper variable.
            q_val = numpy.dot(estimated_lm_r_delta.transpose(), estimated_lm_r_delta).item()
            q_sqr = math.sqrt(q_val)
            delta_x, delta_y = estimated_lm_r_delta[0].item(), estimated_lm_r_delta[1].item()
            expected_reading = numpy.matrix([q_sqr, math.atan2(delta_y, delta_x) - self.mean_mat[2].item()]).transpose()

            low_jacobian = (1 / q_val) \
                           * numpy.matrix([[-q_sqr * delta_x, -q_sqr * delta_y, 0, q_sqr * delta_x, q_sqr * delta_y],
                                           [delta_y, -delta_x, -q_val, -delta_y, delta_x]])

            top = numpy.concatenate((numpy.identity(3), numpy.zeros([3, 2 * len(self.landmarks)], numpy.int16)), 1)
            bottom = numpy.zeros([2, 3 + 2 * len(self.landmarks)], numpy.int16)
            bottom[0, 3 + 2 * landmark_idx], bottom[1, 3 + 2 * landmark_idx + 1] = 1, 1
            # f2_mat is a helper matrix of size 5 * (3 + 2N)
            f2_mat = numpy.concatenate((top, bottom), 0)
            observe_jacobian = numpy.dot(low_jacobian, f2_mat)  # observe_jacobian is of size 2 * (3 + 2N)

            # s_mat is a helper matrix.
            s_mat = numpy.dot(numpy.dot(observe_jacobian, self.cov_mat), observe_jacobian.transpose()) + camera_cov
            self.gain = numpy.dot(numpy.dot(self.cov_mat, observe_jacobian.transpose()), numpy.linalg.inv(s_mat))

            observation_reading = numpy.matrix([observation.dist, observation.angle]).transpose()
            self.mean_mat = self.mean_mat + numpy.dot(self.gain, (observation_reading - expected_reading))
            self.cov_mat = numpy.dot((numpy.identity(3 + 2 * len(self.landmarks), numpy.int16)
                                      - numpy.dot(self.gain, observe_jacobian)), self.cov_mat)

    def get_robot_pose(self):
        robot_pose = self.mean_mat[0: 3]
        robot_cov = self.cov_mat[0: 3, 0: 3]

        return [robot_pose, robot_cov]

    def get_landmark_loc(self, lm_name):
        landmark_idx = self.landmarks[lm_name]

        lm_loc = self.mean_mat[3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2]
        lm_cov = self.cov_mat[3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2,
                 3 + 2 * landmark_idx: 3 + 2 * landmark_idx + 2]

        return [lm_loc, lm_cov]

    def get_integrated_lm(self):
        return self.landmarks
