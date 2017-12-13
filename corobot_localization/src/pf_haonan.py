#!/usr/bin/env python
import roslib
import math
import random

import rospy
import particle
import copy

from corobot_common.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ekf import EKF
from corobot_common.srv import GetCoMap
from utils import odom_to_pose


def pf_initialize(qrcode_pose):
    """
    Initialize the particles based on given QR code sensor readings.
    :param qrcode_pose: QR code sensor reading information.
    :return: None
    """
    print("Entered initialization.")
    # Get pose information
    x_real = qrcode_pose.x
    y_real = qrcode_pose.y
    orientation = qrcode_pose.theta
    mean = 0
    covariance = tuple(qrcode_pose.cov)
    init_probability = 1 / num_particles

    particle_count = 0
    # Initialize 500 objects of particle
    while particle_count < num_particles:
        particles.append(particle.Particle(x_real, y_real, orientation, mean,
                                           covariance[0], covariance[4], covariance[8],
                                           map, init_probability))
        particle_count += 1
    print("Particles size:", len(particles))

def prediction(odom):
    """
    Predict the particle's new location based on Odometry information and particle's location.
    :param odom: Odometry information retrieved by odometer sensors.
    :return: None
    """
    print("Entered prediction.")
    odom_delta = odom_to_pose(odom)
    delta = ekf.get_odom_delta(odom_delta)
    if delta is not None:
        tuple_odom = delta.T.tolist()[0]
    else:
        tuple_odom = 0, 0, 0
    delta_x = tuple_odom[0]
    delta_y = tuple_odom[1]
    delta_theta = tuple_odom[2]
    trv_dist = math.sqrt((delta_x ** 2) + (delta_y ** 2))

    particle_count = 0
    mean = 0
    trv_sigma = 0.05
    orient_sigma = (0.5 / 360) * math.pi * 2
    # Add noises to odometry info and use the result to update every particle.
    for each_particle in particles:
        each_particle.predict_update(trv_dist, delta_theta, mean, trv_sigma, orient_sigma)
        # After updating, if particle's new location is "in the wall", it's definitely a "bad" particle.
        if each_particle.loc_check(map) is False:
            each_particle.probability = 0
        particle_count += 1


def update_model(scan):
    """
    Update particles with given sensor readings.
    :param scan: Laser scan sensor reading information.
    :return: None
    """
    print("Entered update_model.")
    particle_count = 0
    while particle_count < len(particles):
        # If the particle is already a bad one, kick it out from particle list.
        if particles[particle_count].probability == 0:
            particles.pop(particle_count)
        # If it's not a bad one, decide after updating its probability.
        else:
            particles[particle_count].probability_update(scan, map)
            if particles[particle_count].probability <= 0.2:
                particles.pop(particle_count)
            else:
                particle_count += 1

    # Make copies of the most probable particles to replace the popped particles.
    sorted_particles = sorted(particles, key= lambda particle: particle.probability, reverse= True)
    if len(particles) > 0:
        while len(particles) < num_particles:
            gap = num_particles - len(particles)
            if gap < len(sorted_particles):
                copied_particle = copy.deepcopy(sorted_particles[random.randint(0, gap)])
            elif gap == len(sorted_particles):
                copied_particle = copy.deepcopy(sorted_particles[random.randint(0, gap - 1)])
            else:
                copied_particle = copy.deepcopy(sorted_particles[random.randint(0, len(sorted_particles) - 1)])
            particles.append(copied_particle)
    else:
        print("All particles lost.")

    # For evaluating the PF algo.
    mean_x = 0
    mean_y = 0
    mean_theta = 0
    sigma_x = 0
    sigma_y = 0
    sigma_theta = 0
    for each_particle in particles:
        mean_x += each_particle.x_pos
        mean_y += each_particle.y_pos
        mean_theta += each_particle.orientation
    mean_x /= len(particles)
    mean_y /= len(particles)
    mean_theta /= len(particles)

    for  each_particle in particles:
        sigma_x += (each_particle.x_pos - mean_x) ** 2
        sigma_y += (each_particle.y_pos - mean_y) ** 2
        sigma_theta += (each_particle.orientation - mean_theta) ** 2
    sigma_x = math.sqrt(sigma_x / len(particles))
    sigma_y = math.sqrt(sigma_y / len(particles))
    sigma_theta = math.sqrt(sigma_theta / len(particles))
    mean_x *= map.info.resolution
    mean_y *= map.info.resolution

    print("Robot at:", mean_x, ",", mean_y, ", pointing at:",
          mean_theta, "Std deviations:", sigma_x, sigma_y, sigma_theta)


def main():
    global pose_pub, ekf, particles, map, num_particles
    rospy.init_node("localization")
    rospy.wait_for_service('get_map')
    map_srv = rospy.ServiceProxy('get_map', GetCoMap)
    map = map_srv().map
    roslib.load_manifest("corobot_localization")
    ekf = EKF()
    pose_pub = rospy.Publisher("pose", Pose)
    num_particles = 500
    particles = []
    rospy.Subscriber("qrcode_pose", Pose, pf_initialize)
    rospy.Subscriber("odom", Odometry, prediction)
    rospy.Subscriber("scan", LaserScan, update_model)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
