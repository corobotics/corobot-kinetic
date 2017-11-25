import random
import math
from bresenhem import bresenhem
from kinect_loc import bres_condition


class Particle:
    slots = ("x_pos", "y_pos", "orientation", "probability", "map")

    def __init__(self, x_exact, y_exact, orient_exact, mu, x_sigma, y_sigma, orientation_sigma, map, init_probability):
        """
        Initialize a particle instance with pose information.
        :param x_exact: X coordinate in unit of meter.
        :param y_exact: Y coordinate in unit of meter.
        :param orient_exact: Orientation of robot.
        :param mu: Mean value used in Gaussian distribution.
        :param x_sigma: Variance on X-direction.
        :param y_sigma: Variance on Y-direction.
        :param orientation_sigma: Variance on orientation.
        :param map: Map information.
        """
        # Add noises to (x, y) coordinates.
        x_pos = x_exact + round(random.gauss(mu, x_sigma), 4) * random.randint(-1, 1)
        y_pos = y_exact + round(random.gauss(mu, y_sigma), 4) * random.randint(-1, 1)
        self.map = map
        # Transfer the (x, y) coordinates with units of meters to units of pixels.
        self.x_pos = math.floor(x_pos / self.map.info.resolution)
        self.y_pos = math.floor(y_pos / self.map.info.resolution)
        self.orientation = orient_exact + round(random.gauss(mu, orientation_sigma), 4)
        # All particles shares same probability.
        self.probability = init_probability

    def predict_update(self, trv_dist, new_orient, mu, dist_sigma, orient_sigma):
        """
        Predict the particle's new location with given parameters.
        :param trv_dist: Traveled distance in meters.
        :param new_orient: New orientation of robot.
        :param mu: Mean value used for Gaussian distribution.
        :param dist_sigma: Variance of distance used for Gaussian distribution.
        :param orient_sigma: Variance of orientation used for Gaussian distribution.
        :return: None
        """
        # Add Gausssian distributed noises to traveled distance and orientation.
        est_trv_dist = trv_dist + round(random.gauss(mu, dist_sigma), 4) * random.randint(-1, 1)
        est_orient = new_orient + round(random.gauss(mu, orient_sigma), 4) * random.randint(-1, 1)
        # Transfer the traveled distance from unit of meters to unit of pixels.
        pixel_trv_dist = est_trv_dist / self.map.info.resolution
        # Calculate the new location of particle.
        self.orientation = round(math.fmod(self.orientation + est_orient), (2 * math.pi))
        self.x_pos = math.floor(math.cos(self.orientation) * pixel_trv_dist + self.x_pos)
        self.y_pos = math.floor(math.sin(self.orientation) * pixel_trv_dist + self.y_pos)

    def loc_check(self):
        """
        Quick check if the particle is in the wall or obstacle with given map.
        :return: True if particle is in open area; False if particle is in the wall or a mapped obstacle.
        """
        i = self.x_pos + (self.map.info.height - self.y_pos - 1) * self.map.info.width
        occ = self.map.data[i]
        if occ > 50:
            return False
        else:
            return True

    def probability_update(self, laser_scan):
        """
        Update the quality of particle based on its expected readings.
        :param laser_scan: The real laser sensor reading info.
        :return: None
        """
        starting = laser_scan.angle_min
        ending = laser_scan.angle_max
        increment = laser_scan.angle_increment * 10

        # should convert robot pose into kinect pose (offset backward ~9 cm) first
        starting_scan = self.orientation + starting
        ending_scan = self.orientation + ending

        self.probability = self.particle_quality(starting_scan, ending_scan, laser_scan.range_max,
                                                 laser_scan.range_min, increment, laser_scan.ranges)

    def particle_quality(self, starting_scan, ending_scan, range_max, range_min, scan_increment, readings):
        """
        Use Bresenham's algorithm to calculate a particle's expected laser sensor readings.
        :param starting_scan: Orientation of first scanning.
        :param ending_scan: Orientation of last scanning.
        :param range_max: Sensor's maximum scanning range.
        :param range_min: Sensor's minimum scanning range.
        :param scan_increment: Angle between consecutive scanning.
        :param readings: Readings of real laser scanning.
        :return: Probability of a particle, expressing the quality.
        """
        current_scan = starting_scan
        # dist_expec is used to describe the expectation of the ratio of real reading to estimated reading.
        dist_expec = 0
        reading_counter = 0
        particle_quality = 0

        # note theta is in original (right-handed) coords
        # to follow the scan order, we will increment in this coord system
        # but then negate the theta (or 2pi-theta) to do the image testing
        while current_scan <= ending_scan:
            scan_range = range_max / self.map.info.resolution
            target_x = self.x_pos + scan_range * math.cos(current_scan)
            target_y = self.y_pos + scan_range * math.sin(current_scan)

            if self.x_pos < 0:
                x_coord = 0
            elif self.x_pos > self.map.info.width:
                x_coord = self.map.info.width
            else:
                x_coord = int(self.x_pos)

            if self.y_pos < 0:
                y_coord = 0
            elif self.y_pos > self.map.info.height:
                y_coord = self.map.info.height
            else:
                y_coord = int(self.y_pos)

            if target_x < 0:
                target_x = 0
            elif target_x > self.map.info.width:
                target_x = self.map.info.width
            else:
                target_x = int(target_x)

            if target_y < 0:
                target_y = 0
            elif target_y > self.map.info.height:
                target_y = self.map.info.height
            else:
                target_y = int(target_y)

            # Use Bresenhem's algorithm to calculate the scanning reading.
            distance = bresenhem(x_coord, y_coord, target_x, target_y, bres_condition)
            pix_occupied_idx = 2

            # If there's a block between target pixel and particle. Estimated reading is the distance
            # between the particle and the block.
            if distance[pix_occupied_idx] is True:
                est_scan = math.sqrt((distance[0] - x_coord) ** 2 + (distance[1] - y_coord) ** 2) \
                                * self.map.info.resolution
            # Else, the sensor reading is the max of scanning range.
            else:
                est_scan = range_max

            current_real_reading = readings[current_scan]
            # If the real reading is out of effective range, set real/estimated ratio to 1.
            if current_real_reading < range_min or current_real_reading > range_max:
                dist_expec += 1
            # Else, calculate the ratio.
            else:
                dist_expec += current_real_reading / est_scan
            reading_counter += 1

            current_scan += scan_increment

        # After computing all estimated readings. Divide the summation of expected ratio by the number
        # of used readings.
        dist_expec /= reading_counter

        if dist_expec <= 0.8:
            particle_quality = 0.4
        elif dist_expec <= 0.95:
            particle_quality = (4/3) * dist_expec - (2 / 3)
        elif dist_expec <= 1.05:
            particle_quality = 0.6
        elif dist_expec <= 1.2:
            particle_quality = (-8 / 3) * dist_expec + 3.4
        else:
            particle_quality = 0.2

        return particle_quality