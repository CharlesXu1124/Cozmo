# author: Victoria Neal, Zheyuan Xu
from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
import random
np.random.seed(setting.RANDOM_SEED)
random.seed(setting.RANDOM_SEED)

# class: defining a datatype for particle-weight pair
class ParticleType(object):
    '''
    class type: ParticleType
    attributes: particle, weight
    '''
    def __init__(self, particle, weight):
        self.particle = particle
        self.weight = weight

# helper function for getting the weight for each particle
def reweight(mm_list, pm_list, particle):
    if len(mm_list) == 0 or len(pm_list) == 0:
        return ParticleType(particle, 0)
    # initialize the list for matched marker
    counter = 0
    match_list = []
    # convert list to set to randomize the process
    p_set = set(pm_list)

    while counter < len(mm_list) and len(p_set) > 0:
        # get the next measured marker
        marker_measurement = mm_list[counter]
        # inner function for calculating the distance, as a criterion for selecting the closest particles to
        # the designated marker
        def distance(particle):
            return grid_distance(marker_measurement[0], marker_measurement[1], particle[0], particle[1])
        min_p = min(pm_list, key=lambda p: distance(p))
        
        match_list.append((marker_measurement, min_p))
        # remove the best particle which forms the marker-marker pair
        if min_p in p_set:
            p_set.remove(min_p)
        # increment the counter for the measured markers to find the next marker-marker pair
        counter += 1

    # do the weight updates
    prob = 1
    for m, p in match_list:
        # extract angle and distance from the marker in the matched list
        rot = diff_heading_deg(m[2], p[2])
        dist = grid_distance(m[0], m[1], p[0], p[1])
        # apply the gaussian errors on rotation and translation
        trans_error = 4 * setting.MARKER_TRANS_SIGMA ** 2
        rot_error = 8 * setting.MARKER_ROT_SIGMA ** 2
        
        indices = -((dist**2 / (trans_error)) + (rot**2 / (rot_error)))
        prob *= np.exp(indices)
    return ParticleType(particle, prob)



def motion_update(particles, odom):
    """ Particle filter motion update
        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*
        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    motion_particles = []

    for particle in particles:
        x = odom[0]
        y = odom[1]

        # particle rotation
        x1, y1 = rotate_point(x, y, particle.h)
        particle.x += x1
        particle.x = add_gaussian_noise(particle.x, setting.ODOM_TRANS_SIGMA)
        particle.y += y1
        particle.y = add_gaussian_noise(particle.y, setting.ODOM_TRANS_SIGMA)

        particle.h += odom[2]
        particle.h = add_gaussian_noise(particle.h, setting.ODOM_HEAD_SIGMA)

        motion_particles.append(particle)
    return motion_particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    particle_weights = []
    # if no markers seen within FOV, do nothing to the particles
    if len(measured_marker_list) == 0:
        return particles

    # if there are actually observed markers within FOV
    for p in particles:
        # if particle is within grid
        if grid.is_free(p.x, p.y):
            # if measured marker for the particle is non-zero
            # get the list of markers
            particle_markers = p.read_markers(grid)
            particle_reweight = reweight(measured_marker_list, particle_markers,p)
            particle_weights.append(particle_reweight)
        else:
            # assign 0 weight if the particle is outside grid
            particle_weights.append(ParticleType(particle=p, weight=0))
    # get the total weight
    total_weight = 0
    for p in particle_weights:
        total_weight += p.weight


    # get the average weight
    w_avg = total_weight / len(particle_weights)

    # get the number of positive-weighted particles and zero-weighted particles
    n_pos_particle = 0
    n_zero_weight_particle = 0
    for p in particle_weights:
        if p.weight > 0:
            n_pos_particle += 1
        else:
            n_zero_weight_particle += 1
    # rescale the positive weight so the total weight remains unchanged
    if n_pos_particle > 0:
        w_avg = len(particles) * (w_avg / n_pos_particle)
    if w_avg > 0:
        for pw in particle_weights:
            pw.weight = pw.weight / total_weight

    # delete those 0-weighted particles, meanwhile create a random list of particles and spread them on the grid
    measured_particles = Particle.create_random(n_zero_weight_particle, grid)
    if len(particle_weights) != 0:
        # construct the probability density function for resampling
        weight_list = []
        for p in particle_weights:
            weight_list.append(p.weight)
        # randomly choose count_positive amount of particles with sampling pdf proportional to their weight
        # a.k.a. resampling process
        resampling = np.random.choice(particle_weights, size=n_pos_particle, p=weight_list, replace=True)
        for s in resampling:
            p = s.particle
            measured_particles.append(Particle(p.x, p.y, p.h))

    return measured_particles


