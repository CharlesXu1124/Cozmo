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
    def __init__(self, particle, weight):
        self.particle = particle
        self.weight = weight

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
    dx, dy, dh = odom
    # update the dx and dy in the global frame
    # rotation remains unchanged in different frames -- no need to update
    for p in particles:
        rx, ry = rotate_point(dx, dy, p.h)
        p.x += add_gaussian_noise(rx, setting.ODOM_TRANS_SIGMA)
        p.y += add_gaussian_noise(ry, setting.ODOM_TRANS_SIGMA)
        p.h += add_gaussian_noise(dh, setting.ODOM_HEAD_SIGMA)
        motion_particles.append(p)
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
    for p in particles:
        # if particle is within grid
        if grid.is_free(p.x, p.y):
            # if measured marker for the particle is non-zero
            if len(measured_marker_list) > 0:
                # get the list of markers
                particle_markers = p.read_markers(grid)
                particle_weights.append(
                    reweight_particle(
                        measured_marker_list=measured_marker_list,
                        pm_list=particle_markers,
                        particle=p,
                    )
                )
            else:
                # if there is no marker in the observed list, assign the weight to 1
                particle_weights.append(ParticleType(particle=p, weight=1))
        else:
            # assign 0 weight if the particle is outside grid
            particle_weights.append(ParticleType(particle=p, weight=0))
    # get the total weight
    total_weight = 0
    for p in particle_weights:
        total_weight += p.weight


    # get the average weight
    w_avg = total_weight / len(particle_weights)

    # get the number of positive-weighted particles
    n_pos_particle = 0
    for p in particle_weights:
        if p.weight > 0:
            n_pos_particle += 1
    # rescale the positive weight so the total weight remains unchanged
    if n_pos_particle > 0:
        w_avg = len(particles) * (w_avg / n_pos_particle)
    if w_avg > 0:
        for pw in particle_weights:
            pw.weight = pw.weight / total_weight

    # get the number of 0 weighted particles
    n_zero_weight_particle = len(particle_weights) - n_pos_particle

    # delete those 0-weighted particles, meanwhile create a random list of particles and spread them on the grid
    measured_particles = Particle.create_random(n_zero_weight_particle, grid)
    if particle_weights != []:
        pdf = []
        for p in particle_weights:
            pdf.append(p.weight)
        # randomly choose count_positive amount of particles with sampling pdf proportional to their weight
        # a.k.a. resampling process
        sample = np.random.choice(particle_weights, size=n_pos_particle, p=pdf, replace=True)
        for s in sample:
            p = s.particle
            measured_particles.append(Particle(p.x, p.y, p.h))

    # assert len(measured_particles) == len(particles)
    return measured_particles

# helper function for getting the weight for each particle
def reweight_particle(measured_marker_list, pm_list, particle):
    if len(measured_marker_list) == 0 or len(pm_list) == 0:
        return ParticleType(particle, 0)
    # initialize the list for matched marker
    measure_counter = 0
    match_list = []
    # convert it list to set to randomize the process
    particle_set = set(pm_list)

    while measure_counter < len(measured_marker_list) and len(particle_set) > 0:
        # get the next measured marker
        marker_measurement = measured_marker_list[measure_counter]
        
        def distance(particle):
            return grid_distance(marker_measurement[0], marker_measurement[1], particle[0], particle[1])
        closest_particle = min(pm_list, key=lambda p: distance(p))
        

        match_list.append((marker_measurement, closest_particle))
        # remove the best particle which forms the marker-marker pair
        if closest_particle in particle_set:
            particle_set.remove(closest_particle)
        # increment the counter for the measured markers to find the next marker-marker pair
        measure_counter += 1
    # do the weight updates
    weight = 1
    for m, p in match_list:
        # extract angle and distance from the marker in the matched list
        rot = diff_heading_deg(m[2], p[2])
        dist = grid_distance(m[0], m[1], p[0], p[1])
        
        trans_error = 2 * setting.MARKER_TRANS_SIGMA ** 2
        rot_error = 2 * setting.MARKER_ROT_SIGMA ** 2
        
        indices = -((dist**2 / (trans_error)) + \
              (rot**2 / (rot_error)))


