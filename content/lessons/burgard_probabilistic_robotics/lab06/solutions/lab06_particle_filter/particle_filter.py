import numpy as np
import scipy.stats
import matplotlib.pyplot as plt


def plot_state(particles, landmarks, map_limits):
    # Visualizes the state of the particle filter.
    #
    # Displays the particle cloud, mean position and landmarks.

    xs = []
    ys = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

    # landmark positions
    lx = []
    ly = []

    for i in range(len(landmarks)):
        lx.append(landmarks[i+1][0])
        ly.append(landmarks[i+1][1])

    # mean pose as current estimate
    estimated_pose = mean_pose(particles)

    # plot filter state
    plt.clf()
    plt.plot(xs, ys, 'r.')
    plt.plot(lx, ly, 'bo', markersize=10)
    plt.quiver(
        estimated_pose[0], estimated_pose[1], np.cos(
            estimated_pose[2]), np.sin(estimated_pose[2]),
        angles='xy', scale_units='xy')
    plt.axis(map_limits)

    plt.pause(0.01)


def initialize_particles(num_particles, map_limits):
    # randomly initialize the particles inside the map limits

    particles = []

    for i in range(num_particles):
        particle = dict()

        # draw x,y and theta coordinate from uniform distribution
        # inside map limits
        particle['x'] = np.random.uniform(map_limits[0], map_limits[1])
        particle['y'] = np.random.uniform(map_limits[2], map_limits[3])
        particle['theta'] = np.random.uniform(-np.pi, np.pi)

        particles.append(particle)

    return particles


def mean_pose(particles):
    # calculate the mean pose of a particle set.
    #
    # for x and y, the mean position is the mean of the particle coordinates
    #
    # for theta, we cannot simply average the angles because of the wraparound
    # (jump from -pi to pi). Therefore, we generate unit vectors from the
    # angles and calculate the angle of their average

    # save x and y coordinates of particles
    xs = []
    ys = []

    # save unit vectors corresponding to particle orientations
    vxs_theta = []
    vys_theta = []

    for particle in particles:
        xs.append(particle['x'])
        ys.append(particle['y'])

        # make unit vector from particle orientation
        vxs_theta.append(np.cos(particle['theta']))
        vys_theta.append(np.sin(particle['theta']))

    # calculate average coordinates
    mean_x = np.mean(xs)
    mean_y = np.mean(ys)
    mean_theta = np.arctan2(np.mean(vys_theta), np.mean(vxs_theta))

    return [mean_x, mean_y, mean_theta]


def sample_motion_model(odometry, particles):
    # Samples new particle positions, based on old positions, the odometry
    # measurements and the motion noise
    # (probabilistic motion models slide 27)

    delta_rot1 = odometry['r1']
    delta_trans = odometry['t']
    delta_rot2 = odometry['r2']

    # the motion noise parameters: [alpha1, alpha2, alpha3, alpha4]
    noise = [0.1, 0.1, 0.05, 0.05]

    # generate new particle set after motion update
    new_particles = []

    '''your code here'''
    '''***        ***'''
    for pt in particles:
        rot1_sigma = (noise[0] * np.abs(delta_rot1)) + \
            (noise[1] * delta_trans)
        rot2_sigma = (noise[0] * np.abs(delta_rot2)) + \
            (noise[1] * delta_trans)
        trans_sigma = (noise[2] * delta_trans) + \
            (noise[3] * (np.abs(delta_rot1) + np.abs(delta_rot2)))
        dhat_rot1 = delta_rot1 + np.random.normal(0, rot1_sigma)
        dhat_rot2 = delta_rot2 + np.random.normal(0, rot2_sigma)
        dhat_trans = delta_trans + + np.random.normal(0, trans_sigma)

        x = pt['x'] + (dhat_trans * np.cos(pt['theta'] + dhat_rot1))
        y = pt['y'] + (dhat_trans * np.sin(pt['theta'] + dhat_rot1))
        theta = pt['theta'] + dhat_rot1 + dhat_rot2

        new_particles.append({'x': x, 'y': y, 'theta': theta})

    return new_particles


def eval_sensor_model(sensor_data, particles, landmarks):
    # Computes the observation likelihood of all particles, given the
    # particle and landmark positions and sensor measurements
    # (probabilistic sensor models slide 33)
    #
    # The employed sensor model is range only.

    sigma_r = 0.2

    # measured landmark ids and ranges
    ids = sensor_data['id']
    ranges = sensor_data['range']

    weights = np.array([])

    '''your code here'''
    '''***        ***'''
    for count, pt in enumerate(particles):
        w = 1.0

        for idx in range(len(ids)):
            landmark = landmarks[ids[idx]]
            z = ranges[idx]
            dist_to_landmark = np.sqrt(
                ((pt['x']-landmark[0]) ** 2) +
                ((pt['y']-landmark[1]) ** 2))

            # Faster to just compute PDF manually
            # p_z_given_x_l = scipy.stats.norm.pdf(
            #    z - dist_to_landmark, scale=sigma_r)
            loc = z - dist_to_landmark
            p_z_given_x_l = (
                np.exp(-0.5 * (loc**2))/np.sqrt(2.0*np.pi)) / sigma_r
            w = w * p_z_given_x_l

        weights = np.append(weights, w)

    # normalize weights
    normalizer = sum(weights)
    weights = weights / normalizer

    return weights


def resample_particles(particles, weights):
    # Returns a new set of particles obtained by performing
    # stochastic universal sampling, according to the particle weights.

    new_particles = []

    '''your code here'''
    '''***        ***'''
    c = []
    n = len(particles)
    for i in range(n):
        if i == 0:
            c.append(weights[i])
        else:
            c.append(c[i-1] + weights[i])
    assert(np.isclose(c[-1], 1.0))
    c[-1] = 1.0

    step = 1.0/float(n)
    u = np.random.uniform() * step
    i = 0
    for j in range(n):
        while u > c[i]:
            i += 1
        new_particles.append(particles[i])
        u += step

    return new_particles
