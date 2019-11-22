#!/usr/bin/env python

import rospy
import numpy as np
import json
from copy import copy as copy

# Message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#from gazebo_msgs.msg import ModelStates
import tf
import sys
import rospkg

PUBLISH_RATE = 0.1

ODOM_RATE = 30.

NOISE_MOVE = 0.02
NOISE_TURN = np.deg2rad(1)
NOISE_SENSE = 0.05

SENSE_DIST = 0.06 # 6 cm offset between the wheel center and the sensor

MIN_VALID_MEASUREMENT = 0.12

class Particle(object):
    """
    Particle class.
    """
    def __init__(self, map, x, y, yaw, x_pert = 0, y_pert = 0, yaw_pert = 0, nb_rays = 8):
        """
        Initializes a particle/robot.
        input:
        ------
            - map: a map object reference.
            - x: initial x position of the robot.
            - y: initial y position of the robot.
            - yaw: initial yaw angle of the robot.
            - x_pert: perturbation around the x position. Default = 0.
            - y_pert: perturbation around the y position. Default = 0.
            - yaw_pert: perturbation around the yaw angle. Default = 0.
            - nb_rays: nomber of rays used for the measurements. Default = 8.
        """
        # Initialize particle arround the robot initial pose.
        self.x = x + np.random.rand()*x_pert
        self.y = y + np.random.rand()*y_pert
        self.yaw = yaw + np.random.rand()*yaw_pert
        self.xSens = self.x - np.cos(self.yaw)*SENSE_DIST
        self.ySens = self.y - np.sin(self.yaw)*SENSE_DIST

        # Noise for sensing and moving
        self.move_noise = 0
        self.turn_noise = 0
        self.sense_noise = 0

        # Number of rays used to get the measurements.
        self.nb_rays = nb_rays

        # Map of the world
        self.map = map

        return

    def __str__(self):
        return "x {}, y {}, yaw {} ".format(self.x, self.y, self.yaw)

    def setNoise(self, move, turn, sense):
        """
        Sets a new pose for the robot.
        input:
        ------
            - move: noise when moving forward.
            - turn: noise when turning.
            - sense: noise when sensing.
        output:
        -------
            None
        """
        self.move_noise = move
        self.turn_noise = turn
        self.sense_noise = sense
        return

    def _filterMeasurement(self, ideal, measurement):
        if measurement < MIN_VALID_MEASUREMENT or measurement == float('inf'):
            return 1 # there is an error in this measurement, don't include it in the model for this timestep
        else:
            return gaussian(ideal, self.sense_noise, measurement)

    def measureProb(self, m):
        """
        measures the probability of geting a measurement m when in state x.
        @f$(p(m_t | x))@f$. Where x is the state of the robot.
        input:
        ------
            m: a set of measurement.
        output:
        -------
            p(m|x)
        """
        prob = 1.0
        d = []
        for i, angle in enumerate(np.linspace(self.yaw, self.yaw + np.pi - (2*np.pi/self.nb_rays), self.nb_rays/2)):

            # Get the minimum intersections in front and behind the robot at this angle
            points, distances = self.map.minIntersections(self, angle)
            # debug info
            d.append(distances)

            if points[0] is None or points[1] is None:
                    # no intersection found indicating the robot is outside the arena
                    # probability is 0 for whole robot
                    #return 0
                    prob = 0
            else:
                # calculate probability of measurement
                prob *= self._filterMeasurement(distances[0], m[i])
                prob *= self._filterMeasurement(distances[1], m[i + int(self.nb_rays/2)])
                #if prob == 0:
                    #return 0

        return prob, d

    def move(self, x_vel, y_vel, yaw_vel):
        """
        Updates the particle position according to the last messages on the
        /odom topic
        input:
        ------
            - x: the change in x.
            - y: the change in y.
            - yaw: the change in yaw
        output:
        -------
            None
        """
        dt = 1./ODOM_RATE
        # If angular velocity is close to 0 we use the simpler motion model
        # equation derived from http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
        if np.abs(yaw_vel) > 1e-6:
            # Compute the rotational radius
            r = x_vel/yaw_vel
            # Instantaneous center of curvature
            icc = [self.x - r*np.sin(self.yaw), self.y + r*np.cos(self.yaw)]
            wdt = yaw_vel*dt
            self.x = (self.x - icc[0])*np.cos(wdt) - (self.y - icc[1])*np.sin(wdt) + icc[0]
            self.y = (self.x - icc[0])*np.sin(wdt) + (self.y - icc[1])*np.cos(wdt) + icc[1]
            self.yaw = self.yaw + wdt
        else:
            self.x += x_vel*np.cos(self.yaw)*dt
            self.y += x_vel*np.sin(self.yaw)*dt
            # yaw remains constant when no angular velocity
        # add some noise to the update
        self.yaw += np.random.uniform(-1, 1) * self.turn_noise
        self.x += np.random.uniform(-1, 1) * self.move_noise * np.cos(self.yaw)
        self.y += np.random.uniform(-1, 1) * self.move_noise * np.sin(self.yaw)

        self.xSens = self.x - np.cos(self.yaw)*SENSE_DIST
        self.ySens = self.y - np.sin(self.yaw)*SENSE_DIST

        return

class ParticleFilter(object):
    """
    Particle filter class. Manages a set of particles.
    """
    def __init__(self, map, nb_p,
                 x = 0, y = 0, yaw = 0, nb_rays = 8,
                 m_noise=NOISE_MOVE, t_noise=NOISE_TURN, s_noise=NOISE_SENSE):
        """
        Initialize the set of paritcles.
        input:
        ------
            nb_p: the number of particles
        """
        self.particles = []
        self.weights = []

        # estimated value of the robot pose
        self.x_est = x
        self.y_est = y
        self.yaw_est = yaw

        for _ in range(nb_p):
            p = Particle(map, x, y, yaw)
            # TODO estimate the std for the different operations
            p.setNoise(m_noise, t_noise, s_noise)
            self.particles.append(p)

    def actionUpdate(self, x_vel, y_vel, yaw_vel):
        """
        Update the particles position after a new transition operation.
        input:
        ------
            action: the set of action [turn, forward]. This will have to be
            changed when integration with ROS.
        """
        for p in self.particles:
            p.move(x_vel, y_vel, yaw_vel)

    def measurementUpdate(self, mt):
        """
        COmpute the weight of each particle given the current measurement.
        input:
        ------
            - mt: the current measurement.
        output:
        -------
            none
        """
        # the set of weights
        w = []
        ray = []
        for p in self.particles:
            # get the measurement probability for each particle
            prob, d = p.measureProb(mt)
            ray.append(d)
            w.append(prob)
        self.w = np.array(w)/np.sum(w)
        return ray, w

    def particleUpdate(self):
        """
        Resampleing process after probability measurement.
        input:
        ------
            None
        output:
        -------
            None
        """
        # Resample TODO implement
        N = len(self.particles)
        beta=0
        j=0
        w_max= max(self.w)
        p_temp=[]
        for _ in range(N):
            beta += 2.0*w_max*np.random.rand()
            while beta>self.w[j]:
                beta -= self.w[j]
                j=(j + 1) % N
            selectedParticle = copy(self.particles[j])
            p_temp.append(selectedParticle) # if beta<w[index], this indexed particle is selected
        self.particles = p_temp

    def estimate(self):
        w = np.array(self.w)
        i = np.argmax(w)
        x = self.particles[i].x
        y = self.particles[i].y
        yaw = self.particles[i].yaw
        #for i, p in enumerate(self.particles):
        #    x += self.w[i]*p.x
        #    y += self.w[i]*p.y
        #    yaw += self.w[i]*p.yaw
        return x, y, yaw

    def getPositions(self):
        pos = []
        for p in self.particles:
            pos.append((p.x, p.y, p.yaw))
        return pos

def gaussian(mu, sigma, x):
    """
    Computes the probability of x w.r.t a gaussian law @f$(\mathcal{N}(/mu,/sigma^(2)))@f$
    input:
    -----
    - mu: the mean of the distribution.
    - sigma: the standart deviation.
    - x: the evaluation point.
    output:
    -------
    - the probability of x.
    """
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))
