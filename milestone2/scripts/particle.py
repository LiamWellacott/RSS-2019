#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy
import rospy

# Message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from gazebo_msgs.msg import ModelStates
import tf
import sys
import rospkg

#from utile import Segment
from utile import Map
# initial position in the map as per the brief
INITIAL_X = 0.561945
INITIAL_Y = 0.509381
INITIAL_YAW = 0.039069

# relative path from package directory
MAP_FILE = "/maps/rss_offset.json"

NUM_RAYS = 8
NUM_PARTICLES = 50

PUBLISH_RATE = 0.1

ODOM_RATE = 30

NOISE_MOVE = 0.05
NOISE_TURN = 0.05
NOISE_SENSE = 0.5

MAX_VAL = 150

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
        for i, angle in enumerate(np.linspace(self.yaw, self.yaw + np.pi - (2*np.pi/self.nb_rays), self.nb_rays/2)):

            # Get the minimum intersections in front and behind the robot at this angle
            points, distances = self.map.minIntersections(self, angle)

            if points[0] is None or points[1] is None:
                    # no intersection found indicating the robot is outside the arena
                    # probability is 0 for whole robot
                    return 0
            else:
                # calculate probability of measurement
                prob *= gaussian(distances[0], self.sense_noise, m[i])
                prob *= gaussian(distances[1], self.sense_noise, m[i + int(self.nb_rays/2)])
        return prob

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
        dt = 1/ODOM_RATE
        # If angular velocity is close to 0 we use the simpler motion model
        # equation derived from http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
        if yaw_vel > 1e-6:
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
        self.x += np.random.uniform(-1, 1) * self.move_noise
        self.y += np.random.uniform(-1, 1) * self.move_noise
        self.yaw += np.random.uniform(-1, 1) * self.turn_noise
        return

class ParticleFilter(object):
    """
    Particle filter class. Manages a set of particles.
    """
    def __init__(self, map, nb_p, x = 0, y = 0, yaw = 0, nb_rays = 8):
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
            p.setNoise(NOISE_MOVE, NOISE_TURN, NOISE_SENSE)
            self.particles.append(p)

        ### DATA SAVE FOR VISUALISATION ###
        self.dict = {}
        self.counter = 0
        self.MAX_VAL = MAX_VAL
        self.true_x = 0
        self.true_y = 0
        self.true_yaw = 0
        rospy.Subscriber("gazebo/model_states", ModelStates, self.modelCB)

    def modelCB(self, msg):
        j = 0
        for i, s in enumerate(msg.name):
            if s == "turtlebot3":
                j = i
        pose = msg.pose[j]
        self.true_x = pose.position.x
        self.true_y = pose.position.y
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        self.true_yaw = tf.transformations.euler_from_quaternion(orientation)[2]

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
        for p in self.particles:
            # get the measurement probability for each particle
            w.append(p.measureProb(mt))
        # normailze the weights.
        rospy.loginfo("w = {}".format(w))
        self.w = np.array(w)/np.sum(w)

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
        if self.counter < self.MAX_VAL:
            rospy.loginfo("UPDATE")
            rospy.loginfo("{} / {}".format(self.counter, self.MAX_VAL))
            self.updateData()
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
        x = 0
        y = 0
        yaw = 0
        for i, p in enumerate(self.particles):
            x += self.w[i]*p.x
            y += self.w[i]*p.y
            yaw += self.w[i]*p.yaw
        return x, y, yaw

    def updateData(self):
        parts = []
        robot = {"x" : self.true_x, "y" : self.true_y, "yaw" : self.true_yaw}
        print(len(self.particles))
        for j, p in enumerate(self.particles):
            a = {j: [p.x, p.y, p.yaw]}
            parts.append(a)
        self.dict.update({self.counter: parts, str(self.counter) + "robot": robot})

        self.counter += 1
        if self.counter >= self.MAX_VAL:
            rospy.loginfo("DUMP FUCKING DATA")
            self.dumpData("test.json")

    def dumpData(self, file_path):
        with open(file_path, 'w') as file:
            json.dump(self.dict, file)

class Robot(object):
    """
    Robot class used to represent particles and simulate the robot to
    test the particles.
    """
    def __init__(self, map, nb_p, x, y, yaw, nb_rays = 8):
        """
        Initializes a particle/robot.
        input:
        ------
            - map: a map object reference.
        """
        # initialise
        rospy.init_node("milestone2", anonymous=True)

        # subscribe
        rospy.Subscriber("scan", LaserScan, self.scanCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)

        # Pose publisher, initialise message
        self.pose_pub = rospy.Publisher('pf_pose', Twist, queue_size = 10)
        self.pose_msg = Twist()
        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw
        # timer for pose publisher
        rospy.Timer(rospy.Duration(PUBLISH_RATE), self.pubPose)

        # set initial position
        self.x = x
        self.y = y
        self.yaw = yaw

        self.dict={}

        self.counter = 0

        # Initialise particle filter
        self.nb_rays = nb_rays
        self.map = map
        self.particle_filter = ParticleFilter(map, nb_p, x, y, yaw, nb_rays)

        rospy.loginfo("Started particle filter node")
        while not rospy.is_shutdown():
            rospy.sleep(10)
        return

    def scanCallback(self, msg):

        # get the measurements for the specified number of points out of the scan information
        indexes = np.rint(np.linspace(0, 360 - 360/self.nb_rays, self.nb_rays)).astype(int)
        m = np.array(msg.ranges)
        measure = m[indexes]

        # update position estimation
        self.poseEstimationUpdate(measure)
        return

    def odomCallback(self, msg):

        # add the received position increment to the particles
        vel = msg.twist.twist
        self.particle_filter.actionUpdate(vel.linear.x, vel.linear.y, vel.angular.z)

        values = [vel.linear.x, vel.linear.y, vel.angular.z]
        self.dict.update({str(self.counter) : values})
        self.counter+=1
        self.dumpData("values.json")

        return

    def dumpData(self, file_path):
        with open(file_path, 'w') as file:
            json.dump(self.dict, file)

    def poseEstimationUpdate(self, measurements):

        self.particle_filter.measurementUpdate(measurements)
        self.particle_filter.particleUpdate()
        x, y, yaw = self.particle_filter.estimate()

        rospy.logdebug("x = {}, y = {}, yaw = {}".format(x, y, yaw))

        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw
        return

    def pubPose(self, event):
        self.pose_pub.publish(self.pose_msg)
        return

    def __str__(self):
        return "x {}, y {}, yaw {} ".format(self.x, self.y, self.yaw)

    def setPose(self, x, y, yaw):
        """
        Sets a new pose for the robot.
        input:
        ------
            - x: the new x position.
            - y: the new y position.
            - yaw: the new yaw angle.
        output:
        -------
            None
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        return

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

def main():

    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    r = Robot(map, NUM_PARTICLES, INITIAL_X, INITIAL_Y, INITIAL_YAW, NUM_RAYS)

if __name__ == "__main__":
    main()
