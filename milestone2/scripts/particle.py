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

import rospkg

# class using a vectorial represnetation for the segments
# p1 the origin and p2 - p1 the orientation and length associated
# to it.
class Segment(object):
    """
    Class to represent a vector. This class alows operation such as checking
    intersection with two vectors.
    """
    def __init__(self, p1, p2):
        """
        Segment constructor.
            input:
            ------
                - first point p1 as array [x1, y1]
                - second point p2 as array [x2, y2]
            output:
            -------
                - None
        """
        self.p1 = p1
        self.p2 = p2
        self.vec = p2 - p1

        # Store the vectors perpendicular for collision calculation
        self.perp = np.copy(self.vec)
        tmp = self.perp[0]
        self.perp[0] = -self.perp[1]
        self.perp[1] = tmp

    def __str__(self):
        return self._toArray().__str__()

    def _toArray(self):
        return np.array([self.p1, self.p2])

class Map(object):

    LENGTH = 5.32 # sqrt(4.25**2 + 3.20**2)

    """
    Class representing the map. It has a set of segments.
    """
    def __init__(self, map_file):
        """
        Loads a map from a map file. The file is assumed to be  a json file
        and the structure should be as follows:
        - first entry has the key word 'segments'. This entry is an array of
        segments. A segment is an array of points and a point is an array of
        x and y coordinates.
            input:
            ------
             - map_file: path to the json map file.
        """
        self.segments = []
        with open(map_file) as json_file:
            data = json.load(json_file)
            for seg in data['segments']:
                p1 = np.array(seg[0])
                p2 = np.array(seg[1])
                s = Segment(p1, p2)
                self.segments.append(s)

    def __str__(self):
        ret = ""
        for seg in self.segments:
            ret += seg.__str__()
            ret += "\n"
        return ret

    def _createLaser(self, particle, angle):
        '''
            input:
            ------
                - particle : The Robot which is the object of the ray trace, used for position
                - angle : what angle relative to the current yaw to cast the ray through the robot
            output:
            -------
                - a segment which passes through the world map with the robot at the halfway point
                This allows us to calculate 2 data points with a single ray trace 
        '''
        end_x = np.cos(angle)*self.LENGTH + particle.x
        end_y = np.sin(angle)*self.LENGTH + particle.y

        start_x = -np.cos(angle)*self.LENGTH + particle.x
        start_y = -np.sin(angle)*self.LENGTH + particle.y

        return Segment(np.array([start_x, start_y]), np.array([end_x, end_y]))

    def minIntersections(self, particle, angle):
        '''
            input:
            ------
                - particle : The Robot which is the focus of the ray trace, used for position
                - angle : what angle relative to the current yaw to cast the ray through the robot
            output:
            -------
                - a 2rry of the pairs (point, distance), which represents the two minimum points of intersection in front of the robot and behind.
                If no valid intersection is found these points will be "None".
                        
        Computes the intersection point between two segments according to
        [https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect]
        '''
        # TODO REVIEW used variables to store the distance when point is sufficient
        # this was to reduce the number of distance calculations, but makes the code not as clean.
        # Possibly change the WORLD_MAP.LENGTH to a World.LENGTH and make the robot use this also.
        # not sure on the type used for return

        # Create a line segment to represent the laser beam
        ray = self._createLaser(particle, angle)

        # initialise minimum intersection vars
        min_forward_point = min_backward_point = None
        min_forward_dist = min_backward_dist = self.LENGTH

        # check collisions of ray with each of the worlds walls
        for wall in self.segments:

            prod = cross(wall.vec, ray.vec)
            if prod == 0: # filter rare case of parallel
                continue

            # find scalar offsets which indicate crossover point
            t = cross((ray.p1 - wall.p1), ray.vec)/prod
            s = cross((wall.p1 - ray.p1), wall.vec)/-prod # because cross(wall.vec, ray.vec) = -cross(ray.vec, wall.vec)
            
            # is the crossover point not within the segment? (segment limits at t or s value 0 and 1)
            if 0.0 > t or t > 1.0 or 0.0 > s or s > 1.0:
                continue
            
            # There is an intersection, get the point by applying the offset
            point = wall.p1 + t*wall.vec

            # measure distance to the point of intersection from the robot
            dist = np.linalg.norm(point - np.array([particle.x, particle.y]))

            # if scalar is more than halfway (0.5) the obstacale is in front of robot, otherwise behind.
            # check if the newly found intersection is closer than previously measured points (for a given direction)
            if s >= 0.5 and dist < min_forward_dist:
                min_forward_point = point
                min_forward_dist = dist
            elif s < 0.5 and dist < min_backward_dist:
                min_backward_point = point
                min_backward_dist = dist

        # return the min intersections in both directions (default is at length of the ray)
        return [(min_forward_point, min_forward_dist), (min_backward_point, min_backward_dist)]
        
class Particle:
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

        # TODO: THIS IS TEMPORARY, NEED TO BE REMOVED AFTER TESTING
        self.x = np.random.rand()*4.1
        self.y = np.random.rand()*3.05
        self.yaw = np.random.uniform(-1, 1) * np.pi

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
            for point, distance in self.map.minIntersections(self, angle):
                if point is None :
                    # no intersection found indicating the robot is outside the arena
                    # probability is 0 for whole robot
                    return 0
                else:
                    # calculate probability of measurement 
                    prob *= gaussian(distance, self.sense_noise, m[i])
        return prob

    def move(self, x, y, yaw):
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
        self.x += x + np.random.rand() * self.move_noise
        self.y += y + np.random.rand() * self.move_noise
        self.yaw += yaw + np.random.rand() * self.turn_noise
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
        self.w = []

        # estimated value of the robot pose
        self.x_est = x
        self.y_est = y
        self.yaw_est = yaw

        for _ in range(nb_p):
            p = Particle(map, 0, 0, 0)
            # TODO estimate the std for the different operations
            p.setNoise(0.1, 0.1, 0.5)
            self.particles.append(p)

    def actionUpdate(self, action):
        """
        Update the particles position after a new transition operation.
        input:
        ------
            action: the set of action [turn, forward]. This will have to be
            changed when integration with ROS.
        """
        for p in self.particles:
            p.move(action[0], action[1], action[2])

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
        self.w = w/np.sum(w)

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
        self.particles = resampling(self.particles, self.w) #TODO resampling should be in the filter

    def estimate(self):
        x = 0
        y = 0
        yaw = 0
        for i, p in enumerate(self.particles):
            x += self.w[i]*p.x
            y += self.w[i]*p.y
            yaw += self.w[i]*p.yaw
        return x, y, yaw

class Robot(object):
    """
    Robot class used to represent particles and simulate the robot to
    test the particles.
    """
    def __init__(self, map, x, y, yaw, nb_rays = 8):
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

        # Pose publisher
        self.pose_pub = rospy.Publisher('pf_pose', Twist, queue_size = 10)
        self.pose_msg = Twist()
        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw
        # timer for pose publisher
        rospy.Timer(rospy.Duration(0.1), self.pubPose)

        self.x = x
        self.y = y
        self.yaw = yaw

        self.x_twist = 0
        self.y_twist = 0
        self.yaw_twist = 0

        self.estimate = False
        self.nb_rays = nb_rays
        self.map = map
        self.particle_filter = ParticleFilter(map, 50, x, y, yaw, nb_rays)
        self.mt = np.zeros((self.nb_rays,))

        while not rospy.is_shutdown():
            rospy.sleep(10)
        return

    def scanCallback(self, msg):
        measure = np.zeros((self.nb_rays,))
        indexes = np.rint(np.linspace(0, 360 - 360/self.nb_rays, self.nb_rays)).astype(int)
        m = np.array(msg.ranges)
        measure = m[indexes]
        self.mt = measure
        self.poseEstimationUpdate()
        return

    def odomCallback(self, msg):
        twist = msg.twist
        twist = twist.twist
        self.x_twist += twist.linear.x
        self.y_twist += twist.linear.y
        self.yaw_twist += twist.angular.z
        return

    def poseEstimationUpdate(self):
        self.particle_filter.actionUpdate([self.x_twist, self.y_twist, self.yaw_twist])
        self.x_twist = 0
        self.y_twist = 0
        self.yaw_twist = 0

        self.particle_filter.measurementUpdate(self.mt)
        self.particle_filter.particleUpdate()
        x, y, yaw = self.particle_filter.estimate()

        rospy.loginfo("x {}, y {}, yaw {}".format(x, y, yaw))
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

def cross(v, w):
    """
    Cross product for 2D vector. This function asssumes that the given vector is in 2D.
    input:
    ------
    - v: first vector.
    - w: second vector.
    output:
    -------
    - cross product (@f$(v_x * w_y - v_y * w_x)@f$)
    """
    return v[0]*w[1] - v[1]*w[0]

def resampling(p, w):
    """
    Resampling operation.
    input:
    ------
        - p: a set of particles
        - w: a set of weight associated with each partilces.
    output:
    -------
        - resampled set of particles.
    """
    # TODO: Probably better if the function is put inside the ParticleFilter
    N = len(p)
    beta=0
    j=0
    w_max= max(w)
    p_temp=[]
    for _ in range(N):
        beta += 2.0*w_max*np.random.rand()
        while beta>w[j]:
            beta -= w[j]
            j=(j + 1) % N
        selectedParticle = copy(p[j])
        p_temp.append(selectedParticle) # if beta<w[index], this indexed particle is selected
    return p_temp

def evaluation(robot, particles):
    # Gives the mean error in position between the robot's
    # actual position and the set of particles
    sum = 0.0
    for p in particles: # calculate mean error
        dx = (p.x - robot.x)
        dy = (p.y - robot.y)
        err = np.sqrt(dx**2 + dy**2)
        sum += err
    return sum / float(len(particles))

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + "/maps/rss_offset.json")
    r = Robot(map, 0, 0, 0, 8)

if __name__ == "__main__":
    main()
