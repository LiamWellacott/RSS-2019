#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

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

    def intersect(self, seg):
        """
        Computes the intersection point between two segments according to
        [https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect]
            input:
            ------
                - seg: The second segment
            output:
            -------
                - intersection point if one is found, or a 2D array of nan if
                none is found.
        """
        # TODO possible optimisation of operation (from SO answer) but might not be worth it 
        t = cross((seg.p1 - self.p1), seg.vec)/cross(self.vec, seg.vec)
        s = cross((self.p1 - seg.p1), self.vec)/cross(seg.vec, self.vec)
        
        # is the crossover point not within the segment? (segment limits at t or s value 0 and 1)
        if 0.0 > t or t > 1.0 or 0.0 > s or s > 1.0:
            return np.full(2, np.nan)
        
        # There is an intersection, return the point
        return self.p1 + t*self.vec

    def __str__(self):
        return self._toArray().__str__()

    def _toArray(self):
        return np.array([self.p1, self.p2])

    def plotSeg(self, fig, ax):
        '''
        Plot function used for visualisation in debugging only
        '''
        lines = [self._toArray()]
        c = [(1,0,0,1)]
        lc = mc.LineCollection(lines, linestyles='dashed', colors=c, linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

    def plotPerp(self, fig, ax):
        '''
        Plot function used for visualisation in debugging only
        '''
        point = (self.p1 + self.p2) / 2
        norm = self.perp/np.linalg.norm(self.perp)
        ax.quiver(point[0], point[1], norm[0]/100., norm[1]/100., color=['g'], scale=0.2)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

class Map(object):
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

    def plotMap(self, fig, ax):
        '''
        Plot function used for visualisation in debugging only
        '''
        lines = []
        for seg in self.segments:
            lines.append(seg._toArray())
            fig, ax = seg.plotPerp(fig, ax)
        lc = mc.LineCollection(lines, linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

    def intersect(self, segment):
        """
        Computes the intersection points of a segment with every segment of
        the map.
            input:
            ------
                - segment: a segment.
            output:
            -------
                - a set of points which are the different intersection points.
        """
        points = []
        # TODO: is seems from profiling that intersect if evaluated very often
        # it could probably be worse to reduce the search space for intersection.
        for seg in self.segments:
            p = seg.intersect(segment)
            # the if statement doesn't look optimal. Could be worse to return
            # true or false when a intersection point is found. np.isnan().any()
            # seems to be a expensive operation.
            if not np.isnan(p).any():
                points.append(p)
        return np.array(points)

WORLD_MAP = Map("maps/rss_offset.json")

class Robot:
    """
    Robot class used to represent particles and simulate the robot to
    test the particles.
    """
    def __init__(self, map):
        """
        Initializes a particle/robot.
        input:
        ------
            - map: a map object reference.
        """
        self.x = np.random.rand() * 4.1
        self.y = np.random.rand() * 3.05
        self.yaw = np.random.uniform(-1, 1) * np.pi
        self.move_noise = 0
        self.turn_noise = 0
        self.sense_noise = 0
        self.nb_ray = 8
        self.ray_length = 5.32 # sqrt(4.25**2 + 3.20**2)
        self.world_map = map
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

    def sense(self):
        """
        Sensing for the simulated robot. This will have to be replaced with
        true sensing from the robot when integrated with ros. Extra return
        values are given for plotting and visualisation.
        input:
        ------
            None
        output:
        -------
            - lines: the set of ray used for sensing. Expressed in pair of points.
            - points: the intersection point of each ray with the world.
            - distances: the measured distances from the robot to the sensed point.
        """
        lines = []
        points = []
        distances = []
        for angle in np.linspace(self.yaw, self.yaw + 2*np.pi - (2*np.pi/self.nb_ray), self.nb_ray):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment(np.array([self.x, self.y]), np.array([x, y]))
            pts = self.world_map.intersect(seg)
            if not pts.size == 0:
                dist = np.linalg.norm(pts - np.array([self.x, self.y]), axis=1)
                pt = pts[np.argmin(dist)]
                dist = np.amin(dist)
                distances.append(dist + np.random.rand()*self.sense_noise)
                points.append(pt)
                lines.append([(self.x, self.y), (pt[0], pt[1])])
            else:
                lines.append([(self.x, self.y), (x, y)])
        return lines, points, distances

    def plotRay(self, fig, ax):
        '''
        Plot function used for visualisation in debugging only
        '''
        lines, points, _ = self.sense()
        c = [(1,0,0,1), (0,1,0,1), (0,0,1,1), (1,1,0,1), (1,0,1,1), (0,1,1,1), (0,0,0,1), (0,0,0,1)]
        lc = mc.LineCollection(lines, colors=c, linestyles='dashed', linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        for point in points:
            ax.plot(point[0], point[1], 'ro')
        return fig, ax

    def plotRobot(self, fig, ax):
        '''
        Plot function used for visualisation in debugging only
        '''
        ax.scatter(self.x, self.y, c='r', s=500)
        ax.quiver(self.x, self.y, 5,5, angles=np.rad2deg(self.yaw), scale=1/5, scale_units="dots", units="dots", color="k", pivot="mid",width=2.5, headwidth=5, headlength=2.5)
        return fig, ax

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
        # TODO is it possible to halve the work if we centre the segment on the robot position and look both ways?
        prob = 1.0
        for i, angle in enumerate(np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray)):
            
            # Create a line segment to represent the laser beam
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment(np.array([self.x, self.y]), np.array([x, y]))

            # Get the list of intersections and save the distance to the closest point
            pts = self.world_map.intersect(seg)
            if not pts.size == 0:
                dist = np.linalg.norm(pts - np.array([self.x, self.y]), axis=1)
                dist = np.amin(dist)
                prob *= gaussian(dist, self.sense_noise, m[i])
            else:
                # no intersection found probability is 0
                prob *= 0
        return prob

    def move(self, turn, forward):
        """
        Simple model to move the robot. This will be change when integrated with
        ROS. This function is only there to update the robot state.
        input:
        ------
            - turn: the yaw change.
            - forward: the quantiy of forward mvt.
        output:
        -------
            None
        """
        if forward < 0:
            raise ValueError('Robot cant move backwards')

        # turn, and add Gaussian noise to the turning command
        orientation = self.yaw + float(turn) + np.random.randn()*self.turn_noise
        orientation %= 2 * np.pi # make sure: 0=< orientation <=2*pi

        # move, and add Gaussian noise to the motion command
        dist = float(forward) + np.random.randn()*self.move_noise
        x = self.x + (np.cos(orientation) * dist)
        y = self.y + (np.sin(orientation) * dist)

        # set the new location x, y back to the member variables x y of the class
        self.setPose(x, y, orientation)
        return

class ParticleFilter(object):
    """
    Particle filter class. Manages a set of particles.
    """
    def __init__(self, nb_p):
        """
        Initialize the set of paritcles.
        input:
        ------
            nb_p: the number of particles
        """
        self.particles = []
        self.w = []

        for _ in range(nb_p):
            r = Robot(WORLD_MAP)
            # TODO estimate the std for the different operations
            r.setNoise(0.1, 0.1, 0.5)
            self.particles.append(r)

    def actionUpdate(self, action):
        """
        Update the particles position after a new transition operation.
        input:
        ------
            action: the set of action [turn, forward]. This will have to be
            changed when integration with ROS.
        """
        for p in self.particles:
            p.move(action[0], action[1])

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
        self.particles = resampling(self.particles, self.w)

    def plotParticles(self, fig, ax):
        
        # Plotting the particles
        for p in self.particles:
            plt.scatter(p.x, p.y, c='b')
            plt.quiver(p.x, p.y, 1,1, angles=np.rad2deg(p.yaw), scale=1/5, scale_units="dots",
            units="dots", color="y", pivot="mid", width=1.25, headwidth=2, headlength=0.5)
            return fig, ax

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

def plotPts(pts, fig, ax):
    for pt in pts:
        ax.plot(pt[0], pt[1], 'ro')
    return fig, ax

def main():
    r = Robot(WORLD_MAP)
    r.setPose(3.9, 1.25, np.pi)
    r.setNoise(0.0, 0.0, 0.0)

    steps = 30
    heading = 0
    steplength = .1

    # Initialize the state estimator
    estimator = ParticleFilter(50)
    # plot robot, environment and particles
    fig, ax = plt.subplots()
    fig, ax = r.plotRobot(fig, ax)
    fig, ax = r.world_map.plotMap(fig, ax)
    fig, ax = r.plotRay(fig, ax)
    #plt.show()


    # for each step update the belief
    for _ in range(steps):
        # Implement the particle filter algorithm
        # move robot
        r.move(heading, steplength)

        # update pose of each particle according to the motion model
        estimator.actionUpdate([heading, steplength])

        # obtain a sensor reading
        _, _, measure = r.sense()
        fig, ax = plt.subplots()
        fig, ax = r.plotRay(fig, ax)
        fig, ax = r.plotRobot(fig, ax)
        fig, ax = r.world_map.plotMap(fig, ax)
        #plt.show()

        # update the weights of the particles according to the measrement model
        estimator.measurementUpdate(measure)

        # update the particles according to resampling process
        estimator.particleUpdate()

        #fig, ax = plt.subplots()
        #fig, ax = plotParticles(estimator.p, fig, ax)
        #fig, ax = r.plotRobot(fig, ax)
        #fig, ax = r.world_map.plotMap(fig, ax)
        #plt.show()
        #print('Step: ',  s)
        #print('Robot location:', r)
        #print("Mean error:",evaluation(r, estimator.p))


if __name__ == "__main__":
    #fig, ax = plt.subplots()
    #fig, ax = WORLD_MAP.plotMap(fig, ax)
    #seg = Segment(np.array([0, 0]), np.array([5, 5]))
    #r = Robot(WORLD_MAP)
    #r.setPose(.5, .5, .5)
    #fig, ax = r.plotRay(fig, ax)
    #pts = WORLD_MAP.intersect(seg)
    #fig, ax = seg.plotSeg(fig, ax)
    #fig, ax = plotPts(pts, fig, ax)
    #plt.show()
    main()
