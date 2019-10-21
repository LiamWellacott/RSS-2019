#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

# class using a vectorial represnetation for the segments
# p1 the origin and p2 - p1 the orientation and length associated
# to it.
class Segment(object):
    """
    Segment representation class.
    """
    def __init__(self, p1, p2):
        """
        Segment constructor.
            input:
            -----
                - first point p1 as array [x1, y1]
                - second point p2 as array [x2, y2]
        """
        # p1 is always the point with the smallest x
        if p1[0] < p2[0]:
            self.p1 = p1
            self.p2 = p2
        else:
            self.p1 = p2
            self.p2 = p1
        self.vert = False
        # special case where we assume the segment to be vertical.
        if np.abs(p1[0] - p2[0]) < 0.0001:
            self.vert = True
            self.m = np.nan
            self.b = np.nan
        else:
            # precomputation of the slope and intersect to speed up
            # the computation in the intersection process
            self.m = (self.p1[1] - self.p2[1]) / (self.p1[0] - self.p2[0])
            self.b =  self.p1[1] - self.m*self.p1[0]
        self.empty = np.full(2, np.nan)

    def _orientation(self, p):
        """
        Computes the orientation of 3 given points.
        Either in clockwise order, counterclockwise or colinear.
            input:
            ------
                - p: a point in space.
            output:
            -------
                - a value between 0 and 2. 0 if colinear, 1 if clockwise and 2 if counterclockwise.
        """
        val = (self.p2[1] - self.p1[1]) * (p[0] - self.p2[0]) - \
              (self.p2[0] - self.p1[0]) * (p[1] - self.p2[1])
        if val == 0:
            return 0
        elif val > 0:
            return 1
        else:
            return 2

    def _isIntersect(self, seg):
        """
        Checks if a segment and self intersect.
            input:
            ------
                - seg: a segment.
            output:
            -------
                - boolean: True if intersection, False otherwise.
        """
        o1 = self._orientation(seg.p1)
        o2 = self._orientation(seg.p2)
        o3 = seg._orientation(self.p1)
        o4 = seg._orientation(self.p2)
        if o1 != o2 and o3 != o4:
            return True
        return False

    def intersect(self, seg):
        """
        Computes the intersection point of a segments and self.
        input:
        -----
            - seg: a segment
        output:
        -------
            - point [x, y] of the intersection if one found or a array filed with
            nan if none found.
        """
        # We consider the segments as lines to find intersection point and then
        # we check if the given point is on the two segments.
        if self.vert and seg.vert:
            # both lines a verticals, therefore parallel
            return self.empty

        if not self.vert and not seg.vert:
            # none of the lines are vertical.
            if self.m - seg.m == 0:
                # lines are paralle
                return self.empty
            # setting y to be equal in the two lines to find the intersection
            # point.
            x = (seg.b - self.b) / (self.m - seg.m)
            y = self.m*x + self.b

        # One of the two lines is vertical. We use the other line's slope and
        # intercept to find the intersection point.
        if self.vert:
            y = seg.m * self.p1[0] + seg.b
            x = self.p1[0]

        if seg.vert:
            y = self.m * seg.p1[0] + self.b
            x = seg.p1[0]

        x1a = self.p1[0]
        x1b = self.p2[0]

        x2a = seg.p1[0]
        x2b = seg.p2[0]

        y1a = min(self.p1[1], self.p2[1])
        y1b = max(self.p1[1], self.p2[1])

        y2a = min(seg.p1[1], seg.p2[1])
        y2b = max(seg.p1[1], seg.p2[1])
        if x1b < x or x < x1a or x2b < x or x < x2a or \
           y1b < y or y < y1a or y2b < y or y < y2a:
            # Check if the computed point is located on both of the segments.
            return self.empty

        return np.array([x, y])

    def __str__(self):
        return self._toArray().__str__()

    def _toArray(self):
        return np.array([self.p1, self.p2])

    def plotSeg(self, fig, ax):
        lines = [self._toArray()]
        c = [(1,0,0,1)]
        lc = mc.LineCollection(lines, linestyles='dashed', colors=c, linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

class Segment2(object):
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
        #orientation always from p1 to p2
        #if p1[0] < p2[0]:
        #    self.p1 = p1
        #    self.p2 = p2
        #    self.vec = p2 - p1
        #else:
        #    self.p1 = p2
        #    self.p2 = p1
        #    self.vec = p1 - p2
        self.p1 = p1
        self.p2 = p2
        self.vec = p2 - p1
        self.perp = np.copy(self.vec)
        tmp = self.perp[0]
        self.perp[0] = -self.perp[1]
        self.perp[1] = tmp
        return

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
        if np.dot(self.perp, seg.vec) >= 0:
            return np.full(2, np.nan)

        t = cross((seg.p1 - self.p1), seg.vec)/cross(self.vec, seg.vec)
        s = cross((self.p1 - seg.p1), self.vec)/cross(seg.vec, self.vec)
        # intersection
        if 0.0 > t or t > 1.0 or 0.0 > s or s > 1.0:
            return np.full(2, np.nan)
        return self.p1 + t*self.vec

    def __str__(self):
        return self._toArray().__str__()

    def _toArray(self):
        return np.array([self.p1, self.p2])

    def plotSeg(self, fig, ax):
        lines = [self._toArray()]
        c = [(1,0,0,1)]
        lc = mc.LineCollection(lines, linestyles='dashed', colors=c, linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

    def plotPerp(self, fig, ax):
        point = (self.p1 + self.p2) / 2
        norm = self.perp/np.linalg.norm(self.perp)
        #lines = [[point, point+(self.perp/np.sum(self.perp))]]
        #c = [(0,1,0,1)]
        #lc = mc.LineCollection(lines, linestyles='dotted', colors=c, linewidth=2)
        #ax.add_collection(lc)
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
                s = Segment2(p1, p2)
                self.segments.append(s)

    def __str__(self):
        ret = ""
        for seg in self.segments:
            ret += seg.__str__()
            ret += "\n"
        return ret

    def plotMap(self, fig, ax):
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
        self.x = np.random.uniform(-1, 1) * 2.0
        self.y = np.random.uniform(-1, 1) * 1.5
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
        for angle in np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment2(np.array([self.x, self.y]), np.array([x, y]))
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
        lines, points, _ = self.sense()
        value = np.empty((), dtype=object)
        c = [(1,0,0,1), (0,1,0,1), (0,0,1,1), (1,1,0,1), (1,0,1,1), (0,1,1,1), (0,0,0,1), (0,0,0,1)]
        lc = mc.LineCollection(lines, colors=c, linestyles='dashed', linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        for point in points:
            ax.plot(point[0], point[1], 'ro')
        return fig, ax

    def plotRobot(self, fig, ax):
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
        distances = []
        prob = 1.0
        for i, angle in enumerate(np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray)):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment2(np.array([self.x, self.y]), np.array([x, y]))
            pts = self.world_map.intersect(seg)
            if not pts.size == 0:
                dist = np.linalg.norm(pts - np.array([self.x, self.y]), axis=1)
                dist = np.amin(dist)
                prob *= gaussian(dist, self.sense_noise, m[i])
            else:
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
        self.p = []
        self.w = []

        for _ in range(nb_p):
            r = Robot(WORLD_MAP)
            # TODO estimate the std for the different operations
            r.setNoise(0.1, 0.1, 0.5)
            self.p.append(r)

    def actionUpdate(self, action):
        """
        Update the particles position after a new transition operation.
        input:
        ------
            action: the set of action [turn, forward]. This will have to be
            changed when integration with ROS.
        """
        for p in self.p:
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
        for p in self.p:
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
        self.p = resampling(self.p, self.w)

    def plotParticles(self, fig, ax):
        # Plotting the particles
        for p in self.p:
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
    for i in range(N):
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
    sum = 0.0;
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
    r.setPose(-.5, .5, 0)
    r.setNoise(0.0, 0.0, 0.0)

    steps = 100
    heading = 15.0/180.0*np.pi
    steplength = .1

    # Initialize the state estimator
    estimator = ParticleFilter(50)
    # plot robot, environment and particles
    #fig, ax = plt.subplots()
    #fig, ax = plotParticles(estimator.p, fig, ax)
    #fig, ax = r.plotRobot(fig, ax)
    #fig, ax = r.world_map.plotMap(fig, ax)
    #plt.show()

    # for each step update the belief
    for s in range(steps):
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
        plt.show()
        # update the weights of the particles according to the measrement model
        estimator.measurementUpdate(measure)

        # update the particles according to resampling process
        estimator.particleUpdate()

        #fig, ax = plt.subplots()
        #fig, ax = plotParticles(estimator.p, fig, ax)
        #fig, ax = r.plotRobot(fig, ax)
        #fig, ax = r.world_map.plotMap(fig, ax)
        #plt.show()
        print('Step: ',  s)
        print('Robot location:', r)
        print("Mean error:",evaluation(r, estimator.p))


if __name__ == "__main__":
    fig, ax = plt.subplots()
    fig, ax = WORLD_MAP.plotMap(fig, ax)
    #seg = Segment(np.array([0, 0]), np.array([5, 5]))
    r = Robot(WORLD_MAP)
    r.setPose(.5, .5, .5)
    fig, ax = r.plotRay(fig, ax)
    #pts = WORLD_MAP.intersect(seg)
    #fig, ax = seg.plotSeg(fig, ax)
    #fig, ax = plotPts(pts, fig, ax)
    plt.show()
    #main()
