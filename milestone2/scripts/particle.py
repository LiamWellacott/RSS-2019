#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

def gaussian(mu, sigma, x):
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

class Segment(object):
    '''
    Segment constructor.
    args:
    -----
        - first point p1 as array [x1, y1]
        - second point p2 as array [x2, y2]
    '''
    def __init__(self, p1, p2):
        # p1 is always the point with the smallest x
        if p1[0] < p2[0]:
            self.p1 = p1
            self.p2 = p2
        else:
            self.p1 = p2
            self.p2 = p1
        self.vert = False
        if p1[0] - p2[0] == 0:
            self.vert = True
    '''
    Computes the intersection point of the segment with a line.
    The line is a array [m, b] where m is the slope and b is the
    intersect.
    args:
    -----
        - segment
    return:
    -------
        - point [x, y] of the intersection.
    '''
    def intersect(self, seg):
        # not a vertical line
        if self.vert and seg.vert:
            return np.full(2, np.nan)

        if not self.vert and not seg.vert:
            m1 = (self.p1[1] - self.p2[1]) / (self.p1[0] - self.p2[0])
            b1 = self.p1[1] - m1*self.p1[0]

            m2 = (seg.p1[1] - seg.p2[1]) / (seg.p1[0] - seg.p2[0])
            b2 = seg.p1[1] - m2*seg.p1[0]

            # lines are paralle
            if (m1 - m2) == 0:
                return np.full(2, np.nan)
            x = (b2 - b1) / (m1 - m2)
            y = m1*x + b1


        if self.vert:
            m2 = (seg.p1[1] - seg.p2[1]) / (seg.p1[0] - seg.p2[0])
            b2 = seg.p1[1] - m2 * seg.p1[0]
            y = m2 * self.p1[0] + b2
            x = self.p1[0]

        if seg.vert:
            m1 = (self.p1[1] - self.p2[1]) / (self.p1[0] - self.p2[0])
            b1 = self.p1[1] - m1 * self.p1[0]
            y = m1 * seg.p1[0] + b1
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
            return np.full(2, np.nan)

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

class Map(object):
    def __init__(self, map_file):
        self.segments = []
        with open(map_file) as json_file:
            data = json.load(json_file)
            for seg in data['segments']:
                p1 = np.array(seg[0])
                p2 = np.array(seg[1])
                s = Segment(p1, p2)
                self.segments.append(s)
    '''
       fct evaluated by print.
    '''
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
        lc = mc.LineCollection(lines, linewidth=2)
        ax.add_collection(lc)
        ax.autoscale()
        ax.margins(0.1)
        return fig, ax

    def intersect(self, segment):
        points = []
        for seg in self.segments:
            p = seg.intersect(segment)
            if not np.isnan(p).any():
                points.append(p)
        return np.array(points)


class Robot:
    def __init__(self):
        global WORLD_MAP
        self.x = np.random.rand() * 4.25
        self.y = np.random.rand() * 3.20
        self.yaw = np.random.uniform(-1, 1) * np.pi
        self.move_noise = 0
        self.turn_noise = 0
        self.sense_noise = 0
        self.nb_ray = 8
        self.ray_length = 5.32 # sqrt(4.25**2 + 3.20**2)
        self.world_map = WORLD_MAP
        return

    def __str__(self):
        return "x {}, y {}, yaw {}\n".format(self.x, self.y, self.yaw)

    def setPose(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        return

    def setNoise(self, move, turn, sense):

        self.move_noise = move
        self.turn_noise = turn
        self.sense_noise = sense
        return

    def sense(self):
        lines = []
        points = []
        distances = []
        for angle in np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment([self.x, self.y], [x, y])
            pts = self.world_map.intersect(seg)
            if not pts.size == 0:
                dist = np.linalg.norm(pts - np.array([self.x, self.y]), axis=1)
                pt = pts[np.argmin(dist)]
                dist = np.amin(dist)
                distances.append(dist + np.random.rand()*self.sense_noise)
                points.append(pt)
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
        ax.plot(self.x, self.y, marker=(3, 0, self.yaw), markersize=20, linestyle='None')
        for point in points:
            ax.plot(point[0], point[1], 'ro')
        return fig, ax

    def plotRobot(self, fig, ax):
        ax.scatter(self.x, self.y, c='r', s=500)
        ax.quiver(self.x, self.y, 5,5, angles=np.rad2deg(self.yaw), scale=1/5, scale_units="dots", units="dots", color="k", pivot="mid",width=2.5, headwidth=5, headlength=2.5)
        return fig, ax

    def measureProb(self, m):
        distances = []
        prob = 1.0
        for i, angle in enumerate(np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray)):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment([self.x, self.y], [x, y])
            pts = self.world_map.intersect(seg)
            if not pts.size == 0:
                dist = np.linalg.norm(pts - np.array([self.x, self.y]), axis=1)
                dist = np.amin(dist)
                prob *= gaussian(dist, self.sense_noise, m[i])
            else:
                prob *= 0
        return prob

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cant move backwards')
        
        # turn, and add Gaussian noise to the turning command
        orientation = self.yaw + float(turn) + np.random.randn()*self.turn_noise
        orientation %= 2 * np.pi # make sure: 0=< orientation <=2*pi
    
        # move, and add Gaussian noise to the motion command
        dist = float(forward) + np.random.randn()*self.move_noise
        x = self.x + (np.cos(orientation) * dist)
        y = self.y + (np.sin(orientation) * dist)
        x %= 4.25    # make sure: 0=< position <= world_size
        y %= 3.20
        
        # set the new location x, y back to the member variables x y of the class
        self.setPose(x, y, orientation)
        return None

WORLD_MAP = Map("maps/rss.json")

def plotParticles(particles, fig, ax):
    # Plotting the particles
    for p in particles:
        plt.scatter(p.x, p.y, c='b')
        plt.quiver(p.x, p.y, 1,1, angles=np.rad2deg(p.yaw), scale=1/5, scale_units="dots",
                   units="dots", color="y", pivot="mid", width=1.25, headwidth=2, headlength=0.5)
    return fig, ax

class ParticuleFilter(object):
    def __init__(self, nb_p):
        self.p = []
        self.w = []

        for _ in range(nb_p):
            r = Robot()
            # TODO estimate the std for the different operations
            r.setNoise(0.1, 0.1, 0.5)
            self.p.append(r)

    def actionUpdate(self, action):
        for p in self.p:
            p.move(action[0], action[1])

    def measurementUpdate(self, mt):
        w = []
        for p in self.p:
            w.append(p.measureProb(mt))

        self.w = w/np.sum(w)

    def particleUpdate(self):
        self.p = resampling(self.p, self.w)

def resampling(p, w):
    N = len(p)
    beta=0
    j=0
    w_max= max(w)
    p_temp=[]
    for i in range(N):
        beta= beta+2.0*w_max*np.random.rand()
        while beta>w[j]:
            beta = beta - w[j]
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

def main():
    r = Robot()
    r.setPose(.5, .5, 0)
    r.setNoise(0.0, 0.0, 0.0)

    steps = 100
    heading = 10.0/180.0*np.pi
    steplength = .1
    
    # Initialize the state estimator
    estimator = ParticuleFilter(50)
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
    #fig, ax = plt.subplots()
    #fig, ax = WORLD_MAP.plotMap(fig, ax)
    #plt.show()
    main()
