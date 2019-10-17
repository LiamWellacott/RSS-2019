#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc


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
            #print("x={:.2f}, y={:.2f} intersection of y={:.2f}*x + {:.2f} and y = {:.2f}*x + {:.2f}".format(x, y, m1, b1, m2, b2))
        
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

        #print("seg1= {} \nseg2 = {}".format(self, seg))
        #input()
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
        return points
                
            

        
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
        for angle in np.linspace(self.yaw, self.yaw + 2*np.pi - 2*np.pi/self.nb_ray, self.nb_ray):
            x = np.cos(angle)*self.ray_length + self.x
            y = np.sin(angle)*self.ray_length + self.y
            seg = Segment([self.x, self.y], [x, y])
            pts = self.world_map.intersect(seg)
            for pt in pts:
                points.append(pt)
            lines.append([(self.x, self.y), (x, y)])
        return lines, points

    def plotRay(self, fig, ax):
        lines, points = self.sense()
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
    
    def measureProb(self):
        return

    def move(self):
        return

WORLD_MAP = Map("maps/data.json")
    
if __name__ == "__main__":
    global WORLD_MAP
    r = Robot()
    r.setPose(.5, .5, 0)
    fig, ax = plt.subplots()
    fig, ax = WORLD_MAP.plotMap(fig, ax)
    seg = Segment([0.5, 0.5], [4.26180808, -3.26180808])
    WORLD_MAP.intersect(seg)
    #fig, ax = seg.plotSeg(fig, ax)
    fig, ax = r.plotRay(fig, ax)
    plt.show()
