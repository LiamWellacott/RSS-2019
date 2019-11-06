#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

import time
import math

from utile import Map
from rrt import RRT


class robot(object):
    def __init__(self, x, y, yaw, map):
        self.yaw = yaw
        self.x = x
        self.y = y
        self.pose = np.array([x, y])
        self.look = 0.2
        self.map = map

        self.fig, self.ax = plt.subplots(2,2)
        plt.show(block=False)

    def move(self, v, gamma):
        dt = 0.05
        r = 1/gamma
        w = v/r
        if np.abs(w) > 1e-6:
            # Compute the rotational radius
            r = v/w
            # Instantaneous center of curvature
            icc = [self.x - r*np.sin(self.yaw), self.y + r*np.cos(self.yaw)]
            wdt = w*dt
            self.x = (self.x - icc[0])*np.cos(wdt) - (self.y - icc[1])*np.sin(wdt) + icc[0]
            self.y = (self.x - icc[0])*np.sin(wdt) + (self.y - icc[1])*np.cos(wdt) + icc[1]
            self.yaw = self.yaw + wdt
        else:
            self.x += v*np.cos(self.yaw)*dt
            self.y += v*np.sin(self.yaw)*dt
        self.pose = np.array([self.x, self.y])

    def closest(self):
        dist = np.linalg.norm(self.path - self.pose, axis=1)
        i = np.argmin(dist)
        return i

    def lookahead(self):
        for i, p in enumerate(reversed(self.path[:-1])):
            i_ = len(self.path) - 2 - i
            d = self.path[i_+1] - p
            f = p - self.pose

            a = np.dot(d, d)
            b = 2*np.dot(d, f)
            c = np.dot(f, f) - self.look*self.look

            delta = b**2 - 4*a*c
            if delta >= 0:
                delta = np.sqrt(delta)
                t1 = (-b + delta)/(2*a)
                t2 = (-b - delta)/(2*a)

                if 0 <= t1 <= 1:
                    return p + t1*d
                if 0 <= t2 <= 1:
                    return p + t2*d

        return self.path[self.closest()]

    def mv2pt(self, pt):
        kv = 0.5
        kh = 5
        vm = 0.26
        wm = 0.20
        x = []
        y = []
        vp = []
        wp = []
        dist = np.sqrt((pt[0] - self.x)**2 + (pt[1] - self.y)**2)
        v = kv*dist
        v = min(v, vm)
        dx = pt[0] - self.x
        dy = pt[1] - self.y
        teta = math.atan2(dy, dx)
        a = (teta - self.yaw)
        a = ((a + np.pi) % (2*np.pi)) - np.pi
        gamma = kh*a
        self.move(v, gamma)
        x.append(self.x)
        y.append(self.y)
        vp.append(v)
        wp.append(gamma)
        return x, y, vp, wp

    def followPath(self, path):
        self.path = path
        pt = self.lookahead()
        x = []
        y = []
        v = []
        w = []
        i = 10
        while self.closest() != len(self.path) - 1:
            a, b, c, d = self.mv2pt(pt)
            pt = self.lookahead()
            x += a
            y += b
            v += c
            w += d
            if i % 10 == 0:
                self.plotTraj(x, y, v, w, pt)
            i += 1
        return x, y, v, w

    def plotTraj(self, x, y, v, w, goal):

        self.ax[0][0].clear()
        self.ax[1][0].clear()
        self.ax[1][1].clear()

        self.fig, self.ax[0][0] = self.map.plotMap(self.fig, self.ax[0][0])
        self.ax[0][0].plot(self.path[:,0], self.path[:,1], 'r')
        self.ax[0][0].plot(x, y)
        self.ax[0][0].scatter(goal[0], goal[1])
        self.ax[1][0].plot(range(len(v)), v)
        self.ax[1][1].plot(range(len(w)), w)

        plt.draw()
        plt.pause(0.001)

def main():
    fig, ax = plt.subplots(2,2)
    #plt.show()
    goal = [5.0, 5.0]

    r = robot(5.0, 2.0, 0)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'r')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'r')
    ax[1][1].plot(range(len(w)), w, 'r')

    r = robot(7.0, 3.0, np.pi/4)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'g')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'g')
    ax[1][1].plot(range(len(w)), w, 'g')

    r = robot(8.0, 5.0, np.pi/2)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'r')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'c')
    ax[1][1].plot(range(len(w)), w, 'c')

    r = robot(7.0, 7.0, 3*np.pi/4)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'r')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'm')
    ax[1][1].plot(range(len(w)), w, 'm')

    r = robot(5.0, 8.0, np.pi)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'b')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'y')
    ax[1][1].plot(range(len(w)), w, 'y')

    r = robot(3.0, 7.0, -3*np.pi/4)
    x, y, v, w = r.mv2pt(goal)
    ax[0][0].plot(x, y, 'y')
    ax[0][0].scatter(goal[0], goal[1])
    ax[1][0].plot(range(len(v)), v, 'k')
    ax[1][1].plot(range(len(w)), w, 'k')

    ax[0][0].autoscale()
    plt.show()

def main2():
    map = Map("maps/rss_offset.json")
    planner = RRT(map)
    start = [.5, .5]
    goal = [3.5, 2.5]
    path = np.array(planner.getPath(start, goal))
    r = robot(start[0], start[1], 0, map)
    x, y, v, w = r.followPath(path)


if __name__ == "__main__":
    main2()
