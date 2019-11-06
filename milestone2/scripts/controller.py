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


class Controller(object):
    def __init__(self, x, y, yaw, map, path):
        self.yaw = yaw
        self.pose = np.array([x, y])
        self.look = 0.4
        self.map = map
        self.path = path

        self.fig, self.ax = plt.subplots(2,2)
        plt.show(block=False)

    def move(self, v, gamma):
        dt = 0.05
        if np.abs(gamma) < 1e-6:
            w = 0
        else:
            r = 1/gamma
            if np.abs(v) < 1e-2:
                w = gamma
            else:
                w = v/r
        if np.abs(w) > 1e-6:
            # Compute the rotational radius
            r = v/w
            # Instantaneous center of curvature
            icc = [self.pose[0] - r*np.sin(self.yaw), self.pose[1] + r*np.cos(self.yaw)]
            wdt = w*dt
            self.pose[0] = (self.pose[0] - icc[0])*np.cos(wdt) - (self.pose[1] - icc[1])*np.sin(wdt) + icc[0]
            self.pose[1] = (self.pose[0] - icc[0])*np.sin(wdt) + (self.pose[1] - icc[1])*np.cos(wdt) + icc[1]
            self.yaw = self.yaw + wdt
        else:
            self.pose[0] += v*np.cos(self.yaw)*dt
            self.pose[1] += v*np.sin(self.yaw)*dt

    def closest(self):
        dist = np.linalg.norm(self.path - self.pose, axis=1)
        i = np.argmin(dist)
        return i

    def lookahead(self):
        left_path = self.path[self.closest():]

        d = np.linalg.norm(self.pose - self.path[-1])
        if d < self.look:
            return self.path[-1]

        for i, p in enumerate(reversed(left_path[:-1])):
            # i_ goes in the reverse order than i. i.e. if len(path) = 20 then
            # i = 0 and i_ = 18
            i_ = len(left_path) - 2 - i

            d = left_path[i_+1] - p
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

        kv = 1
        kd = 0
        kh = 5

        vm = 0.26
        wm = 0.20

        x = []
        y = []
        vp = []
        wp = []

        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        teta = math.atan2(dy, dx)
        a = (teta - self.yaw)
        a = ((a + np.pi) % (2*np.pi)) - np.pi
        gamma = kh*a

        dist = np.sqrt((pt[0] - self.pose[0])**2 + (pt[1] - self.pose[1])**2)
        v = kv*dist - kd*a
        v = max(min(v, vm), 0)
        self.move(v, gamma)
        x.append(self.pose[0])
        y.append(self.pose[1])
        vp.append(v)
        wp.append(gamma)
        return x, y, vp, wp

    def followPath(self):
        pt = self.lookahead()
        x = []
        y = []
        v = []
        w = []
        yaw = []
        i = 10

        d_ = np.linalg.norm(self.pose - self.path[-1])
        dist = []
        while d_ > 1e-1:
            a, b, c, d = self.mv2pt(pt)
            pt = self.lookahead()
            x += a
            y += b
            v += c
            w += d
            yaw.append(self.yaw)
            d_ = np.linalg.norm(self.pose - self.path[-1])
            dist.append(d_)
            if i % 10 == 0:
                self.plotTraj(x, y, yaw, v, w, dist, pt)
            i += 1
        return x, y, v, w, dist

    def plotTraj(self, x, y, yaw, v, w, d, goal):

        self.ax[0][0].clear()
        self.ax[1][0].clear()
        self.ax[1][1].clear()
        self.ax[0][1].clear()

        self.fig, self.ax[0][0] = self.map.plotMap(self.fig, self.ax[0][0])
        self.ax[0][0].plot(self.path[:,0], self.path[:,1], 'r')
        self.ax[0][0].plot(x, y)
        self.ax[0][0].scatter(goal[0], goal[1])
        self.ax[1][0].plot(range(len(v)), v)
        self.ax[1][1].plot(range(len(w)), w)
        self.ax[0][1].plot(range(len(yaw)), yaw)

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
    start = [2.0, 1.75]
    goal = [2.0, 2.6]
    #goal = [.5, .6]
    path = np.array(planner.getPath(start, goal))
    #path = np.array([[.5, .5], [.7, .5], [1., .5]])
    r = Controller(start[0], start[1], 0, map, path)
    #r.lookahead()
    x, y, v, w, d = r.followPath()

    start = [.4, 2.8]
    goal = [2.5, 0.5]
    path = np.array(planner.getPath(start, goal))
    r = Controller(start[0], start[1], 0, map, path)
    #r.lookahead()
    x, y, v, w, d = r.followPath()
    #planner.plotGraph(start=start, goal=goal, path=path)


if __name__ == "__main__":
    main2()
