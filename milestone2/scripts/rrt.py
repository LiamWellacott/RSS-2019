#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

from utile import Map

RADIUS_OBSTACLE = 0.1
RADIUS_TARGET = .2
RRT_EXTEND_DIST = .20 # 20 CM between two points at most
SMOOTHING_ITERATIONS = 20
SMOOTHING_STEP = 0.1

np.random.seed(0)

class RRT(object):

    "Class generating rrt based trajectories"
    def __init__(self, map):
        self.map = map
        self.graph = {}
        self.pose = {}
        self.last_index = 0

    def getPath(self, q_init, goal):
        # TODO: should be extended to desired pose using some kind of
        # internal model to select approach direction
        # not sure how this is possible
        """Returns a smoothed path
        from current point to the goal
        input:
        ------
            q_init: current position
            goal: targeted position
        """
        #initialize the graph
        if not self.graph:
            self._updateGraph([], q_init)
        #if not self.goalInGraph(goal):
        self.extendGraph(goal)
        #path = self.astar(q_init, goal)

    def _updateGraph(self, entry, pose):
        self.last_index += 1
        self.graph.update({self.last_index : entry})
        self.pose.update({self.last_index : pose})

    def checkPointCollision(self, p):
        # TODO: not sure if the rrt class needs to have this method
        """ Checks if a point p is inside an obstacale
        input:
        ------
            point: p
        output:
        -------
            boolean: True if collision is detected, False otherwise
        """
        return self.map.inObstacle(p)

    def checkSegmentCollision(self, p, q):
        """ Detects if there is a collision between two points of the graph and
        the map
        input:
        ------
            p: one point
            q: second point
        output:
            boolean: True if collision is detected, False otherwise
        """
        return self.map.intersect([p, q])

    def samplePoint(self):
        """Samples a point inside the obstacle this is a guarentie
        of Map samplePoint method
        input:
        ------
            None
        output:
        -------
            A sampled point on the map"""
        return self.map.samplePoint()

    def findQnear(self, q_rand):
        """finds the closest point in the graph given a randomly sampled point
        input:
        ------
            q_rand: a randomly sampled point
        output:
        -------
            id: the id of the closest point in the graph. As the user might want
            to travel the graph from there
        """
        id = 1
        dist_min = float('inf')
        for key in self.graph:
            q = self.pose[key]
            dist = np.sqrt(np.power(q_rand[0] - q[0], 2) + np.power(q_rand[1] - q[1], 2))
            if dist < dist_min:
                dist_min = dist;
                id = key
        return id

    def findQnew(self, q_rand, q_near):
        """ Find the point which chould be linked to the newly found point on
        the map.
        input:
        ------
            q_near: the nearest point found.
        output:
        -------
            the node of the graph q_new should be attached to.
        """
        dist = np.sqrt(np.power(q_rand[0] - q_near[0], 2) + np.power(q_rand[1] - q_near[1], 2))
        if dist > RRT_EXTEND_DIST:
            q_normalized_x = q_near[0] + RRT_EXTEND_DIST*(q_rand[0]- q_near[0])/dist
            q_normalized_y = q_near[1] + RRT_EXTEND_DIST*(q_rand[1]- q_near[1])/dist
        else:
            q_normalized_x = q_rand[0]
            q_normalized_y = q_rand[1]
        q_new = (q_normalized_x, q_normalized_y)
        return q_new

    def extendGraph(self, goal):
        """ extends the graph towards a goal. Should be called only if a goal
        is set!
        input:
        ------
            None
        output:
        -------
            None
        """
        is_reached = False
        while not is_reached:

            # Sample a new point
            q_sample = self.samplePoint()

            # Find the index of the nearest node (q_near) in the graph
            id = self.findQnear(q_sample)
            print(self.graph)
            print(self.pose)
            q_near = self.pose[id]
            # Find the closest feasible point to the randomly sampled point (q_new)
            q_new = self.findQnew(q_sample, q_near)

            # Check if the edge is collision free
            if not self.checkSegmentCollision(q_near, q_new):
                self._updateGraph([id], q_new)
                self.graph[id].append(self.last_index)
                # Check if the goal has been reached
                if np.sqrt(np.power(q_new[0]-goal[0], 2) + np.power(q_new[1]-goal[1], 2)) < RADIUS_TARGET:
                    is_reached = True
        return

    def astar(self, start=None, goal=None):
        """ Astar implementation. This astar function uses the graph to
        find a path between init and goal. Need to make sure the goal is in
        the path.
        input:
        ------
            start: the starting point
            goal: the goal point
        """
        return

    def plotGraph(self, start=None, goal=None):
        fig, ax = plt.subplots()
        fig, ax = self.map.plotMap(fig, ax)

        if start is not None:
            #Draw start and target points
            circle_start_1 = plt.Circle(start, RADIUS_TARGET, color='g', alpha=0.5)
            circle_start_2 = plt.Circle(start, RADIUS_OBSTACLE, color='g')
            plt.gcf().gca().add_artist(circle_start_1)
            plt.gcf().gca().add_artist(circle_start_2)
        if goal is not None:
            circle_target_1 = plt.Circle(goal, RADIUS_TARGET, color='r', alpha=0.5)
            circle_target_2 = plt.Circle(goal, RADIUS_OBSTACLE, color='r')
            plt.gcf().gca().add_artist(circle_target_1)
            plt.gcf().gca().add_artist(circle_target_2)


        #Draw tree
        for key in self.graph:
            pose = self.pose[key]
            for edge in self.graph[key]:
                pose_edge = self.pose[edge]
                plt.plot([pose_edge[0], pose[0]], [pose_edge[1], pose[1]], color='y', marker='.')
        plt.axis('scaled')
        plt.grid()
        plt.show()


def main():
    map = Map("maps/rss_offset.json")
    planner = RRT(map)
    start = [.5, .5]
    goal = [3.50, 2.5]
    planner.getPath(start, goal)
    #sample = planner.samplePoint()
    #sample2 = planner.samplePoint()
    #planner.graph.update({1: [2]})
    #planner.graph.update({2: [1]})
    #planner.pose.update({1: sample})
    #planner.pose.update({2: sample2})
    planner.plotGraph()

if __name__ == '__main__':
    main()
