#!/usr/bin/env python

#import rospy
import numpy as np
import json
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from copy import copy as copy

from utile import Map

from Queue import PriorityQueue

RADIUS_TARGET = .08 # used to determine if a node is acceptably close to the target
RRT_EXTEND_DIST = .20 # 20 CM between two points at most

# TODO not currently used
SMOOTHING_ITERATIONS = 20
SMOOTHING_STEP = 0.1

# used for plotting
RADIUS_OBSTACLE = 0.05
PLOT_RADIUS = .05

# fixed for repeatability
np.random.seed(0)

class Node(object):
    def __init__(self, parent, position, id):
        self.parent = parent
        self.position = position
        self.id = id

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, node):
        return self.position == node.position

    def __lt__(self, node):
        return self.f < node.f

    def __le__(self, node):
        return self.f <= node.f

    def __ne__(self, node):
        return self.position != node.position

    def __ge__(self, node):
        return self.f >= node.f

    def __gt__(self, node):
        return self.f > node.f

    def cost(self, position):
        """Compute the cost to get from the node to
        a given position
        input:
        ------
            position: the position to compute the cost to
        output:
        -------
            the L2 norm cost from self to position
        """
        return np.sqrt(np.power(self.position[0] - position[0], 2) + \
               np.power(self.position[1] - position[1], 2))

class RRT(object):

    "Class generating rrt based trajectories"
    def __init__(self, map):
        self.map = map
        self.graph = {}
        self.pose = {}
        self.last_index = 0
        self.heurisitc = self.l2heurisitc

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
        # if nothing in the graph yet, initialise
        if not self.graph:
            self._updateGraph([], q_init)

        # if the goal position is not in the graph, loop until route to goal exists
        if not self.goalInGraph(goal):
            self.extendGraph(goal)

        # calculate and return path to the goal node
        path = self.astar(q_init, goal)
        return path

    def _updateGraph(self, entry, pose):
        self.last_index += 1
        self.graph.update({self.last_index : entry})
        self.pose.update({self.last_index : pose})

    def goalInGraph(self, goal):

        # for each node in the graph
        for key in self.graph:
            q = self.pose[key]

            # calculate distance between goal and node
            dist = np.sqrt(np.power(goal[0] - q[0], 2) + np.power(goal[1] - q[1], 2))
            if dist < RADIUS_TARGET: # less than threshold?
                return True
 
        # No node found within RADIUS_TARGET
        return False

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
                dist_min = dist
                id = key

        # return id of the closest node
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
        i = 0
        while not is_reached:

            # Sample a new point
            q_sample = self.map.samplePoint()

            # Find the index of the nearest node (q_near) in the graph
            id = self.findQnear(q_sample)
            q_near = self.pose[id]
            # Find the closest feasible point to the randomly sampled point (q_new)
            q_new = self.findQnew(q_sample, q_near)

            # Check if the edge is collision free
            if not self.map.intersect([q_near, q_new]):
                self._updateGraph([id], q_new)
                self.graph[id].append(self.last_index)
                # Check if the goal has been reached
                if np.sqrt(np.power(q_new[0]-goal[0], 2) + np.power(q_new[1]-goal[1], 2)) <= RADIUS_TARGET:
                    is_reached = True
        return

    def astar(self, start, goal):
        """ Astar implementation. This astar function uses the graph to
        find a path between init and goal. Need to make sure the goal is in
        the path.
        input:
        ------
            start: the starting point
            goal: the goal point
        """
        path_x = []
        path_y = []
        id_start = self.findQnear(start)
        id_goal = self.findQnear(goal)
        fringe = PriorityQueue()
        visited = {}
        #visited[id_start] = 0
        fringe.put(Node(None, self.pose[id_start], id_start))
        end_node = Node(None, self.pose[id_goal], id_goal)

        # keeps the current estimated cost of one node
        g_cost = {}
        g_cost[id_start] = 0

        while not fringe.empty():
            # Retrives the top element of the PriorityQueue. I.E
            # the node with the smallest cost.
            current = fringe.get()
            visited[current.id] = current

            # Found the goal
            if current == end_node:
                path = []
                prev = current
                while prev is not None:
                    path.append(prev.position)
                    prev = prev.parent
                return path[::-1]

            # List all the childrens
            for child_id in self.graph[current.id]:
                child = Node(current, self.pose[child_id], child_id)
                if child in visited:
                    continue

                #compute values for A*
                child.g = current.g + child.cost(current.position)
                child.h = self.heurisitc(child.position, end_node.position)
                child.f = child.g + child.h

                if child.id not in g_cost.keys() or child.g < g_cost[child.id]:
                    g_cost[child.id] = child.g
                    fringe.put(child)

    def smoothingPath(self, path):
        
        return new_path

    def plotGraph(self, start=None, goal=None, path=None):
        fig, ax = plt.subplots()
        fig, ax = self.map.plotMap(fig, ax)

        if start is not None:
            #Draw start and target points
            circle_start_1 = plt.Circle(start, PLOT_RADIUS, color='g', alpha=0.5)
            circle_start_2 = plt.Circle(start, RADIUS_OBSTACLE, color='g')
            plt.gcf().gca().add_artist(circle_start_1)
            plt.gcf().gca().add_artist(circle_start_2)
        if goal is not None:
            circle_target_1 = plt.Circle(goal, PLOT_RADIUS, color='b', alpha=0.5)
            circle_target_2 = plt.Circle(goal, RADIUS_OBSTACLE, color='b')
            plt.gcf().gca().add_artist(circle_target_1)
            plt.gcf().gca().add_artist(circle_target_2)

        #Draw tree
        for key in self.graph:
            pose = self.pose[key]
            for edge in self.graph[key]:
                pose_edge = self.pose[edge]
                plt.plot([pose_edge[0], pose[0]], [pose_edge[1], pose[1]], color='y', marker='.')

        if path is not None:
            for node in path:
                circle_target_1 = plt.Circle(node, PLOT_RADIUS, color='r', alpha=0.5)
                circle_target_2 = plt.Circle(node, RADIUS_OBSTACLE, color='r')
                plt.gcf().gca().add_artist(circle_target_1)
                plt.gcf().gca().add_artist(circle_target_2)
                #plt.scatter(node[0], node[1], color='r', marker='.')
        plt.axis('scaled')
        plt.grid()
        plt.show()

    def l2heurisitc(self, node, goal):
        return np.sqrt(np.power(node[0] - goal[0], 2) + np.power(node[1] - goal[1], 2))

def main():
    map = Map("maps/rss_offset.json")
    planner = RRT(map)
    start = [.5, .5]
    goal = [3.50, 2.5]
    path = planner.getPath(start, goal)
    planner.plotGraph(start=start, goal=goal, path=path)
    start = [.4, 3.0]
    goal = [2.5, 0.5]
    path = planner.getPath(start, goal)
    planner.plotGraph(start=start, goal=goal, path=path)
    #sample = planner.map.samplePoint()
    #sample2 = planner.map.samplePoint()
    #planner.graph.update({1: [2]})
    #planner.graph.update({2: [1]})
    #planner.pose.update({1: sample})
    #planner.pose.update({2: sample2})

if __name__ == '__main__':
    main()
