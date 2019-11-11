#!/usr/bin/env python

import rospy
import rospy
import rospkg
import tf

import numpy as np
import json
#import matplotlib
#matplotlib.use('Agg')
#import matplotlib.pyplot as plt
#from matplotlib import collections as mc
from copy import copy as copy

from milestone2.srv import RRTsrv, RRTsrvResponse

import time

from utile import Map

from Queue import PriorityQueue

RADIUS_OBSTACLE = 0.22
RADIUS_TARGET = .08
PLOT_RADIUS = .05
RRT_EXTEND_DIST = .22 # 20 CM between two points at most
SMOOTHING_ITERATIONS = 200
SMOOTHING_STEP = 0.1

MAP_FILE = "/maps/rss_offset.json"

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
        rospy.init_node('rrt_path_planer')
        s = rospy.Service('rrt', RRTsrv, self.getPath)
        rospy.loginfo('started rrt service')
        rospy.spin()

    def getPath(self, req):
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
        rospy.loginfo("request path")
        q_init = req.init
        goal = req.goal
        rospy.loginfo("init {}, goal {}".format(q_init, goal))
        # if nothing in the graph yet, initialise
        #self.graph = {}
        #self.pose = {}
        if not self.graph:
            self._updateGraph([], q_init)

        # if the goal position is not in the graph, loop until route to goal exists
        if not self.goalInGraph(goal):
            self.extendGraph(goal)
        path = self.astar(q_init, goal)
        path = self.smoothingPath(path)
        # Reshape the path to publish on in a single array, this should be reshaped
        # when recieved
        rospy.loginfo("sending path of shape {}".format(path.shape))
        #self.plotGraph(start= q_init, goal=goal, path=path)
        path = path.reshape((-1,))
        return RRTsrvResponse(path)

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
        return self.map.intersect([p, q], offset=RADIUS_OBSTACLE) or self.map.intersectCircle(q, RADIUS_OBSTACLE)

    def samplePoint(self):
        """Samples a point inside the obstacle this is a guarentie
        of Map samplePoint method
        input:
        ------
            None
        output:
        -------
            A sampled point on the map"""
        return self.map.samplePoint(RADIUS_OBSTACLE)

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
            q_sample = self.samplePoint()

            # Find the index of the nearest node (q_near) in the graph
            id = self.findQnear(q_sample)
            q_near = self.pose[id]
            # Find the closest feasible point to the randomly sampled point (q_new)
            q_new = self.findQnew(q_sample, q_near)

            '''
            if i < 10:
                fig, ax = plt.subplots()
                fig, ax = self.map.plotMap(fig, ax)
                circle = plt.Circle((q_new[0], q_new[1]), RADIUS_OBSTACLE, color='r', alpha=0.1)
                plt.gcf().gca().add_artist(circle)
                self.plotGraph(start=q_near, goal=q_new, sample=q_sample)
                plt.show()
            i+=1
            '''
            # Check if the edge is collision free
            if not self.checkSegmentCollision(q_near, q_new):
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

        for i in range(SMOOTHING_ITERATIONS):
            index1 = np.random.randint(0, len(path)-1)
            index2 = np.random.randint(0, len(path)-1)
            if index1 != index2 and not self.checkSegmentCollision(path[index1], path[index2]):
                if index1 < index2:
                    index_low = index1
                    index_up = index2
                else:
                    index_low = index2
                    index_up = index1
                middle = []
                deltax = (path[index_up][0]-path[index_low][0])
                deltay = (path[index_up][1]-path[index_low][1])
                for l in np.arange(SMOOTHING_STEP, 1.0-SMOOTHING_STEP, SMOOTHING_STEP):
                    middle += [(path[index_low][0]+l*deltax, path[index_low][1]+l*deltay)]
                path = path[:index_low+1] + middle + path[index_up:]
        # Second smoothing, consist in having regularly spaced points and curving corners
        # TODO: define those variables as class constants.

        smooth_path = np.copy(path)

        path = np.array(path)
        new_path = np.copy(path[0])
        min_step = 0.10

        for i in range(len(path)-1):

            diff = path[i+1] - path[i]

            dist = np.linalg.norm(path[i+1]-path[i])

            j = min_step
            points = []
            while j < dist:
                new_point = path[i] + j/dist*diff
                points.append(new_point)
                j += min_step

            if points:
                points = np.array(points)
                new_path = np.vstack((new_path, points))

            new_path = np.vstack((new_path, path[i+1]))

        smooth_path = np.copy(new_path)


        weight_data = 0.5
        weight_smooth = 0.5
        tolerance = 0.5
        change = tolerance
        while change >= tolerance:
            change = 0
            for i in range(1, len(new_path)-1):
                for j in range(len(new_path[i])):
                    aux = smooth_path[i][j]
                    smooth_path[i][j] += weight_data * (new_path[i][j] - smooth_path[i][j]) + \
                        weight_smooth * (smooth_path[i-1][j] + smooth_path[i+1][j] - 2*smooth_path[i][j])

        return smooth_path

    def plotGraph(self, start=None, goal=None, sample=None, path=None):
        rospy.loginfo("start plotting")
        fig, ax = plt.subplots()
        rospy.loginfo("Plotting map")
        fig, ax = self.map.plotMap(fig, ax)
        rospy.loginfo("map plotted")

        rospy.loginfo("Plot start")
        if start is not None:
            #Draw start and target points
            circle_start_1 = plt.Circle(start, PLOT_RADIUS, color='g', alpha=0.5)
            plt.gcf().gca().add_artist(circle_start_1)

        rospy.loginfo("Start plotted")
        rospy.loginfo("Plot goal")

        if goal is not None:
            circle_target_1 = plt.Circle(goal, PLOT_RADIUS, color='b', alpha=0.5)
            plt.gcf().gca().add_artist(circle_target_1)

        rospy.loginfo("goal plotted")
        rospy.loginfo("Plot sample")

        if sample is not None:
            circle_target_1 = plt.Circle(sample, PLOT_RADIUS, color='r', alpha=0.5)
            plt.gcf().gca().add_artist(circle_target_1)

        rospy.loginfo("Sample plotted")
        rospy.loginfo("Plot tree")
        #Draw tree
        for key in self.graph:
            pose = self.pose[key]
            for edge in self.graph[key]:
                pose_edge = self.pose[edge]
                plt.plot([pose_edge[0], pose[0]], [pose_edge[1], pose[1]], color='y', marker='.')

        rospy.loginfo("Tree plotted")

        rospy.loginfo("Plot path")

        if path is not None:
            for node in path:
                circle_target_1 = plt.Circle(node, 0.01, color='r', alpha=0.5)
                plt.gcf().gca().add_artist(circle_target_1)
                #plt.scatter(node[0], node[1], color='r', marker='.')

        rospy.loginfo("Path plotted")

        plt.axis('scaled')
        plt.grid()
        plt.show()

    def l2heurisitc(self, node, goal):
        return np.sqrt(np.power(node[0] - goal[0], 2) + np.power(node[1] - goal[1], 2))

def performance():
    map = Map("maps/rss_offset.json")

    iter = 30
    rrt_only = np.zeros((iter,))
    rrt_astar = np.zeros((iter,))
    x = np.arange(iter)
    planner_astar = RRT(map)
    for i in x:
        # Sample points
        start = map.samplePoint(RADIUS_OBSTACLE)
        goal = map.samplePoint(RADIUS_OBSTACLE)

        start_rrt = time.time()
        planner = RRT(map)
        path = planner.getPath(start, goal)
        rrt_only[i] = time.time() - start_rrt

        start_astar = time.time()
        path = planner_astar.getPath(start, goal)
        rrt_astar[i] = time.time() - start_astar
        print(i)

    plt.plot(x, rrt_only, 'r')
    plt.plot(x, rrt_astar, 'b')
    plt.show()

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    planner = RRT(map)

if __name__ == '__main__':
    main()
