import numpy as np

from rrt import RRT
from utile import Map
from utile import Segment

import matplotlib.pyplot as plt

class PurePursuit(object):
    def __init__(self, path, pose, look):
        self.path = path
        self.pose = pose[0:2]
        self.yaw = pose[2]
        self.look = look
        return

    def _path_curvature(self):
        '''
        computes the curvature of the path. Given by
        (x'*y'' - x''*y')/(x'**2 + y' ** 2 ) ** (3/2)
        '''
        path_p = self._path_1_derivative()
        path_pp = self._path_2_derivative()
        x_p = path_p[:,0]
        x_pp = path_pp[:,0]
        y_p = path_p[:,1]
        y_pp = path_pp[:,1]
        k = np.divide(np.dot(x_p, y_pp) - np.dot(x_pp, y_p), np.power((np.power(x_p, 2) + np.power(y_p, 2)), 3/2))

        r = 1 / k
        #plt.plot(np.arange(len(r)), r)
        #plt.show()

        return k

    def _path_speed(self):
        return

    def _path_1_derivative(self):
        s_t = np.copy(self.path)
        s_t1 = np.delete(s_t, 0, 0)
        s_t_1 = np.delete(s_t, -1, 0)

        s = s_t[-1]
        s_t1 = np.vstack((s_t1, s))

        s = s_t[0]
        s_t_1 = np.vstack((s, s_t_1))
        #assume unit step in t
        s_t_prime = .5* (s_t1 - s_t_1)

        return s_t_prime

    def _path_2_derivative(self):
        s_t = np.copy(self.path)
        s_t1 = np.delete(s_t, 0, 0)
        s_t_1 = np.delete(s_t, -1, 0)

        s = s_t[-1]
        s_t1 = np.vstack((s_t1, s))

        s = s_t[0]
        s_t_1 = np.vstack((s, s_t_1))

        s_t_prime = .5* (s_t1 - s_t_1)
        s_t_pp = s_t1 - 2*s_t + s_t_1

        #fig, ax = plt.subplots(2,2)
        #ax[0][0].plot(self.path[:,0], self.path[:,1], 'r')
        #ax[1][0].plot(np.arange(len(s_t[:,0])), s_t[:,0], 'y')
        #ax[1][0].plot(np.arange(len(s_t_prime[:,0])), s_t_prime[:,0], 'b')
        #ax[1][0].plot(np.arange(len(s_t_pp[:,0])), s_t_pp[:,0], 'r')

        #ax[1][1].plot(np.arange(len(s_t[:,1])), s_t[:,1], 'y')
        #ax[1][1].plot(np.arange(len(s_t_prime[:,1])), s_t_prime[:,1], 'b')
        #ax[1][1].plot(np.arange(len(s_t_pp[:,1])), s_t_pp[:,1], 'r')
        #plt.show()

        return s_t_pp

    def _desired_vel(self):
        k = self._path_curvature()
        max_vel = 0.26
        turn_cst = 0.05
        max_acc = 0.05

        vel = []
        for i in k:
            # i is the curvature at a given point on the curve
            vel.append(min(max_vel, turn_cst/i))

        plt.plot(np.arange(len(vel)), vel)
        plt.show()

        vel_acc = []
        for i, w in enumerate(reversed(vel[:-1]), start=1):
            w.append(min(w, math.sqrt(w[-i]**2 + 2 * max_acc * \
                            math.sqrt((self.path[i][0]-self.path[-i][0])**2 + (self.path[i][1]-self.path[-i][1])**2))))

        vel_acc[0] = 0

        for i, w in enumerate(vel[1:], start=1):
            test = math.sqrt(vel_acc[i-1]**2 + 2*max_acc* \
                         math.sqrt((self.path[i][0] - self.path[i-1][0]) ** 2 + (self.path[i][1] - self.path[i-1][1]) ** 2))
            print(test)
            if test < vel_acc[i]:
                vel_acc[i] = test
            else:
                break

        plt.plot(np.arange(len(vel_acc)), vel_acc)
        plt.show()

        return vel

    def control(self, pose):
        return

    def closest(self):
        dist = np.linalg.norm(self.path - self.pose)
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

    def curvature(self, lookahead_pt):
        dist = lookahead_pt - self.pose
        side = np.sign(np.cos(self.yaw)*(dist[0]) - np.sin(self.yaw)*(dist[1]))
        a = 1/np.tan(self.yaw)
        c = -1/np.tan(self.yaw)*self.pose[0] - self.pose[1]
        x = abs(a*lookahead_pt[0] + lookahead_pt[1] + c) / np.sqrt(a**2 + 1)
        return side * (2*x/(self.look)**2)

    def move(self, curv, vel, width):
        return vel*curv

def main():
    map = Map("maps/rss_offset.json")
    planner = RRT(map)
    start = [.5, .5]
    goal = [3.50, 2.5]
    path = np.array(planner.getPath(start, goal))


    start = [.4, .4, np.pi/20]
    pp = PurePursuit(path, np.array(start), 1)
    pp._desired_vel()
    l = pp.lookahead()
    c = pp.curvature(l)
    p2 = path[1]
    seg = Segment(start[0:2], p2)
    origin = np.copy(seg.perp)
    r = 1/c
    origin *= r


    fig, ax = plt.subplots()
    map.plotMap(fig, ax)
    ax.plot(path[:,0], path[:, 1], 'r', marker='.')
    ax.quiver(start[0], start[1], origin[0], origin[1])
    plt.show()

if __name__ == "__main__":
    main()
