import numpy as np
import math

class Controller(object):
    def __init__(self):
        self.path = []
        self.pose = np.array([0, 0])
        self.yaw = 0
        self.look = 0.2

    def setPath(self, path):
        self.path = path
        self.first = True

    def isDone(self, x, y):
        d = np.sqrt((x - self.path[-1][0])**2 + (y - self.path[-1][1])**2)
        if d < 1e-1:
            return True
        return False

    def getSpeed(self, x, y, yaw):
        self.pose = np.array([x, y])
        self.yaw = yaw
        pt = self.lookahead()
        v, w = self.mv2pt(pt)
        return v, w

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
        kd = 0.1
        kh = 5

        vm = 0.05
        wm = 0.4

        dx = pt[0] - self.pose[0]
        dy = pt[1] - self.pose[1]
        teta = math.atan2(dy, dx)
        a = (teta - self.yaw)
        a = ((a + np.pi) % (2*np.pi)) - np.pi
        gamma = kh*a

        dist = np.sqrt((pt[0] - self.pose[0])**2 + (pt[1] - self.pose[1])**2)
        v = kv*dist - kd*a
        v = max(min(v, vm), 0)

        if np.abs(gamma) < 1e-6:
            w = 0
        else:
            r = 1/gamma
            if np.abs(v) < 1e-2:
                w = gamma
            else:
                w = v/r

        w = min(w, wm)

        if self.first and np.abs(a) > 5:
            v = 0
        elif self.first and np.abs(a) < 5:
            self.first = False
        return v, w

    def isDoneAngle(self, pt_target, pose, yaw):
        dx = pt_target[0] - pose[0]
        dy = pt_target[1] - pose[1]
        teta = math.atan2(dy, dx)
        a = (teta - yaw)
        a = ((a + np.pi) % (2*np.pi)) - np.pi
        if np.abs(a) < np.abs(np.deg2rad(5)):
            return True
        return False

    def align(self, pt_target, pose, yaw):
        kh = 1
        wm = 0.80

        dx = pt_target[0] - pose[0]
        dy = pt_target[1] - pose[1]
        teta = math.atan2(dy, dx)
        a = (teta - yaw)
        a = ((a + np.pi) % (2*np.pi)) - np.pi
        w = kh*a
        w = min(w, wm)

        return w
