#!/usr/bin/env python

import rospy
import rospkg

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import tf
import numpy as np
from particle import Map

import json

import matplotlib.pyplot as plt
from matplotlib import collections as mc

r_x = 0
r_y = 0
r_yaw = 0

e_x = 0
e_y = 0
e_yaw = 0

MAP_FILE = "/maps/rss_offset.json"

class FakeParticle(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

class ParticleSet(object):
    def __init__(self, file_path, steps, particles, map):
        self.map = map
        self.robot_x = []
        self.robot_y = []
        self.robot_yaw = []
        self.est_x = []
        self.est_y = []
        self.est_yaw = []
        with open(file_path) as file:
            data = json.load(file)
            for i in range(steps):
                self.p = []
                for j,p in enumerate(data[str(i)]):
                    vec = p[str(j)]
                    self.p.append(FakeParticle(vec[0], vec[1], vec[2]))
                def est(particles):
                    x = 0
                    y = 0
                    yaw = 0
                    print(len(particles))
                    for p in particles:
                        x += p.x
                        y += p.y
                        y += p.yaw
                    x_e = x/len(particles)
                    y_e = y/len(particles)
                    yaw_e = yaw/len(particles)
                    return x_e, y_e, yaw_e
                x_e, y_e, yaw_e = est(self.p)
                self.est_x.append(x_e)
                self.est_y.append(y_e)
                self.est_yaw.append(yaw_e)
                robot = data[str(i) + 'robot']
                self.robot_x.append(robot['x'])
                self.robot_y.append(robot['y'])
                self.robot_yaw.append(robot['yaw'])
                self.plotParticles()
        plt.plot(self.robot_x, self.robot_y, c='r')
        plt.plot(self.est_x, self.est_y, c='g')
        plt.show()

    def plotParticles(self):
        fig, ax = plt.subplots()
        fig, ax = self.map.plotMap(fig, ax)
        for p in self.p:
            ax.scatter(p.x, p.y, c='b')
            #ax.quiver(p.x, p.y, 1, 1, angles=np.rad2deg(p.yaw), scale=1/5, scale_units="dots",
            #units="dots", color="y", pivot="mid", width=1.25, headwidth=2, headlength=0.5)
        print(len(self.robot_x), len(self.robot_y))
        ax.plot(self.robot_x, self.robot_y, c='r')
        ax.plot(self.est_x, self.est_y, c='g')
        plt.show()

def poseCB(msg):
    global r_x, r_y, r_yaw, e_x, e_y, e_yaw
    e_x = msg.linear.x
    e_y = msg.linear.y
    e_yaw = msg.angular.z
    dx = e_x - r_x
    dy = e_y - r_y
    err = np.sqrt(dx**2 + dy**2)
    rospy.loginfo("x = {}, y = {}, yaw = {}".format(r_x, r_y, np.rad2deg(r_yaw)))
    rospy.loginfo("x_e = {}, y_e = {}, yaw_e = {}".format(e_x, e_y, np.rad2deg(e_yaw)))
    rospy.loginfo("Error = {}".format(err))

def modelCB(msg):
    global r_x, r_y, r_yaw
    j = 0
    for i, s in enumerate(msg.name):
        if s == "turtlebot3":
            j = i
    pose = msg.pose[j]
    r_x = pose.position.x
    r_y = pose.position.y
    orientation = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )
    r_yaw = tf.transformations.euler_from_quaternion(orientation)[2]
    return

def main():
    rospy.init_node("eval", anonymous=True)
    rospy.loginfo("INIT EVAL NODE")

    # subscribe
    rospy.Subscriber("pf_pose", Twist, poseCB)
    rospy.Subscriber("gazebo/model_states", ModelStates, modelCB)

    while not rospy.is_shutdown():
        rospy.sleep(10)

def main2():
    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    p = ParticleSet("test.json", 150, 50, map)

if __name__ == "__main__":
    main2()
