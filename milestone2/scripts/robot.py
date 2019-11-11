#!/usr/bin/env python

# Ros packages
import rospy
import rospkg
import tf

import numpy as np
# To implement our task Queue
from Queue import Queue

# Misc classes
from utile import Map
from controller import Controller
from particle import ParticleFilter

# Message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Gazebo messages
#from gazebo_msgs.msg import ModelStates
# RRT path planing service messages
from milestone2.srv import RRTsrv, RRTsrvResponse
# Set goal for the robot
from milestone2.msg import Task


#import matplotlib
#matplotlib.use('Agg')
#import matplotlib.pyplot as plt

# initial position in the map as per the brief
INITIAL_X = 3.80
INITIAL_Y = 1.50
INITIAL_YAW = np.pi
#PATH = [[3.60, 1.5], [3.40, 1.5], [3.20, 1.5], [3.0, 1.5], [2.8, 1.5]]

# relative path from package directory
MAP_FILE = "/maps/rss_offset.json"

NUM_RAYS = 8
NUM_PARTICLES = 25

PUBLISH_RATE = 0.1

ODOM_RATE = 30

NOISE_MOVE = 0.05
NOISE_TURN = 0.05
NOISE_SENSE = 0.5

MAX_VAL = 150

class Robot(object):
    """
    Robot class used to represent particles and simulate the robot to
    test the particles.
    """
    def __init__(self, map, nb_p, x, y, yaw, nb_rays = 8):
        """
        Initializes a particle/robot.
        input:
        ------
            - map: a map object reference.
        """
        # initialise
        rospy.init_node("milestone2", anonymous=True)

        # Initialise particle filter
        self.nb_rays = nb_rays
        self.map = map
        self.particle_filter = ParticleFilter(map, nb_p, x, y, yaw, nb_rays)

        # subscribe
        rospy.Subscriber("scan", LaserScan, self.scanCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        #rospy.Subscriber("gazebo/model_states", ModelStates, self.gazeboCallback)
        # Allows to set a goal
        rospy.Subscriber("task", Task, self.setObjective)
        self.objectives = Queue()
        # Pose publisher, initialise message
        # TODO: UNCOMMENT THIS
        self.pose_msg = Twist()
        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw
        # timer for pose publisher
        # TODO: UNCOMMENT THIS
        self.pose_pub = rospy.Publisher('pf_pose', Twist, queue_size = 10)
        rospy.Timer(rospy.Duration(PUBLISH_RATE), self.pubPose)

        # Publisher for cmd vel
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        rospy.Timer(rospy.Duration(PUBLISH_RATE), self.pubVel)

        # set initial position
        self.x = x
        self.y = y
        self.yaw = yaw

        # Initialise controller
        self.controller = Controller()

        rospy.loginfo("Started robot node")
        while not rospy.is_shutdown():
            if self.objectives.empty():
                rospy.sleep(1)
            else:
                self.objectiveHandler(self.objectives.get())
        return

    def setObjective(self, msg):
        rospy.loginfo("received new task {}".format(msg.task))
        self.objectives.put(msg)

    def goTo(self, goal):
        rospy.loginfo("received new goal {}".format(goal))
        rospy.loginfo("Waiting for rrt service...")
        rospy.wait_for_service('rrt')
        try:
            rrt = rospy.ServiceProxy('rrt', RRTsrv)
            resp = rrt([self.x, self.y], goal)
            path = np.array(resp.path).reshape((-1,2))
            rospy.loginfo("Following new path...")
            #path = np.array(PATH)
            self.controller.setPath(path)

            #fig, ax = plt.subplots()
            #fig, ax = self.map.plotMap(fig, ax)
            #ax.plot(path[:, 0], path[:, 1], 'r')
            #plt.show()

            self.followPath()
            rospy.loginfo("Reached goal.")
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
            return

    def objectiveHandler(self, objective):
        task = objective.task
        if task == "goal":
            goal = objective.objective
            self.goTo(goal)
            return
        elif task == "pick":
            # TODO call handler
            rospy.loginfo("Pick up sample")
            return
        elif task == "smash":
            # TODO call handler
            rospy.loginfo("Smash button")
            return
        elif task == "move":
            # TODO call handler
            rospy.loginfo("move obstacle")
            return
        else:
            rospy.logwaring("Couldn't indentify objective {}".format(task))

    def followPath(self):
        while not self.controller.isDone(self.x, self.y):
            v, w = self.controller.getSpeed(self.x, self.y, self.yaw)
            self.vel_msg.linear.x = v
            self.vel_msg.angular.z = w
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0

    def gazeboCallback(self, msg):
        j = 0
        for i, s in enumerate(msg.name):
            if s == "turtlebot3":
                j = i
        pose = msg.pose[j]
        self.x = pose.position.x
        self.y = pose.position.y
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        self.yaw = tf.transformations.euler_from_quaternion(orientation)[2]

    def scanCallback(self, msg):

        # get the measurements for the specified number of points out of the scan information
        indexes = np.rint(np.linspace(0, 360 - 360/self.nb_rays, self.nb_rays)).astype(int)
        m = np.array(msg.ranges)
        measure = m[indexes]

        # update position estimation
        # TODO UNCOMMENT THIS
        self.poseEstimationUpdate(measure)
        return

    def odomCallback(self, msg):
        # add the received position increment to the particles
        vel = msg.twist.twist
        self.particle_filter.actionUpdate(vel.linear.x, vel.linear.y, vel.angular.z)
        return

    def poseEstimationUpdate(self, measurements):

        self.particle_filter.measurementUpdate(measurements)
        self.particle_filter.particleUpdate()
        x, y, yaw = self.particle_filter.estimate()

        self.x = x
        self.y = y
        self.yaw = yaw

        rospy.logdebug("x = {}, y = {}, yaw = {}".format(x, y, yaw))
        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw

        return

    def pubPose(self, event):
        self.pose_pub.publish(self.pose_msg)
        return

    def pubVel(self, event):
        self.vel_pub.publish(self.vel_msg)
        return

    def __str__(self):
        return "x {}, y {}, yaw {} ".format(self.x, self.y, self.yaw)

    def setPose(self, x, y, yaw):
        """
        Sets a new pose for the robot.
        input:
        ------
            - x: the new x position.
            - y: the new y position.
            - yaw: the new yaw angle.
        output:
        -------
            None
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        return

def main():

    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    r = Robot(map, NUM_PARTICLES, INITIAL_X, INITIAL_Y, INITIAL_YAW, NUM_RAYS)

if __name__ == "__main__":
    main()
