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
from avoid import Avoid

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
MAP_FILE = "/maps/rss_offset_box1.json"

NUM_RAYS = 8
NUM_PARTICLES = 25

PUBLISH_RATE = 0.1

MAX_VAL = 150

SCAN_HZ = 5.0
CUTOFF = 1.0

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

        self.scanLen = 0

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
        self.collision_avoidance = Avoid()

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

    def pickUp(self, sample):
        # TODO implement handler
        return

    def smashButton(self, button):
        # TODO implement handler
        return

    def moveObst(self, obstacle):
        # TODO implement handler
        return

    def objectiveHandler(self, objective):
        task = objective.task
        if task == "goal":
            goal = objective.objective
            rospy.loginfo("Moving to objective...")
            self.goTo(goal)
            rospy.loginfo("Reached objective")
            return
        elif task == "pick":
            rospy.loginfo("Pick up sample...")
            sample = objective.objective
            # TODO implement handler
            self.pickUp(sample)
            rospy.loginfo("Sample collected")
            return
        elif task == "smash":
            rospy.loginfo("Smash button...")
            button = objective.objective
            # TODO implement handler
            self.smashButton(button)
            rospy.loginfo("Button smashed")
            return
        elif task == "move":
            rospy.loginfo("move obstacle...")
            obstacle = objective.objective
            # TODO implement handler
            self.moveObst(obstacle)
            rospy.loginfo("Obstacle moved")
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
        self.collision_avoidance.scanCallback(msg)
        # get the measurements for the specified number of points out of the scan information
        indexes = np.rint(np.linspace(0, 360 - 360/self.nb_rays, self.nb_rays)).astype(int)
        m = np.array(msg.ranges)
        measure = m[indexes]

        self.poseEstimationUpdate(measure)

        '''
        if self.scanLen == 0:
            self.scanBuff = np.copy(measure.reshape(-1,1))
            self.scanLen += 1

        elif self.scanLen < 5:
            self.scanBuff = np.hstack((self.scanBuff, measure.reshape(-1,1)))
            self.scanLen += 1

        if self.scanLen >= 5:
            self.scanBuff = np.hstack((self.scanBuff, measure.reshape(-1,1)))
            self.scanBuff = self.scanBuff[:,1:]


            for i, ray in enumerate(self.scanBuff):
                measure[i] = self.filter(ray, SCAN_HZ, CUTOFF)
            # update position estimation
            #self.poseEstimationUpdate(measure)
        '''
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
        if not self.collision_avoidance.isOK():
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = self.collision_avoidance.turn()
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

    def filter(self, x, hz, fc):
        alpha = 1. / (1. + 1. / 2.*np.pi*hz*fc)
        y = np.zeros(len(x))
        y[0] = x[0]
        for i in range(1, len(x)):
            y[i] = alpha*x[i] + (1 - alpha)*y[i-1]
        return y[-1]

def main():

    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    r = Robot(map, NUM_PARTICLES, INITIAL_X, INITIAL_Y, INITIAL_YAW, NUM_RAYS)

if __name__ == "__main__":
    main()
