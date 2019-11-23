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
from ik_v02 import Arm

# Message types
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# Gazebo messages
# from gazebo_msgs.msg import ModelStates
# RRT path planing service messages
from milestone2.srv import RRTsrv, RRTsrvResponse
# Set goal for the robot
from milestone2.msg import Task

import json

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

ODOM_HZ = 30.0
CUTOFF_ODOM = .05

SCAN_HZ = 5.0
CUTOFF = 1.0

POSE_HZ = 5.0
POSE_CUTOFF = 4.0

FILT_SAMPLES = 10

NOISE_MOVE_X = 0.05
NOISE_MOVE_Y = 0.01
NOISE_TURN = np.deg2rad(10.)
NOISE_SENSE = 0.05


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
        self.particle_filter = ParticleFilter(map, nb_p, x, y, yaw, nb_rays, NOISE_MOVE_X, NOISE_MOVE_Y, NOISE_TURN, NOISE_SENSE)

        # filtering values
        self.scan_len = 0
        self.odom_len = 0
        self.pose_len = 0


        # subscribe
        rospy.Subscriber("scan", LaserScan, self.scanCallback)
        rospy.Subscriber("odom", Odometry, self.odomCallback)
        # rospy.Subscriber("gazebo/model_states", ModelStates, self.gazeboCallback)
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

        # Initialise arm
        self.arm = Arm()

        # Logging info
        '''
        self.i = 0
        self.log_dict = {}
        self.o = 0
        self.odom_log = {}
        self.MAX_LOG = 500
        self.x_true = 0
        self.y_true = 0
        self.yaw_true = 0
        '''

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
        # Sample[0] and sample[1] should be the target's coordinates in weedle's frame
        arm.startSequence(arm.pickup(sample[0] , sample[1]))
        return

    def smashButton(self, button):
        # Button[0] and Button[1] should be the target's coordinates in weedle's frame
        arm.startSequence(arm.push_button(button[0] , button[1]))
        return

    def moveObst(self, obstacle):
        # Objective[0] and Objective[1] should be the target's coordinates in weedle's frame
        arm.startSequence(arm.move_obstacle(obstacle[0] , obstacle[1]))
        return

    def _worldToRobotFrame(self, coordinates):
        xdiff = coordinates[0] - self.x
        ydiff = coordaintes[1] - self.y

        xrobot = xdiff*np.cos(self.yaw) + ydiff*np.sin(self.yaw)
        yrobot = ydiff*np.cos(self.yaw) - xdiff*np.sin(self.yaw)
        return [xrobot, yrobot]

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
            samplerobot = self._worldToRobotFrame(sample)
            self.pickUp(samplerobot)
            rospy.loginfo("Sample collected")
            return
        elif task == "smash":
            rospy.loginfo("Smash button...")
            button = objective.objective
            buttonrobot = self._worldToRobotFrame(button)
            self.pickUp(buttonrobot)
            self.smashButton(buttonrobot)
            rospy.loginfo("Button smashed")
            returnParticleFilter
        elif task == "move":
            rospy.loginfo("move obstacle...")
            obstacle = objective.objective
            obstaclerobot = self._worldToRobotFrame(obstacle)
            self.pickUp(obstaclerobot)
            self.moveObst(obstaclerobot)
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
        self.x_true = pose.position.x
        self.y_true = pose.position.y
        orientation = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        self.yaw_true = tf.transformations.euler_from_quaternion(orientation)[2]

        vel = msg.twist[j]

        self.true_x_vel = vel.linear.x
        self.true_y_vel = vel.linear.y
        self.true_yaw_vel = vel.angular.z

    def scanCallback(self, msg):
        self.collision_avoidance.scanCallback(msg)
        # get the measurements for the specified number of points out of the scan information
        indexes = np.rint(np.linspace(0, 360 - 360/self.nb_rays, self.nb_rays)).astype(int)
        m = np.array(msg.ranges)
        measure = m[indexes]

        #self.poseEstimationUpdate(measure)


        if self.scan_len == 0:
            self.scan_buff = np.copy(measure.reshape(-1,1))
            self.scan_len += 1

        elif self.scan_len < FILT_SAMPLES:
            self.scan_buff = np.hstack((self.scan_buff, measure.reshape(-1,1)))
            self.scan_len += 1

        if self.scan_len >= FILT_SAMPLES:
            self.scan_buff = np.hstack((self.scan_buff, measure.reshape(-1,1)))
            self.scan_buff = self.scan_buff[:,1:]


            for i, ray in enumerate(self.scan_buff):
                measure[i] = self.filter(ray, SCAN_HZ, CUTOFF)
            # update position estimation
            self.poseEstimationUpdate(measure)

        return

    def odomCallback(self, msg):

        vel = msg.twist.twist
        vel = np.array([vel.linear.x, vel.angular.z])

        vel_before = np.copy(vel)
        if self.odom_len == 0:
            self.odom_buff = np.copy(vel.reshape(-1,1))
            self.odom_len += 1

        elif self.odom_len < FILT_SAMPLES:
            self.odom_buff = np.hstack((self.odom_buff, vel.reshape(-1,1)))
            self.odom_len += 1

        if self.odom_len >= FILT_SAMPLES:
            self.odom_buff = np.hstack((self.odom_buff, vel.reshape(-1,1)))
            self.odom_buff = self.odom_buff[:,1:]


            for i, v in enumerate(self.odom_buff):
                vel[i] = self.filter(v, CUTOFF_ODOM, ODOM_HZ)
            # add the received position increment to the particles

            self.particle_filter.actionUpdate(vel[0], 0, vel[1])
        #self.logInfoOdom(vel_before.tolist(), vel.tolist())

        return

    def imuCallback(self, msg):
        return


    def poseEstimationUpdate(self, measurements):
        # Update current weights
        rays, w = self.particle_filter.measurementUpdate(measurements)
        i = np.argmax(w)
        ray = rays[i]
        # Get Position according to the current max weight.
        x, y, yaw = self.particle_filter.estimate()
        state_b = np.array([x, y, yaw])
        state_a = np.array([x, y, yaw])

        # Low pass filter on position
        if self.pose_len == 0:
            self.pose_buff = np.copy(state_b.reshape(-1,1))
            self.pose_len += 1

        elif self.pose_len < FILT_SAMPLES:
            self.pose_buff = np.hstack((self.pose_buff, state_b.reshape(-1,1)))
            self.pose_len += 1

        if self.pose_len >= FILT_SAMPLES:
            self.pose_buff = np.hstack((self.pose_buff, state_b.reshape(-1,1)))
            self.pose_buff = self.pose_buff[:,1:]


            for i, s in enumerate(self.pose_buff):
                state_a[i] = self.filter(s, POSE_HZ, POSE_CUTOFF)


            self.x = state_a[0]
            self.y = state_a[1]
            self.yaw = state_a[2]

        #self.logInfo(measurements, ray)
        # Resample particles
        self.particle_filter.particleUpdate()

        rospy.logdebug("x = {}, y = {}, yaw = {}".format(x, y, yaw))
        self.pose_msg.linear.x = x
        self.pose_msg.linear.y = y
        self.pose_msg.angular.z = yaw


        return

    def pubPose(self, event):
        self.pose_pub.publish(self.pose_msg)
        return

    def pubVel(self, event):
        #if not self.collision_avoidance.isOK():
        #    self.vel_msg.linear.x = 0
        #    self.vel_msg.angular.z = self.collision_avoidance.turn()
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
        alpha = 1. / (1. + 1. / (2.*np.pi*hz*fc))
        y = np.zeros(len(x))
        y[0] = x[0]
        for i in range(1, len(x)):
            y[i] = alpha*x[i] + (1 - alpha)*y[i-1]
        return y[-1]

    def logInfo(self, m, rays):
        p = self.particle_filter.getPositions()
        dict = {"{}rPose".format(self.i): [self.x, self.y, self.yaw],
                "{}tPose".format(self.i): [self.x_true, self.y_true, self.yaw_true],
                "{}pPose".format(self.i): p,
                "{}rRay".format(self.i): rays,
                "{}tScan".format(self.i): m.tolist()}

        self.log_dict.update(dict)
        if self.i < self.MAX_LOG:
            rospy.loginfo("{}/{}".format(self.i, self.MAX_LOG))
        if self.i == self.MAX_LOG:
            rospack = rospkg.RosPack()
            path = rospack.get_path('milestone2')
            file = path + "/loginfo/log_{}_{}_{}_{}.json".format(NOISE_MOVE_X, NOISE_MOVE_Y, np.rad2deg(NOISE_TURN), NOISE_SENSE)
            with open(file, "w") as out_file:
                json.dump(self.log_dict, out_file, indent=4)

            rospy.loginfo("Data dumped")
        self.i += 1

    def logInfoOdom(self, vel1, vel2):
        dict = {"{}vel1".format(self.o): vel1,
                "{}vel2".format(self.o): vel2}
        self.odom_log.update(dict)
        if self.o < self.MAX_LOG:
            rospy.loginfo("{}/{}".format(self.o, self.MAX_LOG))
        if self.o == self.MAX_LOG:
            rospack = rospkg.RosPack()
            path = rospack.get_path('milestone2')
            file = path + "/loginfo/log_odom.json"
            with open(file, "w") as out_file:
                json.dump(self.odom_log, out_file, indent=4)

            rospy.loginfo("Odom Data dumped")

        self.o += 1
def main():

    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    map = Map(path + MAP_FILE)
    r = Robot(map, NUM_PARTICLES, INITIAL_X, INITIAL_Y, INITIAL_YAW, NUM_RAYS)

if __name__ == "__main__":
    main()
