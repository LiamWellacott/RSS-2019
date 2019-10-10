#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from numpy import concatenate
import numpy as np
import matplotlib.pyplot as plt


class RobotController:

    MAX_LIN_SPEED=0.26
    MIN_LIN_SPEED=0.01

    MAX_TURN_SPEED=np.deg2rad(45)
    MIN_TURN_SPEED=0.01

    MAIN_RATE = 20

    ACCURACY = 0.01
    
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
        self.mc = Twist()

        self.tfsub = rospy.Subscriber('tf', TFMessage, self.processTF)

        self.current_point = None
        self.current_yaw = 0

        # rate used to check if termination condition
        self.rate = rospy.Rate(self.MAIN_RATE)

    def processTF(self, msg):
        # Get distance from origin point
        pose = msg.transforms[0].transform
        self.current_point = np.array([pose.translation.x, pose.translation.y])

        orientation = (
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        )
        self.current_yaw = tf.transformations.euler_from_quaternion(orientation)[2]

    '''
    adjust speed based on distance to the target yaw, constrained by a max and min speed
    '''
    def getAngSpeed(self, target_yaw, alpha):
        return max(min(alpha*(np.abs(target_yaw - self.current_yaw)),
                        self.MAX_TURN_SPEED),
                    self.MIN_TURN_SPEED)

    def getAngSpeed2(self, init_yaw, target_yaw, alpha):
        speed_goal = max(min(alpha*(np.abs(target_yaw - self.current_yaw)),
                             self.MAX_TURN_SPEED),
                         self.MIN_TURN_SPEED)
        speed_start = max(min(alpha*(np.abs(init_yaw - self.current_yaw)),
                             self.MAX_TURN_SPEED),
                         self.MIN_TURN_SPEED)
        return min(speed_goal, speed_start)

    '''
    adjust speed based on distance to the target location, constrained by a max and min speed
    '''
    def getLinSpeed(self, init_point, target_point, alpha):
        dist = np.linalg.norm(target_point - self.current_point) 
        return max(min(alpha*dist, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
    
    def getLinSpeed2(self, init_point, target_point, alpha):
        dist_goal = np.linalg.norm(target_point - self.current_point)
        dist_start = np.linalg.norm(init_point - self.current_point)
        
        speed_start = max(min(alpha*dist_start, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
        speed_goal = max(min(alpha*dist_goal, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
        return min(speed_start, speed_goal)

    def getLinSpeed2plot(self, init_point, target_point, current_point, alpha):
        dist_goal = np.linalg.norm(target_point - current_point)
        dist_start = np.linalg.norm(init_point - current_point)
        
        speed_start = max(min(alpha*dist_start, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
        speed_goal = max(min(alpha*dist_goal, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
        return min(speed_start, speed_goal)

    def plotSpeed(self, alpha):
        start = 0.0
        goal = 1.0
        to_start = np.arange(start, goal, 0.001)
        speed = np.zeros(to_start.shape)
        for i,_ in enumerate(to_start):
            speed[i] = self.getLinSpeed2plot(start, goal, to_start[i], alpha)
            
        plt.plot(to_start, speed)
        plt.show()
        
    def turn(self, degrees):

        # TODO reject which are not in range -pi to pi

        # calculate absolute target yaw in range -pi to pi
        target_yaw = self.current_yaw + degrees
        init_yaw = self.current_yaw
        if target_yaw > np.pi: # overflow pixs
            target_yaw -= 2*np.pi
        elif target_yaw < -np.pi: # underflow -pi
            target_yaw += 2*np.pi
        
        rospy.loginfo("target : {}".format(target_yaw))
        rospy.loginfo("origin : {}".format(self.current_yaw))

        # making the function blocking until the end of the mvt
        while np.abs(self.current_yaw - target_yaw) > np.deg2rad(self.ACCURACY):

            # adjust speed
            if degrees > 0: # if asked to turn right 
                self._command_motor(0, self.getAngSpeed2(init_yaw, target_yaw, 2))
            else:
                self._command_motor(0, -self.getAngSpeed2(init_yaw, target_yaw, 2))

            self.rate.sleep()

        rospy.loginfo("Finished turning")

        self._stop()
            
    '''
    moves the robot 'meters' in the current orientation
    '''
    def move(self, meters):

        # calculate target based on current position
        x = meters*np.cos(self.current_yaw) + self.current_point[0]
        y = meters*np.sin(self.current_yaw) + self.current_point[1]
        target_point = np.array([x, y])
        init_point = np.copy(self.current_point)

        rospy.loginfo("target : {}, {}".format(x, y))
        rospy.loginfo("origin : {}, {}".format(self.current_point[0], self.current_point[1]))
        
        # making the function blocking until the end of the mvt
        while np.linalg.norm(target_point - self.current_point) > self.ACCURACY and np.linalg.norm(init_point - self.current_point) < meters:
            #adjust speed
            self._command_motor(self.getLinSpeed2(init_point, target_point, 2), 0)

            self.rate.sleep()

        rospy.loginfo("Finished moving forward")

        self._stop()
        
        
    def _stop(self):
        self._command_motor(0,0)

    def _command_motor(self, lin, turn):
        # set speed
        self.mc.linear.x = lin
        self.mc.angular.z = turn

        # publish message
        self.pub.publish(self.mc)

def navigate(rc): 
    
    rc.move(.9)
    rc.turn(-np.deg2rad(93))
    rc.move(.9)
    rc.turn(np.deg2rad(93))
    rc.move(.9)      

def main():

    # initialise
    rospy.init_node("Navigate", anonymous=True)

    rc = RobotController()
    rospy.sleep(1) # delay to avoid loss of message
    navigate(rc)
    #rc.plotSpeed(1)

if __name__ == '__main__':
    main()
