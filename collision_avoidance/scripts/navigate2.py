#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from numpy import concatenate
import numpy as np


class RobotController:

    MAX_LIN_SPEED=0.1
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
        self.rate = rospy.Rate(MAIN_RATE)

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
    def getAngSpeed(self, alpha):
        return max(min(alpha*(np.abs(self.current_yaw - self.target_yaw)),
                        self.MAX_TURN_SPEED),
                    self.MIN_TURN_SPEED)

    '''
    adjust speed based on distance to the target location, constrained by a max and min speed
    '''
    def getLinSpeed(self, target_point, alpha):
        dist = np.linalg.norm(target_point - self.current_point) 
        return max(min(alpha*dist, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
 
    def turn(self, degrees):

        # TODO reject which are not in range -pi to pi

        # calculate absolute target yaw in range -pi to pi
        target_yaw = self.current_yaw + degrees
        if target_yaw > np.pi: # overflow pi
            target_yaw -= 2*np.pi
        elif target_yaw < -np.pi: # underflow -pi
            target_yaw += 2*np.pi
        
        rospy.loginfo("target : {}".format(self.target_yaw))
        rospy.loginfo("origin : {}".format(self.yaw))

        # making the function blocking until the end of the mvt
        while np.abs(self.current_yaw - self.target_yaw) > np.deg2rad(self.ACCURACY):

            # adjust speed
            if degrees > 0: # if asked to turn right 
                self._command_motor(0, -self.getAngSpeed(2))
            else:
                self._command_motor(0, self.getAngSpeed(2))

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

        rospy.loginfo("target : {}, {}".format(x, y))
        rospy.loginfo("origin : {}, {}".format(self.current_point[0], self.current_point[1]))
        
        # making the function blocking until the end of the mvt
        while np.linalg.norm(target_point - self.current_point) > self.ACCURACY:

            #adjust speed
            self._command_motor(self.MAX_LIN_SPEED, 0)

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
    
    rc.move(1.0)
    rc.turn(np.deg2rad(90))
    rc.move(1.0)
    rc.turn(-np.deg2rad(90))
    rc.move(1.0)      

def main():

    # initialise
    rospy.init_node("Navigate", anonymous=True)

    rc = RobotController()
    rospy.sleep(1) # delay to avoid loss of message
    navigate(rc)

if __name__ == '__main__':
    main()
