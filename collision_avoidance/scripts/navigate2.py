#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from numpy import concatenate
import numpy as np


class RobotController:

    MAX_LIN_SPEED=0.26
    MIN_LIN_SPEED=0.01

    MAX_TURN_SPEED=np.deg2rad(45)
    MIN_TURN_SPEED=0.01

    ACCURACY = 0.01 # used for both
    
    RIGHT = -1
    LEFT = 1

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
        self.mc = Twist()

        self.tfsub = rospy.Subscriber('tf', TFMessage, self.processTF)

        self.is_turning = False
        self.is_moving = False

        self.init_point = None
        self.target_distance = 0.0
        self.current_point = None
        self.target_point = None

        self.direction = 0
        self.current_yaw = 0
        self.target_yaw = 0

    def processTF(self, msg):
        # Get the current distance from robots reference origin point
        pose = msg.transforms[0].transform
        self.current_point = np.array([pose.translation.x, pose.translation.y])

        # Convert representation of rotation from quaternion to yaw
        orientation = (
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        )
        self.current_yaw = tf.transformations.euler_from_quaternion(orientation)[2]

        if self.is_moving:

            if np.linalg.norm(self.target_point - self.current_point) > self.ACCURACY and \
            np.linalg.norm(self.init_point - self.current_point) > self.target_distance:
                
                #adjust speed
                self._command_motor(self.getLinSpeed(1), 0)
            
            else: # destination reached
                is_moving = False
                self._stop()


        if self.is_turning:

            if np.abs(self.current_yaw - self.target_yaw) > self.ACCURACY:

                # adjust speed
                self._command_motor(0, self.direction * self.getAngSpeed(2))

            else: # destination reached
                is_turning = False
                self._stop()

    '''
    adjust speed based on distance to the target yaw, constrained by a max and min speed
    '''
    def getAngSpeed(self, alpha):
        return max(min(alpha*(np.abs(self.target_yaw - self.current_yaw)),
                        self.MAX_TURN_SPEED),
                    self.MIN_TURN_SPEED)

    '''
    adjust speed based on distance to the target location, constrained by a max and min speed
    '''
    def getLinSpeed(self, alpha):
        dist = np.linalg.norm(self.target_point - self.current_point) 
        return max(min(alpha*dist, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
 
    '''
    sets the target yaw for the robot relative to current yaw by 'degrees' 
    '''
    def turn(self, degrees):

        # TODO reject which are not in range -pi to pi

        # calculate absolute target yaw in range -pi to pi
        self.target_yaw = self.current_yaw + degrees
        if self.target_yaw > np.pi: # overflow pi
            self.target_yaw -= 2*np.pi
        elif self.target_yaw < -np.pi: # underflow -pi
            self.target_yaw += 2*np.pi

        # set direction of movement
        if degrees > 0:
            self.direction = self.RIGHT
        else:
            self.direction = self.LEFT

        rospy.loginfo("target : {}".format(target_yaw))
        rospy.loginfo("origin : {}".format(self.current_yaw))

        # make the function blocking until the end of the mvt
        self.is_turning = True
        while self.is_turning:
            None

        rospy.loginfo("Finished turning")

                
    '''
    sets the target point for the robot 'meters' forward in the current orientation
    '''
    def move(self, meters):

        # TODO discard distance

        # calculate target based on current position
        x = meters*np.cos(self.current_yaw) + self.current_point[0]
        y = meters*np.sin(self.current_yaw) + self.current_point[1]
        self.target_point = np.array([x, y])
        self.init_point = np.copy(self.current_point)
        self.target_distance = meters

        rospy.loginfo("target : {}, {}".format(x, y))
        rospy.loginfo("origin : {}, {}".format(self.current_point[0], self.current_point[1]))
        
        # making the function blocking until the end of the mvt
        self.is_moving = True
        while self.is_moving:
            None

        rospy.loginfo("Finished moving forward")
        
        
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
