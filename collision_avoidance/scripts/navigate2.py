#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from numpy import concatenate
import numpy as np


class RobotController:

    MAX_LIN_SPEED=0.5
    MIN_LIN_SPEED=0.01
    MAX_TURN_SPEED=np.deg2rad(45)
    MIN_TURN_SPEED=0.01
    
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
        self.sub = rospy.Subscriber('tf', TFMessage, self.moveStop)
        self.mc = Twist()
        self.moving = False
        self.driving = False
        self.turning = False
        self.lin_target = None
        self.target_yaw = 0
        self.current_point = None
        self.yaw = 0
        self.rate = rospy.Rate(20)

    def moveStop(self, msg):
        # Get distance from origin point
        pose = msg.transforms[0].transform
        self.current_point = pose.translation

        orientation = (
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        )
        self.yaw = tf.transformations.euler_from_quaternion(orientation)[2]
        if self.yaw < 0.0:
            self.yaw += 2*np.pi
            
        if self.moving and self.turning:
            if np.rad2deg(np.abs(self.yaw - self.target_yaw)) < .01:
                self._stop()
                self.moving = False
                self.turning = False
                return
            if self.yaw > self.direction:
                self._command_motor(0, -self.getAngSpeed(2))
            else:
                self._command_motor(0, self.getAngSpeed(2))
                
        if self.moving and self.driving:
            if np.sqrt((self.lin_target[0] - self.current_point.x) ** 2 +
                       (self.lin_target[1] - self.current_point.y) ** 2) < .01:
                self._stop()
                self.moving = False
                self.driving = False
                return
            self._command_motor(self.getLinSpeed(2), 0)
            

    def getAngSpeed(self, alpha):
        return max(min(alpha*(np.abs(self.yaw - self.target_yaw)),
                        self.MAX_TURN_SPEED),
                    self.MIN_TURN_SPEED)

    def getLinSpeed(self, alpha):
        dist = np.sqrt((self.lin_target[0] - self.current_point.x) ** 2 +
                       (self.lin_target[1] - self.current_point.y) ** 2)
        return max(min(alpha*dist, self.MAX_LIN_SPEED),
                   self.MIN_LIN_SPEED)
 
    def turn(self, degrees):
        self.target_yaw = (degrees + self.yaw)
        # direction is used to choose turning direction, for the
        # particular situation where target_yaw > 360 and
        # where therfore the robot would be turning then entire
        # way round
        self.direction = self.target_yaw
        if self.target_yaw > 2*np.pi:
            self.target_yaw -= 2*np.pi
        rospy.loginfo("target : {}".format(self.target_yaw))
        rospy.loginfo("origin : {}".format(self.yaw))
        self.moving = True
        self.turning = True
        # making the function blocking until the end of the mvt
        while self.moving:
            self.rate.sleep()
        rospy.loginfo("finished turning")
            
    def move(self, meters):
        x = meters*np.cos(self.yaw) + self.current_point.x
        y = meters*np.sin(self.yaw) + self.current_point.y
        self.lin_target = [x, y]
        rospy.loginfo("target : {}, {}".format(x, y))
        rospy.loginfo("origin : {}, {}".format(self.current_point.x, self.current_point.y))
        self.moving = True
        self.driving = True
        # making the function blocking until the end of the mvt
        while self.moving:
            self.rate.sleep()
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
    rospy.sleep(1)
    navigate(rc)

if __name__ == '__main__':
    main()
