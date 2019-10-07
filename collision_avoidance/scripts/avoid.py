#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import concatenate
import numpy as np

lin = 0.5 #todo put in constants
turn = 0
turning = False

def left():
    global lin
    global turn

    rospy.loginfo("turning left")
    turn = 0.5
    lin = 0

def right():
    global lin
    global turn

    rospy.loginfo("turning right")
    turn = -0.5
    lin = 0

def forward():
    global lin
    global turn

    rospy.loginfo("going forward")
    turn = 0
    lin = 0.5

def scanCallback(msg):
   
    global turning

    front_laser_left = msg.ranges[0:20]
    front_laser_right = msg.ranges[340:359]
    front_laser=np.concatenate((front_laser_left,front_laser_right),axis=None)
    #rospy.loginfo(front_laser)
    left_laser = msg.ranges[89]
    right_laser = msg.ranges[269]

    rospy.loginfo("new data")

    # if we detect an object and not turning
    if not turning:
        for i in front_laser:
            if i > 0.12 and i < 0.4: 
                rospy.loginfo("starting to turn")
                #start turning
                turning = True
                # if left is closer to you (smaller sensor value)
                # go this direction until free
                if (left_laser>0.10) and left_laser > right_laser:
                    left()
                elif right_laser>0.10 and left_laser < right_laser:
                    right()
                else: # noisy sensor decision
                    left()
                
                break               

   
    # if we are already turning (higher threshold than initial turning)
    if turning and front_laser[0] > 0.5:
        rospy.loginfo("finishing turn")
        # we exceeded threshold of space in front, go forward
        forward()
        turning = False


def move_motor(pub):
    global lin
    global turn
    
    #rospy.loginfo("going forward again!")
    
    # create message object 
    mc = Twist()

    # set speed
    mc.linear.x = lin
    mc.angular.z = turn

    # publish message
    pub.publish(mc)

def moveloop(pub): 
    
    # set message rate 
    rate = rospy.Rate(20)

    # set initial direction
    forward()

    # loop updating 
    while not rospy.is_shutdown():
        move_motor(pub)
        rate.sleep()        

def scanListener():

    # initialise
    rospy.init_node("CollisionAvoidance", anonymous=True)

    # subscribe
    rospy.Subscriber("scan", LaserScan, scanCallback)

    # create publisher
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    moveloop(pub) #does not return


if __name__ == '__main__':
    scanListener()
