#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import concatenate
import numpy as np

SPEED=0.26
ANGULAR_SPEED=0.5
OBS_ANGLE=45
SIDE_ANGLES=5
DETECT_RANGE=0.5
SELF_BODY=0.220

lin = SPEED #todo put in constants
turn = ANGULAR_SPEED
turning = False

def left():
    global lin
    global turn

    rospy.loginfo("turning left")
    turn = ANGULAR_SPEED
    lin = 0

def right():
    global lin
    global turn

    rospy.loginfo("turning right")
    turn = -ANGULAR_SPEED
    lin = 0

def forward():
    global lin
    global turn

    rospy.loginfo("going forward")
    turn = 0
    lin = SPEED

def scanCallback(msg):
    global turning
    front_laser_left = msg.ranges[:OBS_ANGLE]
    front_laser_right = msg.ranges[-OBS_ANGLE:]
    front_laser=np.concatenate((front_laser_left,front_laser_right),axis=None)
    left_laser = np.mean(msg.ranges[89-SIDE_ANGLES:89+SIDE_ANGLES])
    right_laser = np.mean(msg.ranges[269-SIDE_ANGLES:269+SIDE_ANGLES])

    # if we detect an object and not turning
    if not turning:
        for i in front_laser:
            if i > SELF_BODY and i < DETECT_RANGE:
                rospy.loginfo("starting to turn")
                #start turning
                turning = True
                # if left is closer to you (smaller sensor value)
                # go this direction until free
                if left_laser > right_laser:
                    left()
                elif left_laser < right_laser:
                    right()
                else: # noisy sensor decision
                    left()
                
                break               

   
    # if we are already turning (higher threshold than initial turning)
    if turning:
	safe_to_go_forward = True
	for i in front_laser:
		if i > SELF_BODY and i < DETECT_RANGE:
			safe_to_go_forward = False

	if safe_to_go_forward:
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
