#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

lin = 1
turn = 0

def scanCallback(msg):
    global lin
    global turn

    if msg.ranges[0] > 0.12 and msg.ranges[0] < 0.5:
    rospy.loginfo(msg.ranges[0])
        turn = 1
        lin = 0
    else:
        turn = 0
        lin = 1

def move_motor(pub):
    global lin
    global turn

    mc = Twist()
    mc.linear.x = lin
    mc.angular.z = turn
    pub.publish(mc)

def moveloop(pub):  
    rate = rospy.Rate(20)

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
