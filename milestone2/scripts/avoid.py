#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import concatenate
import numpy as np

SPEED=0.26
ANGULAR_SPEED=0.5
OBS_ANGLE=45
DETECT_RANGE=0.35
SELF_BODY=0.12


class Avoid(object):
    def __init__(self):
        self.turning = False
        self.ang_vel = 0

    def scanCallback(self, msg):
        front_laser_left = msg.ranges[:OBS_ANGLE]
        front_laser_right = msg.ranges[-OBS_ANGLE:]
        front_laser=np.concatenate((front_laser_left,front_laser_right),axis=None)
        left_laser = msg.ranges[89]
        right_laser = msg.ranges[269]

        # if we detect an object and not turning
        if not self.turning:
            for i in front_laser:
                if i > SELF_BODY and i < DETECT_RANGE:

                    #start turning
                    self.turning = True
                    # if left is closer to you (smaller sensor value)
                    # go this direction until free
                    if left_laser > right_laser:
                        self.ang_vel = ANGULAR_SPEED
                    else: # noisy sensor decision
                        self.ang_vel = -ANGULAR_SPEED
                    break


        # if we are already turning (higher threshold than initial turning)
        if self.turning:
            safe_to_go_forward = True
            for i in front_laser:
                if i > SELF_BODY and i < DETECT_RANGE:
                    safe_to_go_forward = False

    	    if safe_to_go_forward:
                self.turning = False

    def isOK(self):
        return not self.turning

    def turn(self):
        return self.ang_vel
