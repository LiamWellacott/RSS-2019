#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class Arm:

    def __init__(self):

        rospy.init_node("arm_control", anonymous=True)

        self.pub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)

        rospy.Subscriber("joint_states", JointState, self.jointCallback)

        self.routine = self.idle
        self.msg = Float64MultiArray()

        self.target_state = [0] * 6
        self.current_state = [0] * 6

    def _move(self, data):

        lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
        upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
        
        new_data = np.maximum(lower_limits,data)
        new_data = np.minimum(new_data,upper_limits)
   
        self.msg.data = new_data
        self.pub.publish(self.msg)

    def jointCallback(self, data):
        # wheel positions 0 1
        # join positions shoulder1 (2), shoulder2 (3), elbow (4), wrist (5)
        # gripper position (6)
        None # TODO how do we use this info

    def step(self):
        self.routine()
        
        directions = np.sign(np.array(self.target_state) - np.array(self.current_state))
        # increment the positibn
        new_position = []
        for i in range(len(self.current_state)):
            new_position.append(self.current_state[i] + (directions[i] * 0.05))

            # TODO remove and add threshold if we are near target
            self.current_state[i] += (directions[i] * 0.05)

        self._move(new_position)

    def setMode(self, mode):
        self.routine = mode
        self.routine(start_routine=True)

    def idle(self, start_routine=False):
        if start_routine: # reset current position
            self.target_state = [0, 0, 0, 0, 0, 0]

    def pushButton(self, start_routine=False):
        if start_routine:
            self.target_state = [0, 0, -0.8, 0, 0, 0]

    def moveObstacle(self, start_routine=False):
        None # TODO

# for testing the arm only
if __name__ == "__main__":

    arm = Arm()
    arm.setMode(arm.pushButton)
    # loop
    rate = rospy.Rate(20) # TODO spin server
    while not rospy.is_shutdown():
        arm.step()
        rate.sleep()