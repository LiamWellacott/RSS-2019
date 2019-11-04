#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class Arm:

    THRESHOLD = 0.1

    IDLE_SEQUENCE = [[0,0,0,0,0,0]]
    PUSH_BUTTON_SEQUENCE = [[0, 0, -0.8, 0, 0, 0]]
    MOVE_OBSTACLE_SEQUENCE = [[0, 0, -1.0, 0.5, 0, 0], [0, 1, -1, 0.5, 0, 0], [0, -1, -1, 0.5, 0, 0]]

    def __init__(self):

        rospy.init_node("arm_control", anonymous=True)

        self.pub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)

        rospy.Subscriber("joint_states", JointState, self.jointCallback)

        self.routine = self.idle
        self.msg = Float64MultiArray()

        self.target_state = [0] * 6
        self.current_state = [0] * 6

        self.is_idle = True

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
        
        directions = np.sign(np.array(self.target_state) - np.array(self.current_state))
        # increment the positibn
        new_position = []
        for i in range(len(self.current_state)):
            new_position.append(self.current_state[i])
            if not self._atPositionDim(self.target_state[i], i):
                new_position[i] += directions[i] * 0.05
                
                # TODO remove and use sensor
                self.current_state[i] += (directions[i] * 0.05)

        self._move(new_position)

        self.routine()

    def _atTarget(self):
        return self._atPosition(self.target_state)

    def _atPosition(self, other):
        for i in range(len(self.current_state)):
            if not self._atPositionDim(other[i], i):
                return False
        return True

    def _atPositionDim(self, other, dimension):
         return abs(other - self.current_state[dimension]) < self.THRESHOLD

    def isRoutineFinished(self):
        return self.is_idle

    def setMode(self, mode):
        self.routine = mode
        self.is_idle = False
        self.routine(start_routine=True)

    def idle(self, start_routine=False):
        if start_routine: # reset current position
            self.target_state = self.IDLE_SEQUENCE[0]
        if self._atTarget():
            self.is_idle = True

    def pushButton(self, start_routine=False):
        if start_routine:
            self.target_state = self.PUSH_BUTTON_SEQUENCE[0]
            rospy.loginfo("Starting Routine")

        else:
            if self._atTarget():
                rospy.loginfo("At target")
                self.setMode(self.idle)
                

    def moveObstacle(self, start_routine=False):
        if start_routine:
            self.target_state = self.MOVE_OBSTACLE_SEQUENCE[0]

        else:
            if self._atTarget():

                for i in range(len(self.MOVE_OBSTACLE_SEQUENCE)):
                    if self._atPosition(self.MOVE_OBSTACLE_SEQUENCE[i]):
                        if i == len(self.MOVE_OBSTACLE_SEQUENCE):
                            self.setMode(self.idle)
                        else:
                            self.target_state = self.MOVE_OBSTACLE_SEQUENCE[i+1]


# for testing the arm only
if __name__ == "__main__":

    arm = Arm()
    arm.setMode(arm.pushButton)
    # loop
    rate = rospy.Rate(20) # TODO spin server
    while not rospy.is_shutdown():
        arm.step()
        rate.sleep()