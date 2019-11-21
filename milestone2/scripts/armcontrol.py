#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class Arm:

    THRESHOLD = 0.1
    SPEED = 0.05

    IDLE_SEQUENCE = [[0,0,0,1.57,0,0]]
    PUSH_BUTTON_SEQUENCE = [[0, 0, -0.8, 0, 0, 0], IDLE_SEQUENCE[0]]
    MOVE_OBSTACLE_SEQUENCE = [[0, 0, -1.0, 0.5, 0, 0], [0, 1, -1, 0.5, 0, 0], [0, -1, -1, 0.5, 0, 0], IDLE_SEQUENCE[0]]
    PICKUP_PREPARE=[[0,0,-1.5,0,0.8,1]]
    PICKUP_SEQUENCE = [[0,0,-1.5,0,0.8,1], [0,0,-1.5,0,0.8,0.2], [0,0,0,0,0.8,0.2],[0,0,-1.4,0,0.8,0.2],[0,0,-1.4,0,0.8,1], IDLE_SEQUENCE[0]]

    def __init__(self):

        # Create rosnode for controller
        rospy.init_node("arm_control", anonymous=True)
        
        # The arm controller can make use of the JointState sensor values
        # Not currently used 
        rospy.Subscriber("joint_states", JointState, self.jointCallback)

        # The arm is controlled by sending messages to the joint_trajectory_point
        # which is a 6-rry = [0, shoulder1, shoulder2, elbow, wrist, gripper]
        self.pub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.msg = Float64MultiArray()

        # To avoid snapping between each desired state we define a target and move by a fixed amount 
        # each time step
        self.target_state = [0] * 6
        self.current_state = [0] * 6

        # initially the idle sequence is set so the arm is in a neutral position
        self.sequence = self.IDLE_SEQUENCE
        self.stage = 0

    def _sendCommand(self, data):

        # The values sent to the arm controller must be within 
        # the bounds set here or the motors in the arm can crash
        lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
        upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]

        new_data = np.maximum(lower_limits,data)
        new_data = np.minimum(new_data,upper_limits)
   
        self.msg.data = new_data
        self.pub.publish(self.msg)

    def jointCallback(self, data):
        # wheel positions (0) (1)
        # join positions shoulder1 (2), shoulder2 (3), elbow (4), wrist (5)
        # gripper position (6)
        return

    def step(self):
        '''
        periodic update of the arm, if a sequence has been requested progress in the sequence happens here
        '''
        if self._routineFinished():
            return # nothing to do

        # get the direction each joint needs to move to reach the target
        directions = np.sign(np.array(self.target_state) - np.array(self.current_state))
        
        # update the position of each joint
        new_position = []
        for i in range(len(self.current_state)): 
            new_position.append(self.current_state[i]) # new position starts from current
            if not self._atJointTarget(i): # if this joint is not in the target position
                new_position[i] += directions[i] * self.SPEED # update by SPEED
                # TODO remove and use sensor
                self.current_state[i] = new_position[i] # Update our current position

        # Send the new state to the arm.
        self._sendCommand(new_position)

        # check if we need to move to the next stage of the routine
        self._stepRoutine()

    def startSequence(self, seq):
        '''
        sets the passed sequence and sets the initial target position
        '''
        self.sequence = seq
        self._stepRoutine(start_routine=True)

    def _atTarget(self):
        '''
        true if all joint positions are in the target state (by THRESHOLD) 
        '''
        for i in range(len(self.current_state)):
            if not self._atJointTarget(i):
                return False
        return True

    def _atJointTarget(self, dimension):
        '''
        true if the difference between target and current for given dimension is under THRESHOLD
        '''
        return abs(self.target_state[dimension] - self.current_state[dimension]) < self.THRESHOLD

    def _routineFinished(self):
        '''
        true if the current stage is the end of the sequence
        '''
        return self.stage == len(self.sequence)

    def _stepRoutine(self, start_routine=False):
        if start_routine:
            self.stage = 0
            self.target_state = self.sequence[self.stage]
        else:
            if self._atTarget(): # if at next objective
                # move to the next stage
                self.stage += 1
                if not self._routineFinished(): 
                    self.target_state = self.sequence[self.stage]

# for testing the arm only
if __name__ == "__main__":

    arm = Arm()
    arm.startSequence(arm.MOVE_OBSTACLE_SEQUENCE)
    # loop
    rate = rospy.Rate(20) # TODO spin server
    while not rospy.is_shutdown():
        arm.step()
        rate.sleep()
