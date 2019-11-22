#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState 


class Arm:

    THRESHOLD = 0.02
    SPEED = 0.4
    OPEN_GRIP=1.2

    IDLE_SEQUENCE = [np.array([0.1, 0.0, 0.4, OPEN_GRIP])]
    #PUSH_BUTTON_SEQUENCE = [np.array([0.2,0.0,0.25]), IDLE_SEQUENCE[0]]
    #MOVE_OBSTACLE_SEQUENCE = [np.array([0.3, 0.0, 0.3]), np.array([0.25, 0.2, 0.2]), np.array([0.3, 0.0, 0.3]), np.array([0.25, -0.2, 0.2]), IDLE_SEQUENCE[0]]

    def __init__(self):

        #from floor underneath rotating joint to rotating joint
        self.l1=0.2
        self.l2=0.049
        self.l3=0.1
        self.l4=0.035
        self.l5=0.1
        self.l6=0.1265

        # Create rosnode for controller
        rospy.init_node("arm_control", anonymous=True)

        self.current_q = [0.,0.,0.,0., self.OPEN_GRIP] # need an inital until first val is sent to us

        # The arm controller can make use of the JointState sensor values
        rospy.Subscriber("joint_states", JointState, self.jointCallback)


        # The arm is controlled by sending messages to the joint_trajectory_point
        # which is a 6-rry = [0, shoulder1, shoulder2, elbow, wrist, gripper]
        self.pub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.msg = Float64MultiArray()

        # initially the idle sequence is set so the arm is in a neutral position
        self.startSequence(self.IDLE_SEQUENCE)
        self.next_q = np.array([0.,0.,0.,0., self.OPEN_GRIP])

    def push_button(self, x, y):
    	return [np.array([x, y, 0.185, self.OPEN_GRIP]), self.IDLE_SEQUENCE[0]]


    def move_obstacle(self, x, y):
    	#Push objects to the right if objects are already to weedle's right and vice versa
    	if y>=0.0:
    		return [np.array([x, y-0.1, 0.2, self.OPEN_GRIP]), np.array([x, y+0.1, 0.2, self.OPEN_GRIP]), self.IDLE_SEQUENCE[0]]
    	else:
    		return [np.array([x, y+0.1, 0.2, self.OPEN_GRIP]), np.array([x, y-0.1, 0.2, self.OPEN_GRIP]), self.IDLE_SEQUENCE[0]]

    def pickup(self, x ,y):
    	return [np.array([x-(0.02*np.cos(np.arctan(y/x))), y-(0.02*np.sin(np.arctan(y/x))), 0.05, self.OPEN_GRIP]), np.array([x, y, 0.05, 0.3]), np.array([x, y, 0.25, self.OPEN_GRIP]), self.IDLE_SEQUENCE[0]]


    def jointCallback(self, data):
        # wheel positions (0) (1)
        # join positions shoulder1 (2), shoulder2 (3), elbow (4), wrist (5)
        # gripper position (6)
        self.current_q = list(data.position[2:7])
        #self.gripper_state = data.position[6]

        #ospy.loginfo("data: %s, position: %s \r\n" % (data, self.current_q))

        self.step()

    def _transform(self, q, l):
        return np.matrix([[np.cos(q), -np.sin(q),0, l*np.cos(q)],
                            [np.sin(q), np.cos(q), 0, l*np.sin(q)],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

    def _sendCommand(self):

        # The values sent to thfindJointPose arm controller must be within 
        # the bounds set here or the motors in the arm can crash

        data = [0] + list(self.next_q)
        lower_limits = [0, -1.5, -1.5, -1.5, -1.5, -0.9]
        upper_limits = [0,  1.5,  1.5,  1.5,  1.5, 1.5]
        #original values at 1.57

        #rospy.loginfo("%s" % data)
        new_data = np.maximum(lower_limits,data)
        new_data = np.minimum(new_data,upper_limits)
   
        self.msg.data = new_data
        self.pub.publish(self.msg)

    def _forKin(self):

        # from base to first joint
        t01a=np.matrix([[1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,self.l1+self.l2],
                            [0,0,0,1]])

        t01b=t01a*self._transform(self.current_q[0], 0)

        rotx90=np.matrix([[1,0,0,0],
                        [0,0,-1,0],
                        [0,1,0,0],
                        [0,0,0,1]])
        t01=t01b*rotx90

        # from first to second
        t12a=self._transform(np.pi/2.+self.current_q[1],self.l3)
        t12=t12a*self._transform(-np.pi/2., self.l4)

        # from second to third
        t23=self._transform(self.current_q[2],self.l5)

        # from third to end
        t3EE=self._transform(self.current_q[3],self.l6)

        # from base to each joint
        t02=t01*t12
        t03=t02*t23
        t0EE=t01*t12*t23*t3EE
        return t01, t02, t03, t0EE

    def findJointPos(self):
        t01, t02, t03, t0EE=self._forKin()

        j1=np.array([t01[0,3], t01[1,3], t01[2,3]])
        j2=np.array([t02[0,3], t02[1,3], t02[2,3]])
        j3=np.array([t03[0,3], t03[1,3], t03[2,3]])
        jEE=np.array([t0EE[0,3], t0EE[1,3], t0EE[2,3]])

        return j1, j2, j3, jEE

    def geomJac(self, j1, j2, j3, jEE):
        t01, t02, t03, t0EE=self._forKin()
        Z0=np.array([0,0,1])
        Z1=np.array([t01[0,2],t01[1,2], t01[2,2]])
        Z2=np.array([t02[0,2],t02[1,2], t02[2,2]])
        Z3=np.array([t03[0,2],t03[1,2],t03[2,2]])
        ZEE=np.array([t0EE[0,2],t0EE[1,2], t0EE[2,2]])

        j0=[0,0,0]
        Jp1=np.cross(Z0,jEE-j0)
        Jp2=np.cross(Z1,jEE-j1)
        Jp3=np.cross(Z2,jEE-j2)
        JpEE=np.cross(Z3,jEE-j3)

        Jo1=Z0
        Jo2=Z1
        Jo3=Z2
        JoEE=Z3

        Jp=np.array([Jp1, Jp2, Jp3, JpEE]).T

        return Jp

    def step(self):
        '''
        periodic update of the arm, if a sequence has been requested progress in the sequence happens here
        '''
        if self._routineFinished():
            return # nothing to do

        # Calculate current joint position
        j1,j2,j3,jEE = self.findJointPos()

        # Convert to XYZ
        self.current_xyzg = np.hstack((jEE,self.current_q[4]))
        diff_xyz_ = (self.target_xyzg[0:3] - self.current_xyzg[0:3])
        diff_xyz=diff_xyz_ /(0.0001+np.linalg.norm(diff_xyz_)*25)

        J=self.geomJac(j1,j2,j3,jEE)
        invJ = np.linalg.pinv(J)
        radTheta=np.dot(invJ,diff_xyz)
        directions = np.sign(radTheta)

        grip_diff = (self.target_xyzg[3]-self.current_q[4])


        # TODO use the sensor values
        for i in range(len(radTheta)):
            self.next_q[i] = self.current_q[i] + radTheta[i]
        self.next_q[4]= self.current_q[4] + grip_diff
#        for i in range(len(radTheta)):
#            self.next_q[i] = self.current_q[i] + (directions[i]*self.SPEED)

        #rospy.loginfo(("Current xyzg: ", self.current_xyzg, "| Target: ", self.target_xyzg, "| Diff_xyz_ :",
         #diff_xyz_,"| Diff_xyz: ", diff_xyz , "| RadTheta:", radTheta, "| Next q: ", self.next_q))


        # Send the new state to the arm.
        self._sendCommand()

        # check if we need to move to the next stage of the routine
        self._stepRoutine()

    def startSequence(self, seq):
        '''
        sets the passed sequence and sets the initial target position
        '''

        self.sequence = seq
        self._stepRoutine(start_routine=True)

    def _outOfRange(self):
        current_R=np.sqrt(self.current_xyzg[0]**2 + self.current_xyzg[1]**2 + (0.249-self.current_xyzg[2])**2)
        target_R=np.sqrt(self.target_xyzg[0]**2 + self.target_xyzg[1]**2 + (0.249-self.target_xyzg[2])**2)
        return target_R + current_R > 0.66

    def _atTarget(self):
        '''
        true if all joint positions are in the target state (by THRESHOLD) 
        '''
        for i in range(len(self.current_xyzg)):
            if not self._atJointTarget(i):
                return False
        return True

    def _atJointTarget(self, dimension):
        '''
        true if the difference between target and current for given dimension is under THRESHOLD
        '''
        return abs(self.target_xyzg[dimension] - self.current_xyzg[dimension]) < self.THRESHOLD

    def _routineFinished(self):
        '''
        true if the current stage is the end of the sequence
        '''
        #rospy.loginfo(("Routine is finished returns: ", self.stage == len(self.sequence)))

        return self.stage == len(self.sequence)

    def _stepRoutine(self, start_routine=False):
        if start_routine:
            self.stage = 0
            self.target_xyzg = self.sequence[self.stage]
        else:
            if self._atTarget() or self._outOfRange(): # if at next objective
                # move to the next stage
                if self._outOfRange():
                    rospy.loginfo("Out of Range")
                self.stage += 1
                if not self._routineFinished(): 
                    self.target_xyzg = self.sequence[self.stage]
                    rospy.loginfo("Next Target")
                else:
                	rospy.loginfo("Target Achieved")

def main():
    arm=Arm()

    once = True

    rate = rospy.Rate(20) # TODO spin server
    while not rospy.is_shutdown():
        rate.sleep()

        #arm.step()

        if arm._routineFinished() and once:
            arm.startSequence(arm.pickup(0.25 ,0.1))
            once = False

if __name__ == "__main__":
    main()


