#!/usr/bin/env python

import rospy
import numpy as np
#import scipy.linalg
from std_msgs.msg import Float64MultiArray


class Arm:

    SPEED = 0.05

    def __init__(self):

        #from floor underneath rotating joint to rotating joint
        self.l1=0.2
        self.l2=0.049
        self.l3=0.1
        self.l4=0.035
        self.l5=0.1
        self.l6=0.0865

        rospy.init_node("arm_control", anonymous=True)

        self.pub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10)
        self.msg = Float64MultiArray()

        self.target_xyz = [0] * 3
        self.current_q = [0] * 4

        # TODO
        self.gripper_state = 0

    def _transform(self, q, l):
        return np.matrix([[np.cos(q), -np.sin(q),0, l*np.cos(q)],
                            [np.sin(q), np.cos(q), 0, l*np.sin(q)],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

    def _move(self):

        data = [0] + self.current_q + [self.gripper_state]
        lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
        upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]

        rospy.loginfo("%s" % data)
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
        self.t01=t01b*rotx90

        # from first to second
        t12a=self._transform(np.pi/2.+self.current_q[1],self.l3)
        self.t12=t12a*self._transform(-np.pi/2., self.l4)

        # from second to third
        self.t23=self._transform(self.current_q[2],self.l5)

        # from third to end
        self.t3EE=self._transform(self.current_q[3],self.l6)

        # from base to each joint
        self.t02=self.t01*self.t12
        self.t03=self.t02*self.t23
        self.t0EE=self.t01*self.t12*self.t23*self.t3EE

    def findJointPos(self):
        self._forKin()

        j1=np.array([self.t01[0,3],self.t01[1,3], self.t01[2,3]])
        j2=np.array([self.t02[0,3],self.t02[1,3], self.t02[2,3]])
        j3=np.array([self.t03[0,3],self.t03[1,3],self.t03[2,3]])
        jEE=np.array([self.t0EE[0,3],self.t0EE[1,3], self.t0EE[2,3]])

        return j1, j2, j3, jEE

    def geomJac(self, j1, j2, j3, jEE):
        Z0=np.array([0,0,1])
        Z1=np.array([self.t01[0,2],self.t01[1,2], self.t01[2,2]])
        Z2=np.array([self.t02[0,2],self.t02[1,2], self.t02[2,2]])
        Z3=np.array([self.t03[0,2],self.t03[1,2],self.t03[2,2]])
        ZEE=np.array([self.t0EE[0,2],self.t0EE[1,2], self.t0EE[2,2]])

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

    def setTarget(self, target):
        self.target_xyz = target

    def step(self):

        j1,j2,j3,jEE = self.findJointPos()

        step = (self.target_xyz - jEE) * self.SPEED

        J=self.geomJac(j1,j2,j3,jEE)
        invJ = np.linalg.pinv(J)

        radTheta=np.dot(invJ,step)

        # no assignment, use the sensor values
        for i, theta in enumerate(radTheta):
            self.current_q[i] += theta

        self._move()


def main():
    rob=Arm()

    # go from initial position to xyz
    rob.setTarget(np.array([0.1, 0, 0.4]))

    rate = rospy.Rate(20) # TODO spin server
    while not rospy.is_shutdown():
        rob.step()
        rate.sleep()

if __name__ == "__main__":
    main()


