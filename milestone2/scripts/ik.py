import numpy as np
import scipy.linalg


class Robot:
	def __init__(self):
		self.q1=np.deg2rad(0)
		self.q2=np.deg2rad(0)
		self.q3=np.deg2rad(0)
		self.q4=np.deg2rad(0)

		#from floor underneath rotating joint to rotating joint
		self.l1=0.2
		self.l2=0.049
		self.l3=0.1
		self.l4=0.035
		self.l5=0.1
		self.l6=0.0865

	def forKin(self, q1, q2, q3, q4):
		#TODO Check if values are in degrees or rad??
		self.q1=np.deg2rad(q1)
		self.q2=np.deg2rad(q2)
		self.q3=np.deg2rad(q3)
		self.q4=np.deg2rad(q4)

		t01a=np.matrix([[1,0,0,0],
							[0,1,0,0],
							[0,0,1,self.l1+self.l2],
							[0,0,0,1]])

		t01b=t01a*np.matrix([[np.cos(self.q1), -np.sin(self.q1),0, 0],
							[np.sin(self.q1), np.cos(self.q1), 0, 0],
							[0, 0, 1, 0]
							[0, 0, 0, 1]])

		rotx90=np.matrix([[1,0,0,0],
						[0,0,-1,0],
						[0,1,0,0],
						[0,0,0,1]])

		self.t01=t01b*rotx90

		t12a=np.matrix([[np.cos(np.pi/2 + self.q2), -np.sin(np.pi/2 + self.q2),0, self.l3*np.cos(np.pi/2 + self.q2)],
							[np.sin(np.pi/2 + self.q2), np.cos(np.pi/2 + self.q2), 0, self.l3*np.sin(np.pi/2 + self.q2)],
							[0, 0, 1, 0]
							[0, 0, 0, 1]])

		self.t12=t12a*np.matrix([[np.cos(-np.pi/2), -np.sin(-np.pi/2),0, self.l4*np.cos(-np.pi/2)],
							[np.sin(-np.pi/2), np.cos(-np.pi/2), 0, self.l4*np.sin(-np.pi/2)],
							[0, 0, 1, 0]
							[0, 0, 0, 1]])

		self.t23=np.matrix([[np.cos(self.q3), -np.sin(self.q3),0, self.l5*np.cos(self.q3)],
							[np.sin(self.q3), np.cos(self.q3), 0, self.l5*np.sin(self.q3)],
							[0, 0, 1, 0]
							[0, 0, 0, 1]])

		self.t3EE=np.matrix([[np.cos(self.q4), -np.sin(self.q4),0, self.l6*np.cos(self.q4)],
							[np.sin(self.q4), np.cos(self.q4), 0, self.l6*np.sin(self.q4)],
							[0, 0, 1, 0]
							[0, 0, 0, 1]])

		self.t02=self.t01*self.t12
		self.t03=self.t02*self.t23
		self.t0EE=self.t01*self.t12*self.t23*self.t3EE

		return self.t01,self.t02,self.t03,self.t0EE

	def findJointPos(self):
		j1=[self.t01[0,3],self.t01[1,3], self.t01[2,3]]
		j2=[self.t02[0,3],self.t02[1,3], self.t02[2,3]]
		j3=[self.t03[0,3],self.t03[1,3],self.t03[2,3]]
		jEE=[self.t0EE[0,3],self.t0EE[1,3], self.t0EE[2,3]]

		return j1, j2, j3, jEE

	def geomJac(self, j1, j2, j3, jEE):
		Z0=[0,0,1]
		Z1=[self.t01[0,2],self.t01[1,2], self.t01[2,2]]
		Z2=[self.t02[0,2],self.t02[1,2], self.t02[2,2]]
		Z3=[self.t03[0,2],self.t03[1,2],self.t03[2,2]]
		ZEE=[self.t0EE[0,2],self.t0EE[1,2], self.t0EE[2,2]]

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

def main():
	rob=Robot()

	initTheta=np.array([0,0,0,0])
	target=np.array([0.3, 0, 0.3])
	fk=rob.forKin(initTheta[0],initTheta[1],initTheta[2],initTheta[3])
	j1,j2,j3,jEE=rob.findJointPos()

	newTheta=initTheta

	steps=20

	deltaStep=(target-jEE)/steps

	for i in range(steps):
		J=geomJac(j1,j2,j3,jEE)
		subtarget=np.array([deltaStep[0],deltaStep[1],deltaStep[2]])
		invJ=np.linalg.pinv(J)
		radTheta=np.dot(invJ,subtarget)

		newTheta=newTheta + radTheta

		fk=rob.forKin(newTheta[0],newTheta[1],newTheta[2], newTheta[3])
		j1,j2,j3,jEE=rob.findJointPos()


if __name__ == "__main__":
    main()


