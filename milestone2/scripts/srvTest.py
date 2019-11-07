#!/usr/bin/env python

import rospy
from milestone2.srv import RRTsrv, RRTsrvResponse
import numpy as np
import matplotlib.pyplot as plt

def getPath():
    rospy.loginfo("Waiting for rrt service...")
    rospy.wait_for_service('rrt')
    rospy.loginfo("Ask for a path")
    try:
        rrt = rospy.ServiceProxy('rrt', RRTsrv)
        resp1 = rrt([.5, .5], [3.5, 2.5])
        path = np.array(resp1.path).reshape((-1,2))
        #print(path)
        plt.plot(path[:,0], path[:,1])
        plt.show()
        return path
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return 0


if __name__ == "__main__":
    print('SRV TEST')
    getPath()
