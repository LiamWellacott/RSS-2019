#!/usr/bin/env python

import rospy
from milestone2.srv import RRTsrv, RRTsrvResponse

def getPath():
    rospy.loginfo("Waiting for rrt service...")
    rospy.wait_for_service('rrt')
    rospy.loginfo("Ask for a path")
    try:
        rrt = rospy.ServiceProxy('rrt', RRTsrv)
        resp1 = rrt([.5, .5], [3.5, 2.5])
        return resp1.path
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return 0


if __name__ == "__main__":
    print('SRV TEST')
    print(getPath())
