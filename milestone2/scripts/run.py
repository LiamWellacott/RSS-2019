#!/usr/bin/env python
import rospy
import rospkg

import json
from milestone2.msg import Task

POI_FILE = "/config/poi.json"
DELAY = 3

def main():
    rospack = rospkg.RosPack()
    path = rospack.get_path('milestone2')
    with open(path + POI_FILE) as file:
        poi = json.load(file)
    rospy.init_node("milestone2_poi", anonymous=True)
    pub = rospy.Publisher('task', Task, queue_size = 10)
    msg = Task()
    rospy.sleep(DELAY)

    # Go to the button
    p1 = poi['Button']
    p1_ = [p1[0] , p1[1] - 0.35]
    msg.task = "goal"
    msg.objective = p1_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    msg.task = "smash"
    msg.objective = p1
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    p2 = poi['Box1']
    p2_ = [p2[0] + 2., p2[1] + 0.4]
    msg.task = "goal"
    msg.objective = p2_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    p2_ = [p2[0] + 1., p2[1] + 0.4]
    msg.task = "goal"
    msg.objective = p2_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    p2_ = [p2[0], p2[1] + 0.35]
    msg.task = "goal"
    msg.objective = p2_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    msg.task = "move"
    msg.objective = p2
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p3 = poi['Lego1']
    p3_ = [p3[0], p3[1] + 0.35]
    msg.task = "goal"
    msg.objective = p3_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    msg.objective = p3
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p4 = poi['Lego3']
    p4_ = [p4[0] - 0.35, p4[1]]
    msg.task = "goal"
    msg.objective = p4_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    msg.objective = p4
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p5 = poi['Lego2']
    p5_ = [p5[0], p5[1] - 0.35]
    msg.task = "goal"
    msg.objective = p5_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    msg.objective = p5
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p6 = poi['Origin']
    p6_ = [p6[0] + 0.3, p6[1]]
    msg.task = "goal"
    msg.objective = p6_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)



if __name__ == "__main__":
    main()
