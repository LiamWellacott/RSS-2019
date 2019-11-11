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
    p1 = poi['Button']
    p1_ = [p1[0] - 0.2, p1[1] - 0.2]
    msg.task = "goal"
    msg.objective = p1_
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "smash"
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    p2 = poi['Box1']
    msg.task = "goal"
    msg.objective = p2
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "move"
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p3 = poi['Lego1']
    msg.task = "goal"
    msg.objective = p3
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p4 = poi['Lego2']
    msg.task = "goal"
    msg.objective = p4
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p5 = poi['Lego3']
    msg.task = "goal"
    msg.objective = p5
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)

    msg.task = "pick"
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)


    p6 = poi['Origin']
    msg.task = "goal"
    msg.objective = p6
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.sleep(DELAY)



if __name__ == "__main__":
    main()
