#!/usr/bin/env python
# coding=utf-8

import rospy
import math
from std_msgs.msg import String
def deg_to_rad(tuplelist):
    dd=[]
    for i in tuplelist:
        dd.append(i*math.pi/180)
    return tuple(dd)

def aubo_ros_test():
    "the joints of original points is joint_radian1"
    joint_radian1 = ((0.0,-0.24435,2.7524,-0.3,-1.4835,-1.57))
    "the joints of the points to the wall plane is joint_radian2"

    pub1 = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    # pub2 = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    # pub3 = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)
    rospy.init_node('aubo_ros_test', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    while not rospy.is_shutdown():
        movej_points="movej"+str(joint_radian1)
        #movel_points="movel"+str(joint_radian1)+str(joint_radian2)
        # movet_points="movej"+str(joint_radian1)+str(joint_radian2)+str(joint_radian3)+str(joint_radian4)
        # pub1.publish(movej_points)
        # pub2.publish(movel_points)
        pub1.publish(movej_points)
        rate.sleep()

if __name__ == '__main__':
    try:
        aubo_ros_test()
    except rospy.ROSInterruptException:
        pass


