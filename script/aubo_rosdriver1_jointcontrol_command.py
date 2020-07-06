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
    joint_radian1 = deg_to_rad((0.0,-16.3,155.6,70.7,-87.9,-89.9))
    print("joint_radian1 is:",joint_radian1)
    "the joints of the points to the wall plane is joint_radian2"
    joint_radian2 = deg_to_rad((40.451,-64.906,81.583,-29.235,-50.452,-82.019))
    print("joint_radian2 is:",joint_radian2)
    pub1 = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    # pub2 = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    # pub3 = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)
    rospy.init_node('aubo_ros_test', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    count=1
    while not rospy.is_shutdown():
        for i in range(3):
            movej_points="movej"+str(joint_radian2)+str(joint_radian1)     
            pub1.publish(movej_points)
            rate.sleep()   
            
            movej_points="movej"+str(joint_radian1)+str(joint_radian2)
            pub1.publish(movej_points)
            rate.sleep()

if __name__ == '__main__':
    try:
        aubo_ros_test()
    except rospy.ROSInterruptException:
        pass


