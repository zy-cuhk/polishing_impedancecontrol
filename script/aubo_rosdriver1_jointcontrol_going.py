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
    "(47.505,-95.843,46.771,-32.59,-43.42,-82.78)"
    "(57.154,-78.596,98.767,4.486,-31.256,-87.290)"
    joint_radian2 = deg_to_rad((66.857,-76.32,96.278,0.665,-24.18,-86.66))
    print("joint_radian2 is:",joint_radian2)
    joint_radian3 = deg_to_rad((49.359,-82.680,67.966,-24.38,-41.57,-83.01))
    print("joint_radian3 is:",joint_radian3)
    pub1 = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=10)
    # pub2 = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=10)
    # pub3 = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=10)
    rospy.init_node('aubo_ros_test', anonymous=True)
    rate = rospy.Rate(20) # 1hz
    count=1
    # while not rospy.is_shutdown():
        # for i in range(3):    
    movej_points="movej"+str(joint_radian1)+str(joint_radian2)
    pub1.publish(movej_points)
    rate.sleep()

    movej_points="movej"+str(joint_radian2)+str(joint_radian3)
    pub1.publish(movej_points)
    rate.sleep()

if __name__ == '__main__':
    try:
        aubo_ros_test()
    except rospy.ROSInterruptException:
        pass


