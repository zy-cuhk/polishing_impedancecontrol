#!/usr/bin/env python
# -*- coding: utf-8 -*-

import yaml,os,sys, rospy
# print("the paths are:",sys.path)

import numpy
from numpy import matlib,linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped,TwistStamped
from math import *
import frompitoangle
from trans_methods import *
from impedance_netf_data_get import *
from aubo_motion import *

class Impedancecontrol():
    def __init__(self,urdfname,ratet):
        self.aubo_q=[]
        # self.aubo_q=[0,pi/2,pi/2,pi/2,0,0]

        self.robot = URDF.from_xml_file(urdfname)
        self.detat=float(1.0/ratet)

        self.aubo5=Renovation_operation()
        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)
        # self.aubo_pose_sub = rospy.Subscriber('/renov_up_level/aubo_pose', Pose, self.obtain_aubo_pose)
        self.aubo_joint_sub = rospy.Subscriber('/renov_up_level/aubo_joints', JointState, self.obtain_aubo_joints)
        # self.aubo_vel_sub = rospy.Subscriber('/renov_up_level/aubo_vel', catersian_vel,queue_size=10)
        # self.aubo_status_sub = rospy.Subscriber('/renov_up_level/aubo_status',physical_para ,queue_size=10)

        self.aubo_move_track_pub=rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
        self.aubo_move_joint_pub = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=1)
        self.aubo_move_line_pub = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=1)
    def obtain_aubo_joints(self,msg):
        # self.aubo_q=[]
        # list1=msg.position
        # for i in range(len(list1)):
        #     self.aubo_q.append(list1[i])
        list1=msg.position
        self.aubo_q=list1[:]   

    def impedancecontroller(self):

        print("the aubo joints is:",self.aubo_q)
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        pose = kdl_kin.forward(self.aubo_q) 
        Jacobian = kdl_kin.jacobian(self.aubo_q)
        jac_b2e=tr2jac(pose,0)
        print("the pose is:",pose)
        print("the jacobian matrix is:", Jacobian)
        print("the jac_b2e is:",jac_b2e)

        
        lamdaf=[0.0,0.0,0.0001]
        lamdaf_matrix=numpy.matrix([lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]).reshape((3,3))

        force_list = self.netf_reader.ave_netf_force_data
        print("the present force is:",force_list)
        f=[force_list[0],force_list[1],force_list[2]]
        fd=[force_list[0],force_list[1],-2.5]
        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]

        vc=lamdaf_matrix*numpy.matrix(detaf).T
        print("vc is:",vc)
        vcc=[vc.tolist()[0][0],vc.tolist()[1][0],vc.tolist()[2][0],0,0,0]
        print("vcc is:",vcc)
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vcc).T)
        print("ee speed in base is:",ee_speed_in_base)
        j_speed=numpy.dot(Jacobian.I,ee_speed_in_base)
        print("j speed is",j_speed)
        deta_joint_angle=float(self.detat)*numpy.array(j_speed)
        print("delta joints angle is:",deta_joint_angle)

        aubo_q_next=[]
        for i in range(len(deta_joint_angle.tolist())):
            aubo_q_next.append(deta_joint_angle.tolist()[i][0]+self.aubo_q[i])
        print("aubo_q is:",self.aubo_q)
        print("aubo_q_next is:",aubo_q_next)
            
        pubstring1="movej"+str(tuple(self.aubo_q))+str(tuple(aubo_q_next))
        # group_joints=str(tuple(aubo_q_next))
        # print("group_joints",group_joints)
        print("pubstring1 is:",pubstring1)

        if "movej" in pubstring1:
            self.aubo_move_joint_pub.publish(pubstring1)
        elif "movel" in pubstring1:
            self.aubo_move_line_pub.publish(pubstring1)
        elif "movet" in pubstring1:
            self.aubo_move_track_pub.publish(pubstring1)

    def group_joints_to_string(self,q_list):
        group_joints=""
        for i in range(len(q_list)):
            group_joints+=str(tuple(q_list[i]))
        return group_joints


def main():
    rospy.init_node("visionbased_polishingcontroller")
    ratet=5
    rate = rospy.Rate(ratet)                

    urdfname="/data/ros/renov_robot_ws/src/polishingrobot_underusing/polishing_undervibration/script/aubo_i5.urdf"
    polishing=Impedancecontrol(urdfname,ratet)

    while not rospy.is_shutdown():
        try:
            polishing.impedancecontroller()
            rate.sleep()
        except:
            continue

if __name__=="__main__":
    main()




