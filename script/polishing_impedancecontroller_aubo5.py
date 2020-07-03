#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy
from numpy import matlib,linalg
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
import rospy
import yaml,os,sys

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped,TwistStamped
from ur5_planning.msg import uv
from math import *

o_path="/data/ros/yue_ws_201903/src/visionbased_polishing"
sys.path.append(o_path) 

import scripts_arm.frompitoangle
from scripts_arm.ur5_kinematics import Kinematic
from scripts_arm.hand_in_eye import *
from scripts_arm.trans_methods import *
from scripts_arm.get_arpose_from_ar import *
from scripts_arm.ur5_pose_get import *
from scripts_arm.uv_sub_node import *
from scripts_arm.structure_point_xdydzd_sub import *
from scripts_arm.structure_point_xnynan_sub import *
from scripts_arm.impedance_netf_data_get import *
from scripts_arm.ur_tool_velocity_sub import *

"""
withforce control,just cartesian control with trejactory palnning in retangle,and use structure line with z depth,x depth,y depth
"""
class VisonControl():
    def __init__(self,urdfname,ratet):
        
        self.urdfname=urdfname
        self.ace=50
        self.vel=0.1
        self.urt=0

        self.detat=0.05 #float(1.0/ratet)
        self.ur_reader = Urposition()
        self.ur_sub = rospy.Subscriber("/joint_states", JointState, self.ur_reader.callback)
        
        self.tool_get=UrToolVelocityRead()
        self.tool_velocity_sub=rospy.Subscriber("/tool_velocity", TwistStamped, self.tool_get.Ur_tool_velocity_callback)
        self.ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)

        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)
        
    def visionbased_impedancecontroller(self):
        force_list = self.netf_reader.ave_netf_force_data
        netf=[force_list[0],force_list[1],force_list[2]]
        q_now = self.ur_reader.ave_ur_pose

        joint_speed,vcc=self.get_joint_speed(q_now, netf)
        detaangle = self.get_deta_joint_angle(q_now, netf)
        # print "the deta joints angle are:", detaangle
        q_pub_next = self.get_joint_angle(q_now,detaangle)
        # print "the published joints angle are:",q_pub_next
        ss = "movej([" + str(q_pub_next[0]) + "," + str(q_pub_next[1]) + "," + str(q_pub_next[2]) + "," + str(
            q_pub_next[3]) + "," + str(q_pub_next[4]) + "," + str(q_pub_next[5]) + "]," + "a=" + str(self.ace) + "," + "v=" + str(
            self.vel) + "," + "t=" + str(self.urt) + ")"
        # print("ur5 move joints",ss)
        self.ur_pub.publish(ss)                    
                    

    def get_joint_speed(self,q,f):
        Jacabian_joint,T_06=self.get_jacabian_from_joint(self.urdfname,q,0)
        jac_b2e=tr2jac(T_06,0)
        
        lamdaf=[0.001/2,0.001/2,0.001/2]
        lamdaf_matrix=numpy.matrix([lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]).reshape((3,3))
        fd=[0.0,0.0,0.0]
        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]

        vc=lamdaf_matrix*numpy.matrix(detaf).T
        vcc=[vc.tolist()[0][0],vc.tolist()[1][0],vc.tolist()[2][0],0,0,0]
        # print "the camera velocity in camera frame is:",vcc
        v_list = vcc.reshape((1, 6)).tolist()[0]
        flag_list = [1, 1, 1, 0, 0, 0]
        vdot_z = [1.0 * v_list[i] * flag_list[i] for i in range(6)]
        # print "the end effector velocity in end effector frame", vdot_z

        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vdot_z).T)
        j_speed=numpy.dot(Jacabian_joint.I,ee_speed_in_base)
        # print "joints speed are:",j_speed
        return j_speed,vcc
    
    def get_deta_joint_angle(self,q,f):
        j_speed,Vcc=self.get_joint_speed(q,f)
        # print("joint speed is:",j_speed)
        deta_joint_angle=float(self.detat)*numpy.array(j_speed)
        # print("deta_joint_angle is:",deta_joint_angle)
        return deta_joint_angle


    def get_joint_angle(self,qnow,detajoint):
        q_next=[]
        for i in range(len(detajoint.tolist())):
            q_next.append(detajoint.tolist()[i][0]+qnow[i])
        return q_next


    def get_jacabian_from_joint(self,urdfname,jointq,flag):
        robot = URDF.from_xml_file(urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "ee_link")
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        J = kdl_kin.jacobian(jointq)
        pose = kdl_kin.forward(jointq)   
        return J,pose

def main():
    rospy.init_node("visionbased_polishingcontroller")
    ratet=5
    rate = rospy.Rate(ratet)                

    urdfname="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_description/urdf/base/paintingrobot_description.urdf.xacro"
    visionbased_polishing=VisonControl(urdfname,ratet)

    while not rospy.is_shutdown():
    # for i in range(1,100):
        try:
            visionbased_polishing.visionbased_impedancecontroller()
            rate.sleep()
        except:
            continue

if __name__=="__main__":
    main()

    "exectuing painting operation of manipulator when climbing operation is over"
    aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubo_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
    # for i in range(len(aubo_q_list)):
    #     list1=aubo_q_list["aubo_data_num_"+str(i)]
    #     print(list1)
    print("the number of aubo_q is:",len(aubo_q_list))

    time1=time.time()
    aubo5=Renovation_operation()
    aubo5.aubo_motion1(aubo_q_list,rate)
    # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
    time2=time.time()
    delta_time4=time2-time1
    self.time4_pub.publish(delta_time4)


