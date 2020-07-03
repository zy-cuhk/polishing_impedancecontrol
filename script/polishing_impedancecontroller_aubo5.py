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
        self.detat=0.05 #float(1.0/ratet)        
        self.netf_reader = NetfData()
        self.netf_sub = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.netf_reader.callback)
        
        self.aubo_pose_sub = rospy.Subscriber('/renov_up_level/aubo_pose', Pose, queue_size=10)
        # self.aubo_joint_sub = rospy.Subscriber('/renov_up_level/aubo_joints', JointState, queue_size=10)
        # self.aubo_vel_sub = rospy.Subscriber('/renov_up_level/aubo_vel', catersian_vel,queue_size=10)
        # self.aubo_status_sub = rospy.Subscriber('/renov_up_level/aubo_status',physical_para ,queue_size=10)

        self.aubo_q=[0,pi/2,pi/2,pi/2,0,0]
        self.robot = URDF.from_xml_file("/home/zy/catkin_ws/src/polishingrobot_yhl/polishing_undervibration/script/aubo_i5.urdf")


    def visionbased_impedancecontroller1(self,urdfname):
        kdl_kin = KDLKinematics(self.robot, "base_link", "wrist3_Link")
        pose = kdl_kin.forward(self.aubo_q) 
        Jacobian = kdl_kin.jacobian(self.aubo_q)
        jac_b2e=tr2jac(pose,0)
        
        lamdaf=[0.001/2,0.001/2,0.001/2]
        lamdaf_matrix=numpy.matrix([lamdaf[0],0,0,0,lamdaf[1],0,0,0,lamdaf[2]]).reshape((3,3))
        force_list = self.netf_reader.ave_netf_force_data
        f=[force_list[0],force_list[1],force_list[2]]
        fd=[0.0,0.0,0.0]
        detaf = [f[0]-fd[0],f[1]-fd[1],f[2]-fd[2]]

        vc=lamdaf_matrix*numpy.matrix(detaf).T
        vcc=[vc.tolist()[0][0],vc.tolist()[1][0],vc.tolist()[2][0],0,0,0]
        ee_speed_in_base = np.dot(jac_b2e.I, numpy.mat(vcc).T)
        j_speed=numpy.dot(Jacobian.I,ee_speed_in_base)
        deta_joint_angle=float(self.detat)*numpy.array(j_speed)
        q_next=[]
        for i in range(len(detajoint.tolist())):
            q_next.append(detajoint.tolist()[i][0]+qnow[i])
        return q_next



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


