#!/usr/bin/env python

#coding:utf-8

import rospy
import tf_conversions
import geometry_msgs.msg
import moveit_msgs.msg
import time
import threading
import ctypes
from ctypes import *
import numpy as np
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander 
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject



class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",c_uint),
            ("AccMask",c_uint),
            ("Reserved",c_uint),
            ("Filter",c_ubyte),
            ("Timing0",c_ubyte),
            ("Timing1",c_ubyte),
            ("Mode",c_ubyte)
            ]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID",c_uint),
            ("TimeStamp",c_uint),
            ("TimeFlag",c_ubyte),
            ("SendType",c_ubyte),
            ("RemoteFlag",c_ubyte),
            ("ExternFlag",c_ubyte),
            ("DataLen",c_ubyte),
            ("Data",c_ubyte*8),
            ("Reserved",c_ubyte*3)
            ]

class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [('SIZE',ctypes.c_uint16),('STRUCT_ARRAY',ctypes.POINTER(VCI_CAN_OBJ))]

    def __init__(self,num_of_structs):
        self.STRUCT_ARRAY = ctypes.cast((VCI_CAN_OBJ * num_of_structs)(),ctypes.POINTER(VCI_CAN_OBJ))
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]

canDLL = None
VCI_USBCAN2 = 4
STATUS_OK = 1 


class Ti5_py:
    def __init__(self):
        
        rospy.init_node('client_node',anonymous=True)
        rospy.Subscriber('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,self.callback)
        self.init_can()

        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander("armgroup")

        self.group.set_goal_position_tolerance(0.001)
        self.group.set_goal_orientation_tolerance(0.01)
        self.group.set_goal_joint_tolerance(0.001)
        self.group.set_max_acceleration_scaling_factor(0.5)
        self.group.set_max_velocity_scaling_factor(0.5)

        self.reference_frame="base_link"
        self.group.set_pose_reference_frame(self.reference_frame)
        self.group.allow_replanning(True)
        self.group.set_planning_time(5.0)

        self.current_state=self.robot.get_current_state()
        self.joint_model_group=self.robot.get_group_names()
        self.end_effector_link=self.group.get_end_effector_link()
       


    def init_can(self):
        global canDLL
        global VCI_USBCAN2
        global STATUS_OK
        can_dll_name = '/home/ti5robot/controlcan/libcontrolcan.so'
        canDLL = cdll.LoadLibrary('/home/ti5robot/controlcan/libcontrolcan.so')

        print(can_dll_name)

        ret = canDLL.VCI_OpenDevice(VCI_USBCAN2,0,0)
        if ret == STATUS_OK:
            print("VCI_OpenDevice okk")
        else:
            print("VCI_OpenDevice fail")

        ## init tongdao 0
        vci_initconfig = VCI_INIT_CONFIG(0x80000008,0xFFFFFFFF,0,1,0x00,0x14,0) ### 1000K
        ret = canDLL.VCI_InitCAN(VCI_USBCAN2,0,0,byref(vci_initconfig))

        if ret == STATUS_OK:
            print("VCI_InitCAN okk")
        else:
            print("VCI_InitCAN fail")

        ret = canDLL.VCI_StartCAN(VCI_USBCAN2,0,0)
        if ret == STATUS_OK:
            print("VCI_startCAN okk")
        else:
            print("VCI_StartCAN fail")
            canDLL.VCI_CloseDevice(VCI_USBCAN2,0)

        ## init tongdao 1

        ret =canDLL.VCI_InitCAN(VCI_USBCAN2,0,1,byref(vci_initconfig))
        if ret == STATUS_OK:
            print("VCI_InitCAN2 okk")
        else:
            print("VCI_InitCAN2 fail")

        ret = canDLL.VCI_StartCAN(VCI_USBCAN2,0,1)
        if ret == STATUS_OK:
            print("VCI_StartCAN2 okk")
        else:
            print("VCI_StartCAN2 fail")
            canDLL.VCI_CloseDevice(VCI_USBCAN2,0)


    def find_max(self,arr):
        max_val=abs(arr[0])
        for i in range(1,6):
            print(i)
            if abs(arr[i]) > abs(max_val):
                max_val = abs(arr[i])
        return max_val


    def convert_hex_array_to_decimal(self,hex_array):
        result=0
        for i in range(4):
            result=(result<<8) | hex_array[i]
        if result > 0x7FFFFFFF:
            result -= 0x100000000
        return result



    def to_int_array(self,number,size):
        unsigned_number = number if number >= 0 else (1 << 32) + number

        res = []
        for i in range(size):
            res.append(unsigned_number & 0xFF)
            unsigned_number >>= 8
        return res
    

    def send_simple_can_command(self,num_of_actuator,can_id_list,command):
        global canDLL
        global VCI_USBCAN2
        global STATUS_OK
        send = VCI_CAN_OBJ()
        send.ID = 1
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1

        for i in range(6):
            send.Data[0] = command

            if canDLL.VCI_Transmit(VCI_USBCAN2,0,0,byref(send),1) == 1:
                rec = VCI_CAN_OBJ_ARRAY(3000)
                ind = 0
                reclen = canDLL.VCI_Receive(VCI_USBCAN2,0,0,byref(rec.ADDR),3000,0)
                if reclen > 0:
                    for j in range(reclen):
                        hex_array = [rec[j].Data[4],rec[j].Data[3],rec[j].Data[2],rec[j].Data[1]]
                        decimal = self.convert_hex_array_to_decimal(hex_array)

                send.ID += 1
            else:
                break
        print("send_simple_can_command over")


    def send_can_command(self,num_of_actuator,can_id_list,command_list,parameter_list):
        global canDLL
        global VCI_USBCAN2
        global STATUS_OK
        send = VCI_CAN_OBJ()
    
        send.ID = 1
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 5
    
        for i in range(6):
            send.Data[0] = command_list[i]
            res = self.to_int_array(parameter_list[i],4)

            for ii in range (1,5):
                send.Data[ii] = res[ii-1]
                m_run0 = 1
                ret=0

            if canDLL.VCI_Transmit(VCI_USBCAN2,0,0,byref(send),1) == 1:
                #print(f"CAN1 TX ID: 0x{send.ID:08X}",end="")
                #print("CAN1 TX ID: 0x{:08X}".format(send.ID))
    
                #if send.ExternFlag == 0:
                    #print(" Standard ")
                #if send.ExternFlag == 1:
                    #print(" Extend   ")
                #if send.RemoteFlag == 0:
                    #print(" Data     ")
                #if send.RemoteFlag == 1:
                    #print(" Remote   ")

                #print(f"DLC: 0x{send.DataLen:02X}",end=" data: 0x")
                #print("DLC: 0x{:02X}".format(send.DataLen)),
                #print("data: 0x")

                #for iii in range(send.DataLen):
                    #print(f" {send.Data[iii]:02X}",end="")
                    #print(" {:02X}".format(send.Data[iii]))

                #print()
                send.ID += 1 
            else:
                break

        print("send_can_command over")



    def callback(self,msg):
        n=len(msg.trajectory[0].joint_trajectory.points)
        cn = [0]*10
        ori_position = [0]*10
        next_position= [0]*10
        now_position = [0]*10


        tmp0=100
        tmp1=-100
        parameterlist222 = [tmp0] *10
        parameterlist2222 = [tmp1] *10
    
        canidlist222 = [1,2,3,4,5,6]
        cmd222 = [36,36,36,36,36,36]

        self.send_can_command(6,canidlist222,cmd222,parameterlist222)
        time.sleep(100/1e6)
        cmd2222=[37,37,37,37,37,37]
        self.send_can_command(6,canidlist222,cmd2222,parameterlist2222)
        time.sleep(100/1e6)

        ori_position[0]=msg.trajectory[0].joint_trajectory.points[0].positions[0]*57.3
        ori_position[1]=msg.trajectory[0].joint_trajectory.points[0].positions[1]*57.3
        ori_position[2]=msg.trajectory[0].joint_trajectory.points[0].positions[2]*57.3
        ori_position[3]=msg.trajectory[0].joint_trajectory.points[0].positions[3]*57.3
        ori_position[4]=msg.trajectory[0].joint_trajectory.points[0].positions[4]*57.3
        ori_position[5]=msg.trajectory[0].joint_trajectory.points[0].positions[5]*57.3
    
        next_position[0]=msg.trajectory[0].joint_trajectory.points[n-1].positions[0]*57.3
        next_position[1]=msg.trajectory[0].joint_trajectory.points[n-1].positions[1]*57.3
        next_position[2]=msg.trajectory[0].joint_trajectory.points[n-1].positions[2]*57.3
        next_position[3]=msg.trajectory[0].joint_trajectory.points[n-1].positions[3]*57.3
        next_position[4]=msg.trajectory[0].joint_trajectory.points[n-1].positions[4]*57.3
        next_position[5]=msg.trajectory[0].joint_trajectory.points[n-1].positions[5]*57.3

        now_position[0]=-(msg.trajectory[0].joint_trajectory.points[n-1].positions[0])*57.3*101*65536/360
        now_position[1]=-(msg.trajectory[0].joint_trajectory.points[n-1].positions[1])*57.3*101*65536/360
        now_position[2]=-(msg.trajectory[0].joint_trajectory.points[n-1].positions[2])*57.3*101*65536/360
        now_position[3]=msg.trajectory[0].joint_trajectory.points[n-1].positions[3]*57.3*101*65536/360
        now_position[4]=-(msg.trajectory[0].joint_trajectory.points[n-1].positions[4])*57.3*101*65536/360
        now_position[5]=msg.trajectory[0].joint_trajectory.points[n-1].positions[5]*57.3*101*65536/360

        #print(f"{now_position[0]}**{now_position[1]}**{now_position[2]}**{now_position[3]}**{now_position[4]}**{now_position[5]}")

        cn[0]=next_position[0]-ori_position[0]
        cn[1]=next_position[1]-ori_position[1]
        cn[2]=next_position[2]-ori_position[2]
        cn[3]=next_position[3]-ori_position[3]
        cn[4]=next_position[4]-ori_position[4]
        cn[5]=next_position[5]-ori_position[5]
    
        parameterlist2 = [0]*10
        for i in range(0,6):
            parameterlist2[i] = int(now_position[i])

        maxVal = self.find_max(cn)
        print("maxVal: ",maxVal)
        periodTime = maxVal / 130
        status = 1
        print("asadwejfhjehrg      "+str(periodTime))
    
        if 0.3 < periodTime <0.5:
            periodTime *= 1
        elif 0.001 < periodTime < 0.3:
            periodTime *= 3
        
        print("TTTTTTT",periodTime)
        maxSpeed = [0]*10
        for i in range(0,6):
            maxSpeed[i] = abs((cn[i]/periodTime)*10100/360)
    
        print("##",maxSpeed)

        canidlist2=[1,2,3,4,5,6]
        cmd2 = [30,30,30,30,30,30]
        self.send_can_command(6,canidlist2,cmd2,parameterlist2)

        for j in range(10,105):
            acceleration_ratio = (j-5) /100.0
            tmp02=[acceleration_ratio*speed for speed in maxSpeed]
            tmp12=[-tmp for tmp in tmp02]
            parameterlist222b = [int(val) for val in tmp02]
            parameterlist2222b = [int(val) for val in tmp12]

            canidlist222b=[1,2,3,4,5,6]
            cmd222b=[36,36,36,36,36,36]
            self.send_can_command(6,canidlist222b,cmd222b,parameterlist222b)

            time.sleep(10/1e6)
            cmd2222b=[37,37,37,37,37,37]
            self.send_can_command(6,canidlist222b,cmd2222b,parameterlist2222b)

            time.sleep(10/1e6)
            time.sleep((5000*status)/1e6)

        time.sleep((periodTime  - 0.42 * status))
    
        for j in range(105,5,-1):
            deceleration_ratio = (105 - j) /100.0
            tmp02= [j*speed //100 for speed in maxSpeed]
            tmp12=[-tmp for tmp in tmp02]
            parameterlist222b = [int(val) for val in tmp02]
            parameterlist2222b = [int(val) for val in tmp12]
            
            canidlist222b=[1,2,3,4,5,6]
            cmd222b=[36,36,36,36,36,36]
            self.send_can_command(6,canidlist222b,cmd222b,parameterlist222b)

            time.sleep(10/1e6)

            cmd2222b=[37,37,37,37,37,37]
            self.send_can_command(6,canidlist222b,cmd2222b,parameterlist2222b)

            time.sleep(10/1e6)

            time.sleep((5000*status*deceleration_ratio)/1e6)



    def move_by_joint(self, joint_group_positions):
            self.group.set_joint_value_target(joint_group_positions)
            plan=self.group.plan()
            print("123654789")
            if plan:
                self.group.execute()
                time.sleep(3)
                return True
            else:
                print("Failed to plan a trajectory")
                return False


    def move_joint(self, joint_group_positions):
        current_joint_values = self.group.get_current_joint_values()
        for i in range(len(joint_group_positions)):
            current_joint_values[i] += joint_group_positions[i]

        self.group.set_joint_value_target(current_joint_values)
        plan=self.group.plan()
        print("789654123")
        if plan:
            self.group.execute()
            time.sleep(3)
            return True
        else:
            print("Failed to plan a trajectory")
            return False




    def move_by_pos(self, pose):
        target_pose = Pose()
        target_pose.position.x = pose[0]
        target_pose.position.y = pose[1]
        target_pose.position.z = pose[2]

        quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.set_start_state_to_current_state()
        self.set_pose_target(target_pose)

        plan = self.plan()
        rospy.loginfo("move_p: %s", "SUCCESS" if plan[1] else "FAILED")

        if plan[1]:
            self.execute(plan[0])
            rospy.sleep(3)
            return True
        return False


    def get_pos(self):
        current_pose = self.get_current_pose(self.end_effector_link)
        rospy.loginfo("current pose: x:%f, y:%f, z:%f, Roll:%f, Pitch:%f, Yaw:%f",
                      current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                      current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                      current_pose.pose.orientation.z, current_pose.pose.orientation.w)

    def get_joint(self):
        current_joint_values = self.get_current_joint_values()
        rospy.loginfo("current joint values: %f, %f, %f, %f, %f, %f",
                      current_joint_values[0], current_joint_values[1], current_joint_values[2],
                      current_joint_values[3], current_joint_values[4], current_joint_values[5])



def main():
    
        #rospy.init_node('client_node',anonymous=True)

        #rospy.Subscriber('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,callback)
        aha=Ti5_py()
        for i in range (1,10):
            j=[0.2,0.4,-0.3,0.6,1,1.5]
            aha.move_by_joint(j)
            aha.get_joint()
            aha.get_pos()
            print("qwerty")

            jj=[-0.2,-0.4,0.3,-0.6,-1,-1.5]
            aha.move_joint(jj)
            aha.get_joint()
            aha.get_pos()
            print("asdfg")

            aha.chang_v(1500)
            print("zxcvb")

            yzrpy=[-2.36174548954e-06,-8.31433908653e-05,0.436347686674,-5.8718139674278595e-05, -5.824219744912165e-05, 6.269113268998034e-05]
            aha.move_by_pos(xyzrpy)
            aha.get_joint()
            aha.get_pos()
            print("mnbv")

            aha.chang_v(4000)
            print("poiuy")

        #rospy.spin()



if __name__=="__main__":
    main()


