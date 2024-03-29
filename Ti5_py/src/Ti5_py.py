#!/usr/bin/env python

#coding:utf-8

import rospy
import tf_conversions
import geometry_msgs.msg
import moveit_msgs.msg
import time
import serial
import socket
import binascii
import math
import threading
import ctypes
import struct
from ctypes import *
import numpy as np
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory
import moveit_commander 
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


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
v=130

ser = serial.Serial()

crc16tab = [
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
        0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
        0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
        0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
        0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
        0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
        0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
        0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
        0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
        0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
        0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
        0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
        0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
        0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
        0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
        0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0]



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


    def init_serial(self,port,baudrate):
        try:
            ser.port = port
            ser.baudrate = baudrate
            ser.timeout = 1
            ser.open()
        except Exception as e:
            print("Failed to open serial port: ",e)

        if ser.isOpen():
            print("serial port initialized")
        else:
            print("Failed to open serial port.")


    def get_sign16(self,vx):
        if not vx or vx <0x8000:
            return vx
        return vx - 0x10000


    def read_ser(self):
        while True:
            result = ""
            cnt = 0
            while cnt < 16:
                if ser.in_waiting:
                    result += ser.read(2)
                    cnt += 1
            print("result:", result)  
    
            bytes_list = []
            for i in range(0,len(result),2):
                byte = int(result[i:i+2],16)
                bytes_list.append(byte)

            if bytes_list[0] != 0xAA:
                print("Error: Frame header mismatch!")

            if bytes_list[1] == 0xFF or bytes_list[1] == 0xff:
                print("End of transmission received.")
                break

            if len(bytes_list) != 16:
                print("Error: Invalid data length received!")
            
            z = self.get_sign16(bytes_list[2] << 8 | bytes_list[3])
            x = self.get_sign16(bytes_list[4] << 8 | bytes_list[5])
            c = self.get_sign16(bytes_list[6] << 8 | bytes_list[7])
            v = self.get_sign16(bytes_list[8] << 8 | bytes_list[9])
            b = self.get_sign16(bytes_list[10] << 8 | bytes_list[11])
            n = self.get_sign16(bytes_list[12] << 8 | bytes_list[13])
            crc = (bytes_list[14] << 8 | bytes_list[15])
            
            print("z:  ",z, "x:  ",x, "c:  ",c, "v:  ",v, "b:  ",b, "n:  ",n, "crc:  ",crc)
            nnn = self.calc_crc(bytes_list,14)
            print("calc_crc:  ",nnn)

            if crc == self.calc_crc(bytes_list,14):
                res = [z/10000.0,x/10000.0,c/10000.0,v/10000.0,b/10000.0,n/10000.0]
                
                if bytes_list[1] == 0x01:
                    self.move_by_pos(res)
                elif bytes_list[1] == 0x00:
                    self.move_by_joint(res)
                elif bytes_list[1] == 0xFF or bytes_list[1] == 0xff:
                    break
                r = self.get_error()
		data = ""
                for i in r:
                    data += str(i)
		data += "  "
                self.write_ser(data)
                self.get_pos()
                self.get_joint()
            else:
                print("CRC verification error, please check and send again.")
    

    def write_ser(self,data):
        try:
            ser.write(data.encode())
        except serial.SerialException as e:
            print("Failed to write to port: ",e)
            return False
        return True

    def calc_crc(self,data,length):
        crc = 0xFFFF
        for i in range(length):
            crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ data[i]) & 0xFF]
        crc &= 0xFFFF
        return crc


    '''
    def init_udp(self,ip,port):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((ip,port))
        print("udp socket bound to {ip}  port: {port}".format(ip=ip,port=port))
        return sock
    '''
    def init_udp(self,port):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server_addr = ('',port)

        try:
            sock.bind(server_addr)
            print("udp socket bound to port: {}".format(port))
        except Exception as e:
            print("Failed to bind socket: ",e)
            sock.close()
        return sock


    def udp_read(self,sock):
        while True:
            message, client_addr = sock.recvfrom(1024)
            client_ip,client_port = client_addr
            
            print("received message from {client_ip} : {client_port} : {message}".format(client_ip = client_ip,client_port = client_port, message = message))
    
            bytes_list = []
            for i in range(0,32,2):
                byte = int(message[i:i+2],16)
                bytes_list.append(byte)

            #bytes_list = list(message)
            
            if bytes_list[0] != 0xAA:
                print("Error: Frame header mismatch!")

            if bytes_list[1] == 0xFF or bytes_list[1] == 0xff:
                print("End of transmission received.")
                break

            if len(bytes_list) != 16:
                print("Error: Invalid data length received!")

            z = self.get_sign16(bytes_list[2] << 8 | bytes_list[3])
            x = self.get_sign16(bytes_list[4] << 8 | bytes_list[5])
            c = self.get_sign16(bytes_list[6] << 8 | bytes_list[7])
            v = self.get_sign16(bytes_list[8] << 8 | bytes_list[9])
            b = self.get_sign16(bytes_list[10] << 8 | bytes_list[11])
            n = self.get_sign16(bytes_list[12] << 8 | bytes_list[13])
            crc = (bytes_list[14] << 8 | bytes_list[15])

            print("z:  ",z, "x:  ",x, "c:  ",c, "v:  ",v, "b:  ",b, "n:  ",n, "crc:  ",crc)
            nnn = self.calc_crc(bytes_list,14)
            print("calc_crc:  ",nnn)

            if crc == self.calc_crc(bytes_list,14):
                res = [z/10000.0,x/10000.0,c/10000.0,v/10000.0,b/10000.0,n/10000.0]

                if bytes_list[1] == 0x01:
                    self.move_by_pos(res)
                elif bytes_list[1] == 0x00:
                    self.move_by_joint(res)
                elif bytes_list[1] == 0xFF or bytes_list[1] == 0xff:
                    break
                r = self.get_error()
                data = ""
                for i in r:
                    data += str(i)
                data += "  "
                sock.sendto(data,client_addr)

            else:
                print("CRC verification error, please check and send again.")





    def find_max(self,arr):
        max_val=math.fabs(arr[0])
        for i in range(1,6):
            if math.fabs(arr[i]) > math.fabs(max_val):
                max_val = math.fabs(arr[i])
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
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 1
	res = []
        for i in range(0,num_of_actuator):

            send.ID = i+1
            send.Data[0] = command

            if canDLL.VCI_Transmit(VCI_USBCAN2,0,0,byref(send),1) == 1:
                rec = VCI_CAN_OBJ_ARRAY(3000)
		ind=0
                cnt = 5
		
		reclen=canDLL.VCI_Receive(VCI_USBCAN2,0,0,byref(rec.ADDR),3000,0)
                while reclen<=0 and cnt>0:
		    if reclen<=0:
			reclen=canDLL.VCI_Receive(VCI_USBCAN2,0,0,byref(rec.ADDR),3000,0)
                    cnt=cnt-1
                if cnt==0:
                    print("ops! ID %d failed after try 5 times!",send.ID)

                else:
                    for j in range(reclen):
			data_array = [rec.STRUCT_ARRAY[j].Data[4],rec.STRUCT_ARRAY[j].Data[3],rec.STRUCT_ARRAY[j].Data[2],rec.STRUCT_ARRAY[j].Data[1]]
			data_list = [data_array[i] for i in range(len(data_array))]
			decimal = self.convert_hex_array_to_decimal(data_list)
			res.append(decimal)
                        print("ID: {}\tdata: {}".format(send.ID,decimal))

            else:
                break
	return res


    def send_can_command(self,num_of_actuator,can_id_list,command_list,parameter_list):
        global canDLL
        global VCI_USBCAN2
        global STATUS_OK
        send = VCI_CAN_OBJ()
    
        send.SendType = 0
        send.RemoteFlag = 0
        send.ExternFlag = 0
        send.DataLen = 5
    
        for i in range(num_of_actuator):
            send.ID = i+1
            send.Data[0] = command_list[i]
            res = self.to_int_array(parameter_list[i],4)

            for ii in range (1,5):
                send.Data[ii] = res[ii-1]
                ret=0
            
	    canDLL.VCI_Transmit(VCI_USBCAN2,0,0,byref(send),1)


    def callback(self,msg):
        global v
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

        now_position[0]=msg.trajectory[0].joint_trajectory.points[n-1].positions[0]*57.3*101*65536/360
        now_position[1]=msg.trajectory[0].joint_trajectory.points[n-1].positions[1]*57.3*101*65536/360
        now_position[2]=msg.trajectory[0].joint_trajectory.points[n-1].positions[2]*57.3*101*65536/360
        now_position[3]=msg.trajectory[0].joint_trajectory.points[n-1].positions[3]*57.3*101*65536/360
        now_position[4]=msg.trajectory[0].joint_trajectory.points[n-1].positions[4]*57.3*101*65536/360
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
	print(type(maxVal))
        periodTime = float(maxVal) / v
        #print("vvvvvvvvvv:  ",v)
        status = 1
    
        if 0.3 < periodTime <0.5:
            periodTime *= 1
        elif 0.001 < periodTime < 0.3:
            periodTime *= 3
        
        print("TTTTTTT",periodTime)
        maxSpeed = [0]*10
        for i in range(0,6):
            maxSpeed[i] = abs((cn[i]/periodTime)*10100/360)
        #print("##",maxSpeed)

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

        time.sleep((periodTime  - 0.42 * status) if (periodTime -0.42*status) > 0 else 0)
    
        for j in range(105,5,-1):
            deceleration_ratio = (105 - j) /100.0
            tmp02= [j*speed /100 for speed in maxSpeed]
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
	    '''
	    current_joint_values = self.group.get_current_joint_values()
            for i in range(len(joint_group_positions)):
            	current_joint_values[i] = joint_group_positions[i]
	    group.go(current_joint_values,wait=True)
	    '''
            self.group.set_joint_value_target(joint_group_positions)
	    self.group.go()
            plan=self.group.plan()
            if plan:
                self.group.execute(plan)
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
        if plan:
            self.group.execute(plan)
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

        self.group.set_pose_target(target_pose)

        plan = self.group.plan()

        success=self.group.execute(plan)
        rospy.loginfo(success)
        rospy.sleep(3)


    def get_pos(self):
        current_pose = self.group.get_current_pose(self.end_effector_link)
        print("Current Pose:")
        print("{}".format(current_pose.pose))

    def get_joint(self):
        current_joint_values = self.group.get_current_joint_values()
        print("Current Joint Values:")
        print("{}".format(current_joint_values))


    def change_v(self,v_):
        global v
        v = v_/1000*35.6

    def clean_error(self):
        canidlist=[1,2,3,4,5,6]
        cmd=11
        self.send_simple_can_command(6,canidlist,cmd)

    def get_error(self):
        canidlist=[1,2,3,4,5,6]
        cmd=10
        res=self.send_simple_can_command(6,canidlist,cmd)
        return res

    def get_electric(self):
        canidlist=[1,2,3,4,5,6]
        cmd=4
        self.send_simple_can_command(6,canidlist,cmd)

    def get_elec_pos(self):
	canidlist=[1,2,3,4,5,6]
	cmd=8
	res=self.send_simple_can_command(6,canidlist,cmd)
	return res

    def test_joint(self,pose):
	self.move_by_pos(pose)
	res=self.get_elec_pos()
	current_joint_values = self.group.get_current_joint_values()
	jj = []

	f=True
	for i in range(len(res)):
	    jj.append(float(res[i]) / 100 /65536 * 2 *math.pi)
	    print("i: {}  elec_joint: {}   rviz_joint: {}	{}".format(i,jj[i],current_joint_values[i],abs(jj[i]-current_joint_values[i])))
	    if abs(jj[i]-current_joint_values[i]) > 0.1:
		f=False

	return f
	
    '''
    def move_line(self, pose):
        waypoints = []
        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = pose[:3]
        quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = quaternion
        waypoints.append((target_pose))
        self.group.set_pose_target(target_pose)

        (plan,fraction) = self.group.compute_cartesian_path(waypoints,0.01,0.0)
        self.group.execute(plan,wait=True)

        fraction = 0.0
        maxtries = 100
        attempts = 0
        
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
            attempts += 1

        if fraction == 1:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.group.execute(plan)
            rospy.sleep(3)
            return True
        else:
            rospy.loginfo("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries)
            return False
        


    
    def move_line_multiple(self, posees):
        waypoints = []
        for pose in posees:
            target_pose = Pose()
            target_pose.position.x, target_pose.position.y, target_pose.position.z = pose[:3]

            quaternion = quaternion_from_euler(pose[3], pose[4], pose[5])
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = quaternion
            waypoints.append(target_pose)

        fraction = 0.0
        maxtries = 100
        attempts = 0
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
            attempts += 1

        if fraction == 1:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.group.execute(plan)
            rospy.sleep(3)
            return True
        else:
            rospy.loginfo("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries)
            return False

    '''



def main():

        aha=Ti5_py()
'''
        for i in range (1,3):

            y=[0.0412209002559,-0.0298185348209,0.62690382276,0.004137735297700275, -0.6161636287739231, 1.242396596120268]
            aha.move_line(y)
            aha.get_joint()
            aha.get_pos()
            z=[-0.173417617143,-0.163760558971,0.565591849993,1.4371114278295734, -0.8407014559571303, -1.6823912888634394]
            aha.move_line(z)
            aha.get_joint()
            aha.get_pos()
            r=[-0.0524140828506,0.171464117433,0.437485756129,-1.9290747224189193, 0.4700067714405484, 0.8443787464323126]
            aha.move_line(r)
            aha.get_joint()
            aha.get_pos()

            xyzrpy=[[-2.36174548954e-06,-8.31433908653e-05,0.436347686674,-5.8718139674278595e-05, -5.824219744912165e-05, 6.269113268998034e-05],
                    [0.0412209002559,-0.0298185348209,0.62690382276,0.004137735297700275, -0.6161636287739231, 1.242396596120268],
                    [-0.173417617143,-0.163760558971,0.565591849993,1.4371114278295734, -0.8407014559571303, -1.6823912888634394],
                    [-0.0524140828506,0.171464117433,0.437485756129,-1.9290747224189193, 0.4700067714405484, 0.8443787464323126]]

            aha.move_line_multiple(xyzrpy)
            aha.get_joint()
            aha.get_pos()
'''

if __name__=="__main__":
    main()


