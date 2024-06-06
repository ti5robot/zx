// serial_demo.cpp
/*
该程序为使用三个CAN分析仪，共6个CAN通道来控制机械臂
	*/
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <controlcan.h>
#include <iostream>
#include <cstdint>
#include <chrono>

double next_velocitie[6] = {0};

double ori_velocitie[6] = {0};
double now_velocitie[6] = {0};

double next_position[6] = {0};

double ori_position[6] = {0};
int now_position[6] = {0};
char buffer[5120];

class Listener
{
public:
    // void callback(const sensor_msgs::JointState::ConstPtr &msg);
    void callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);
    char *getMessageValue();
};

double findMax(double arr[], int size) 
{
	double max = abs(arr[0]); 
	for (int i = 1; i < size-2; i++) 
	{
		if (abs(arr[i]) > abs(max))  
			max = abs(arr[i]); 
        }	
	return max;
}


int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4])
{
	std::int32_t result=0;
	for(int i=0;i<4;i++)
		result=(result << 8) | hexArray[i];
	if(result>0x7FFFFFFF)
		result -= 0x100000000;

	return result;
}


void toIntArray(int number, int *res, int size)
{
	unsigned int unsignedNumber = static_cast<unsigned int>(number);
	for (int i = 0; i < size; ++i)
	{
		res[i] = unsignedNumber & 0xFF; 
		unsignedNumber >>= 8;           
	}
}


void sendSimpleCanCommand(int canDevice, int nCANInd, uint8_t canId, uint8_t command)
{
        VCI_ClearBuffer(4,canDevice,nCANInd);

        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 1;

        send[0].ID = canId;
        send[0].Data[0] = command;
        if (VCI_Transmit(VCI_USBCAN2, canDevice, nCANInd, send, 1) == 1)
	{
		int cnt = 3;
		int reclen = 0;
		VCI_CAN_OBJ rec[3000];

		while((reclen = VCI_Receive(VCI_USBCAN2, canDevice, nCANInd, rec, 3000, 0)) <= 0 && cnt) cnt--;
		if(cnt==0) std::cout<<"ops! canDevice: "<<canDevice<<"  nCANInd: "<<nCANInd<<"  ID: "<<canId<<" failed after try 3 times."<<std::endl;
		else
		{
			std::cout<<"reclen reclen:  "<<reclen<<std::endl;
			for (int j = 0; j < reclen; j++)
			{
				std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
				std::int32_t decimal = convertHexArrayToDecimal(hexArray);
				std::cout <<"ID: "<<rec[j].ID<<"       data: " << decimal << std::endl;
			}
		}
	}
	else
		std::cout<<"aaaaa! canid:  "<<canId<<"  VCI_Transmit failed."<<std::endl;
}


void sendCanCommand(int canDevice, int nCANInd, uint8_t canId, uint8_t command, uint32_t parameter)
{
	VCI_ClearBuffer(4,canDevice,nCANInd);

        VCI_CAN_OBJ send[1];
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        send[0].ExternFlag = 0;
        send[0].DataLen = 5;
      
	send[0].ID = canId;
        send[0].Data[0] = command;
        int res[4];
	toIntArray(parameter, res, 4);
	for (int j = 1; j < 5; j++)
		send[0].Data[j] = res[j - 1];

	if (VCI_Transmit(VCI_USBCAN2, canDevice, nCANInd, send, 1) == 1)
	{
		std::cout<<" canDevice: "<<canDevice<<"  nCANInd: "<<nCANInd<<"  ID: "<<send[0].ID<<std::endl;
		for (int cnt = 0; cnt < send[0].DataLen; cnt++)
			//printf("");   
			printf(" %02X",send[0].Data[cnt]);
		std::cout<<std::endl;
	}
	else
		std::cout<<"aaaaa! canid: "<<canId<<"   VCI_Transmit failed."<<std::endl;
}




void Listener::callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
	int n = msg->trajectory[0].joint_trajectory.points.size();
	double cn[6];

	uint8_t canidlist[10]={1,2,3,4,5,6};
	//uint8_t cmd_pos[10]={30,30,30,30,30,30},cmd_v_p[10]={36,36,36,36,36,36},cmd_v_n[10]={37,37,37,37,37,37};
	uint32_t para_v_p[10],para_v_n[10],para_pos[10];



	uint8_t canid_1=1, canid_2=2, canid_3=3,canid_4=4,canid_5=5,canid_6=6;
	uint8_t cmd_v_p=36,cmd_v_n=37,cmd_pos=30;
	uint32_t para_v_p_1=100,para_v_p_2=100,para_v_p_3=100,para_v_p_4=100,para_v_p_5=100,para_v_p_6=100;
	uint32_t para_v_n_1=-100,para_v_n_2=-100,para_v_n_3=-100,para_v_n_4=-100,para_v_n_5=-100,para_v_n_6=-100;


        int tmp0=100,tmp1=-100;

	for(int i=0;i<6;i++)
	{
		para_v_p[i]=100;
		para_v_n[i]=-100;
	}

	for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2,canidlist[i],cmd_v_p,para_v_p[i]);
	usleep(100);

	for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2,canidlist[i],cmd_v_n,para_v_n[i]);
	usleep(100);

	for(int i=0;i<6;i++)
	{
		ori_position[i] = msg->trajectory[0].joint_trajectory.points[0].positions[i]*57.3;
		next_position[i] = msg->trajectory[0].joint_trajectory.points[n-1].positions[i] *57.3;
		now_position[i] = (msg->trajectory[0].joint_trajectory.points[n-1].positions[i]) *57.3*101*65536/360;
		cn[i]=next_position[i]-ori_position[i];
		para_pos[i]=static_cast<uint32_t>(now_position[i]);
	}

        double maxVal = findMax(cn, 6);
	std::cout<<"~~~~~~"<<maxVal<<std::endl;
	double periodTime =maxVal/100;
	double status=1;
	if (0.3<periodTime<0.5)
		periodTime *=2;
	else if(0.001<periodTime<0.3)
		periodTime *=3;
	std::cout<<"TTTTTTT"<<periodTime<<std::endl;


	int maxSpeed[6];
	for(int i=0;i<6;i++)
		maxSpeed[i]=abs((cn[i]/periodTime)*10100/360);
	std::cout<<"##"<<maxSpeed[0]<<"##"<<maxSpeed[1]<<"##"<<maxSpeed[2]<<"##"<<maxSpeed[3]<<"##"<<maxSpeed[4]<<std::endl;

	for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2, canidlist[i], cmd_pos, para_pos[i]);


	for (int j=10; j<105; j++) 
	{
		double acceleration_ratio = (j - 5) / 100.0;
   
		for(int i=0;i<6;i++)
		{
			para_v_p[i]=acceleration_ratio*maxSpeed[i];
			para_v_n[i]=-1*para_v_p[i];

		}
		for(int i=0;i<6;i++)
			sendCanCommand(i/2,i%2, canidlist[i], cmd_v_p, para_v_p[i]);
    		usleep(10);
		for(int i=0;i<6;i++)
			sendCanCommand(i/2,i%2, canidlist[i], cmd_v_n, para_v_n[i]);
    		usleep(10);
    		usleep(5000 * status);
	}

	if(periodTime >= 0.55)
		usleep(periodTime*1000000-550000*status);


	for (int j = 105; j > 5; j--) 
	{
		double deceleration_ratio = (105 - j) / 100.0; 

		for(int i=0;i<6;i++)
		{
			para_v_p[i]=j*maxSpeed[i]/100;
			para_v_n[i]=-1*para_v_p[i];
		}
		for(int i=0;i<6;i++)
			sendCanCommand(i/2,i%2, canidlist[i], cmd_v_p, para_v_p[i]);
    		usleep(10);
		for(int i=0;i<6;i++)
			sendCanCommand(i/2,i%2, canidlist[i], cmd_v_n, para_v_n[i]);
    		usleep(10);
    		usleep(5000 * status * deceleration_ratio);
	}

	std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<std::endl;
}


bool init_can()
{
        VCI_BOARD_INFO pInfo, pInfo1[50];
        int count = 0, num = 0;
        printf(">>this is hello !\r\n");
        num = VCI_FindUsbDevice2(pInfo1);
        printf(">>USBCAN DEVICE NUM:   %d PCS\n",num);

        int nDeviceType = 4;
        int Device_1 = 0,Device_2 = 1,Device_3 = 2;
        int CAN_1 = 0, CAN_2 = 1;

	VCI_INIT_CONFIG vic;
        vic.AccCode = 0x80000008;
        vic.AccMask = 0xFFFFFFFF;
        vic.Filter = 1;
        vic.Timing0 = 0x00;
        vic.Timing1 = 0x14;
        vic.Mode = 0;


	// device_1
        if (VCI_OpenDevice(nDeviceType, Device_1, 0) != 1)
	{
                std::cout << "aaaaaaa open device 1 fail"<< std::endl;
		//return false;
	}
        else
                std::cout << "hhhhhhh open device 1 success"<<std::endl;

	if (VCI_InitCAN(nDeviceType, Device_1, CAN_1, &vic) != 1)
        {
                std::cout << "aaaaaa device_1 init CAN_1 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_1);
        }
        else
                std::cout << "hhhhhh device_1 init CAN_1 success." << std::endl;

        if (VCI_InitCAN(nDeviceType, Device_1, CAN_2, &vic) != 1)
        {
                std::cout << "aaaaaa device 1 init CAN_2 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_1);
        }
        else
                std::cout << "hhhhhh device_1 init CAN_2 success." << std::endl;

	if (VCI_StartCAN(VCI_USBCAN2, Device_1, CAN_1) != 1)
        {
                std::cout << "aaaaaa device_1 start CAN_1 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_1);
        }
        else
                std::cout << "hhhhhh device_1 start CAN_1 success." << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, Device_1, CAN_2) != 1)
        {
                std::cout << "aaaaaa device_1 start CAN_2 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_1);
        }
        else
                std::cout << "hhhhhh device_1 start CAN_2 success." << std::endl;

	// device_2
	if (VCI_OpenDevice(nDeviceType, Device_2, 0) != 1)
	{
                std::cout << "aaaaaaa open device 2 fail"<< std::endl;
		//return false;
	}
        else
                std::cout << "hhhhhhh open device 2 success"<<std::endl;

        if (VCI_InitCAN(nDeviceType, Device_2, CAN_1, &vic) != 1)
        {
                std::cout << "aaaaaa device_2 init CAN_1 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_2);
        }
        else
                std::cout << "hhhhhh device_2 init CAN_1 success." << std::endl;

        if (VCI_InitCAN(nDeviceType, Device_2, CAN_2, &vic) != 1)
        {
                std::cout << "aaaaaa device 2 init CAN_2 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_2);
        }
        else
                std::cout << "hhhhhh device_2 init CAN_2 success." << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, Device_2, CAN_1) != 1)
        {
                std::cout << "aaaaaa device_2 start CAN_1 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_2);
        }
        else
                std::cout << "hhhhhh device_2 start CAN_1 success." << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, Device_2, CAN_2) != 1)
        {
                std::cout << "aaaaaa device_2 start CAN_2 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_2);
        }
        else
                std::cout << "hhhhhh device_2 start CAN_2 success." << std::endl;

	// device_3
	if (VCI_OpenDevice(nDeviceType, Device_3, 0) != 1)
	{
                std::cout << "aaaaaaa open device 3 fail"<< std::endl;
		//return false;
	}
        else
                std::cout << "hhhhhhh open device 3 success"<<std::endl;

        if (VCI_InitCAN(nDeviceType, Device_3, CAN_1, &vic) != 1)
        {
                std::cout << "aaaaaa device_3 init CAN_1 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_3);
        }
        else
                std::cout << "hhhhhh device_3 init CAN_1 success." << std::endl;

        if (VCI_InitCAN(nDeviceType, Device_3, CAN_2, &vic) != 1)
        {
                std::cout << "aaaaaa device 3 init CAN_2 fail." << std::endl;
                VCI_CloseDevice(nDeviceType, Device_3);
        }
        else
                std::cout << "hhhhhh device_3 init CAN_2 success." << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, Device_3, CAN_1) != 1)
        {
                std::cout << "aaaaaa device_3 start CAN_1 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_3);
        }
        else
                std::cout << "hhhhhh device_3 start CAN_1 success." << std::endl;

        if (VCI_StartCAN(VCI_USBCAN2, Device_3, CAN_2) != 1)
        {
                std::cout << "aaaaaa device_3 start CAN_2 failed." << std::endl;
                VCI_CloseDevice(VCI_USBCAN2, Device_3);
        }
        else
        	std::cout << "hhhhhh device_3 start CAN_2 success." << std::endl;


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "client_node");
    	ros::NodeHandle nh;
    	ros::Rate loop_rate(500);
    	Listener listener;
    	ros::Subscriber client_sub = nh.subscribe("/move_group/display_planned_path", 1000, &Listener::callback, &listener);

    	init_can(); 

        int tmp0=500;
        int tmp1=-500;
        uint32_t para_v_p[10] = {tmp0,tmp0,tmp0,tmp0,tmp0,tmp0},para_v_n[10] = {tmp1,tmp1,tmp1,tmp1,tmp1,tmp1};
        uint8_t canidlist[10] = {1,2,3,4,5,6};
	uint8_t canid_1=1,canid_2=2,canid_3=3,canid_4=4,canid_5=5,canid_6=6;
	uint8_t cmd_v_p=36,cmd_v_n=37,cmd_pos=30;
	//uint8_t cmd_v_p[10] = {36,36,36,36,36,36}, cmd_v_n[10]={37,37,37,37,37,37},cmd_pos[10]={30,30,30,30,30,30};
	for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2,canidlist[i], cmd_v_p, para_v_p[i]);
        usleep(50);
        for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2,canidlist[i], cmd_v_n, para_v_n[i]);
        usleep(50);


	uint32_t para_pos[10]={0,0,0,0,0,0};
	for(int i=0;i<6;i++)
		sendCanCommand(i/2,i%2,canidlist[i], cmd_pos, para_pos[i]);
	/*
		i/2 的取值为0,1,2，对应于CAN的三个设备,i%2的取值为0,1，对应于CAN设备的CAN1，CAN2通道
	*/
	usleep(5000000);
	int num=10000;
	while (ros::ok())
	{
        	ros::spinOnce();
        	loop_rate.sleep();
        	uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd_get_pos=8;

		auto start = std::chrono::high_resolution_clock::now();
		while(num--)
		{
			for(int i=0;i<6;i++)
				sendSimpleCanCommand(i/2,i%2,canidlist[i],cmd_get_pos);
		}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> execution_time = end - start;
		std::cout<<"Function execution time:  "<<execution_time.count() <<"   seconds"<<std::endl;
		return 0;
	}
    	return 0;
}
