// serial_demo.cpp
// serial_demo.cpp
// serial_demo.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>

#include "std_msgs/String.h"

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <iostream>
#include <controlcan.h>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdint>
#define MESSAGE_FREQ 100
#define MESSAGE_FREQ_END 60

double next_velocitie[6] = {0};

double ori_velocitie[6] = {0};
double cha_velocitie[6] = {0};
double now_velocitie[6] = {0};

double next_position[6] = {0};

double ori_position[6] = {0};
double cha_position[6] = {0};
int now_position[6] = {0};
char buffer[5120];

int r1[500] = {0};
int r2[500] = {0};
int r3[500] = {0};

int r4[500] = {0};
int r5[500] = {0};
int r6[500] = {0};


uint32_t pose[10];

class Listener
{
	private:
		char topic_message[2048] = {0};

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



std::int32_t convertHexArrayToDecimal(const std::uint8_t hexArray[4])
{
	std::int32_t result = 0;

	for (int i = 0; i < 4; i++)
		result = (result << 8) | hexArray[i];

	if (result > 0x7FFFFFFF) 
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

int32_t sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, int Command)
{
	VCI_CAN_OBJ send[1];
	send[0].DataLen = 1;

	for (int i = 0; i < numOfActuator; i++)
	{
		send[0].ID=canIdList[i];
		send[0].Data[0] = Command; 
		pthread_t threadid;

		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			printf("CAN1 TX ID:0x%08X", send[0].ID);
			std::cout<<std::endl;

			VCI_CAN_OBJ rec[3000]; 
			int ind = 0,reclen=0;

			reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100);
			if(reclen==0) 
			{
				for(int c=0;c<10;c++)
					reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100);
			}
			//std::cout<<"reclen:  "<<reclen<<std::endl;

			if (reclen > 0) 
			{
				for (int j = 0; j < reclen; j++)
				{
					std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
					std::int32_t decimal = convertHexArrayToDecimal(hexArray);
					pose[i+1]=decimal;
					//std::cout << "Decimal:         Decimal   " << decimal << std::endl;
				}
			}
		}
		else
			break;
	}
}



int32_t sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
	int Command = commandList[0]; 
	VCI_CAN_OBJ send[1];
	send[0].ID = 1;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 5;
	int i = 0;
	for (i = 0; i < 6; i++)
	{
		send[0].Data[0] = Command; // 位置环30
		//std::cout << "ID:" << i << "   " << parameterList[i] << std::endl;
		int res[4];
		toIntArray(parameterList[i], res, 4);
		int i_back = 4;
		for (int iiii = 1; iiii < 5; iiii++)
			send[0].Data[iiii] = res[iiii - 1];

		int m_run0 = 1;
		pthread_t threadid;
		int ret;

		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{

			printf("CAN1 TX ID:0x%08X", send[0].ID);
			if (send[0].ExternFlag == 0)
				printf(" Standard ");
			if (send[0].ExternFlag == 1)
				printf(" Extend   ");
			if (send[0].RemoteFlag == 0)
				printf(" Data   ");
			if (send[0].RemoteFlag == 1)
				printf(" Remote ");
			printf("DLC:0x%02X", send[0].DataLen);
			printf(" data:0x");

			for (int iii = 0; iii < send[0].DataLen; iii++)
			{
				printf(" %02X", send[0].Data[iii]);
			}
			//printf("ORI:%02X", parameterList[i]);

			printf("\n");
			send[0].ID += 1;
		}
		else
			break;
	}
}





void moveRobotToTarget(double start_position[6], double target_position[6]) 
{
	double cn[6];

	int tmp0 = 100;
	int tmp1 = -100;
	uint32_t parameterlist222[10] = {tmp0, tmp0, tmp0, tmp0, tmp0, tmp0};
	uint32_t parameterlist2222[10] = {tmp1, tmp1, tmp1, tmp1, tmp1, tmp1};
	uint8_t canidlist222[10] = {1, 2, 3, 4, 5, 6}, cmd222[10] = {36, 36, 36, 36, 36, 36};
	sendCanCommand(6, canidlist222, cmd222, parameterlist222);
	usleep(100);
	uint8_t cmd2222[10] = {37, 37, 37, 37, 37, 37};
	sendCanCommand(6, canidlist222, cmd2222, parameterlist2222);
	usleep(100);


	for (int i = 0; i < 6; ++i) 
		cn[i] = target_position[i]*57.3 - start_position[i]*57.3;


	now_position[0] = target_position[0] *57.3*101*65536/360;
	now_position[1] = target_position[1] *57.3*101*65536/360;
	now_position[2] = target_position[2] *57.3*101*65536/360;
	now_position[3] = target_position[3] *57.3*101*65536/360;
	now_position[4] = target_position[4] *57.3*101*65536/360;
	now_position[5] = target_position[5] *57.3*101*65536/360;

	uint32_t parameterlists2[10]={0};

	for(size_t i = 0; i < 6; ++i) 
		parameterlists2[i] = static_cast<uint32_t>(now_position[i]);


	double maxVal = findMax(cn, 6); 
	std::cout<<"~~~~~~"<<maxVal<<std::endl;
	double periodTime =maxVal/130;
	double status=1;

	if (0.3<periodTime<0.6)
		periodTime *=2;
	else if (0<periodTime<0.3)
		periodTime *=3;

	//double periodTime =maxVal/170;
	std::cout<<"TTTTTTT"<<periodTime<<std::endl;
	int maxSpeed[6];
	maxSpeed[0]=abs((cn[0]/periodTime)*10100/360);
	maxSpeed[1]=abs((cn[1]/periodTime)*10100/360);
	maxSpeed[2]=abs((cn[2]/periodTime)*10100/360);
	maxSpeed[3]=abs((cn[3]/periodTime)*10100/360);
	maxSpeed[4]=abs((cn[4]/periodTime)*10100/360);
	maxSpeed[5]=abs((cn[5]/periodTime)*10100/360);

	std::cout<<"##"<<maxSpeed[0]<<"##"<<maxSpeed[1]<<"##"<<maxSpeed[2]<<"##"<<maxSpeed[3]<<"##"<<maxSpeed[4]<<std::endl;

	//get max speed split 
	uint8_t canidlist2[10] = {1, 2, 3, 4, 5, 6}, cmd2[10] = {30, 30, 30, 30, 30, 30};
	sendCanCommand(6, canidlist2, cmd2, parameterlists2);



	//send speed and target
	uint32_t parameterlist2221[10] = {maxSpeed[0],maxSpeed[1],maxSpeed[2],maxSpeed[3],maxSpeed[4],maxSpeed[5]};
	//uint32_t parameterlist22221[10] = {-maxSpeed[0],-maxSpeed[1],-maxSpeed[2],-maxSpeed[3],-maxSpeed[4],-maxSpeed[5]};
	uint32_t parameterlist22221[10] = {maxSpeed[0],maxSpeed[1],maxSpeed[2],maxSpeed[3],maxSpeed[4],maxSpeed[5]};
	sendCanCommand(6, canidlist222, cmd222, parameterlist2221);
	usleep(100);
	sendCanCommand(6, canidlist222, cmd2222, parameterlist22221);
	usleep(50);


	for (int j=10; j<105; j++) 
	{
		double acceleration_ratio = (j - 5) / 100.0;  
		int tmp02[6] = {0};
		tmp02[0] = acceleration_ratio * maxSpeed[0];
		tmp02[1] = acceleration_ratio * maxSpeed[1];
		tmp02[2] = acceleration_ratio * maxSpeed[2];
		tmp02[3] = acceleration_ratio * maxSpeed[3];
		tmp02[4] = acceleration_ratio * maxSpeed[4];
		tmp02[5] = acceleration_ratio * maxSpeed[5];

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t parameterlist222b[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t parameterlist2222b[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		uint8_t canidlist222b[10] = {1, 2, 3, 4, 5, 6}, cmd222b[10] = {36, 36, 36, 36, 36, 36};
		sendCanCommand(6, canidlist222b, cmd222b, parameterlist222b);

		usleep(10);

		uint8_t cmd2222b[10] = {37, 37, 37, 37, 37, 37};
		sendCanCommand(6, canidlist222b, cmd2222b, parameterlist2222b);

		usleep(10);

		usleep(5000 * status);
	}


	//run maxspeed with periodTime-200000
	usleep(periodTime*1000000-580000*status);

	//cn unit is du
	for (int j = 105; j > 5; j--) 
	{
		double deceleration_ratio = (105 - j) / 100.0;  
		int tmp02[6] = {0};
		tmp02[0] = j * maxSpeed[0] / 100;
		tmp02[1] = j * maxSpeed[1] / 100;
		tmp02[2] = j * maxSpeed[2] / 100;
		tmp02[3] = j * maxSpeed[3] / 100;
		tmp02[4] = j * maxSpeed[4] / 100;
		tmp02[5] = j * maxSpeed[5] / 100;

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t parameterlist222b[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t parameterlist2222b[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		uint8_t canidlist222b[10] = {1, 2, 3, 4, 5, 6}, cmd222b[10] = {36, 36, 36, 36, 36, 36};
		sendCanCommand(6, canidlist222b, cmd222b, parameterlist222b);

		usleep(10);

		uint8_t cmd2222b[10] = {37, 37, 37, 37, 37, 37};
		sendCanCommand(6, canidlist222b, cmd2222b, parameterlist2222b);

		usleep(10);

		usleep(5000 * status * deceleration_ratio);
	}


	std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<std::endl;
}




void Listener::callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
	int n = msg->trajectory[0].joint_trajectory.points.size();
	double cn[6];
	int t[6];


	int tmp0=100;
	int tmp1=-100;
	uint32_t parameterlist222[10] = {tmp0,tmp0,tmp0,tmp0,tmp0,tmp0};
	uint32_t parameterlist2222[10] = {tmp1,tmp1,tmp1,tmp1,tmp1,tmp1};
	uint8_t canidlist222[10] = {1,2, 3,4,5,6}, cmd222[10] = {36,36,36,36,36, 36};
	sendCanCommand(6, canidlist222, cmd222, parameterlist222);
	usleep(100);
	uint8_t cmd2222[10] = {37,37,37,37,37,37};
	sendCanCommand(6, canidlist222, cmd2222, parameterlist2222);
	usleep(100);



	ori_position[0] = msg->trajectory[0].joint_trajectory.points[0].positions[0]*57.3;
	ori_position[1] = msg->trajectory[0].joint_trajectory.points[0].positions[1] *57.3;
	ori_position[2] = msg->trajectory[0].joint_trajectory.points[0].positions[2] *57.3;
	ori_position[3] = msg->trajectory[0].joint_trajectory.points[0].positions[3] *57.3;
	ori_position[4] = msg->trajectory[0].joint_trajectory.points[0].positions[4] *57.3;
	ori_position[5] = msg->trajectory[0].joint_trajectory.points[0].positions[5] *57.3;
	next_position[0] = msg->trajectory[0].joint_trajectory.points[n-1].positions[0] *57.3;
	next_position[1] = msg->trajectory[0].joint_trajectory.points[n-1].positions[1] *57.3;
	next_position[2] = msg->trajectory[0].joint_trajectory.points[n-1].positions[2] *57.3;
	next_position[3] = msg->trajectory[0].joint_trajectory.points[n-1].positions[3] *57.3;
	next_position[4] = msg->trajectory[0].joint_trajectory.points[n-1].positions[4] *57.3;
	next_position[5] = msg->trajectory[0].joint_trajectory.points[n-1].positions[5] *57.3;

	now_position[0] = -(msg->trajectory[0].joint_trajectory.points[n-1].positions[0]) *57.3*101*65536/360;
	now_position[1] = -(msg->trajectory[0].joint_trajectory.points[n-1].positions[1]) *57.3*101*65536/360;
	now_position[2] = -(msg->trajectory[0].joint_trajectory.points[n-1].positions[2]) *57.3*101*65536/360;
	now_position[3] = msg->trajectory[0].joint_trajectory.points[n-1].positions[3] *57.3*101*65536/360;
	now_position[4] = -(msg->trajectory[0].joint_trajectory.points[n-1].positions[4]) *57.3*101*65536/360;
	now_position[5] = msg->trajectory[0].joint_trajectory.points[n-1].positions[5] *57.3*101*65536/360;
	std::cout<<now_position[0]<<"**"<<now_position[1]<<"**"<<now_position[2]<<"**"<<now_position[3]<<"**"<<now_position[4]<<std::endl;

	cn[0] = next_position[0] - ori_position[0];
	cn[1] = next_position[1] - ori_position[1];
	cn[2] = next_position[2] - ori_position[2];
	cn[3] = next_position[3] - ori_position[3];
	cn[4] = next_position[4] - ori_position[4];
	cn[5] = next_position[5] - ori_position[5];




	uint32_t parameterlists2[10]={0};

	for(size_t i = 0; i < 6; ++i) 
		parameterlists2[i] = static_cast<uint32_t>(now_position[i]); 


	//ready control max speed
	double maxVal = findMax(cn, 6);
	std::cout<<"~~~~~~"<<maxVal<<std::endl;
	double periodTime =maxVal/100;
	double status=1;
	if (0.3<periodTime<0.6)
		periodTime *=4;
	else if (0<periodTime<0.3)
		periodTime *=6;



	//double periodTime =maxVal/170;
	std::cout<<"TTTTTTT"<<periodTime<<std::endl;
	int maxSpeed[6];
	maxSpeed[0]=abs((cn[0]/periodTime)*10100/360);
	maxSpeed[1]=abs((cn[1]/periodTime)*10100/360);
	maxSpeed[2]=abs((cn[2]/periodTime)*10100/360);
	maxSpeed[3]=abs((cn[3]/periodTime)*10100/360);
	maxSpeed[4]=abs((cn[4]/periodTime)*10100/360);
	maxSpeed[5]=abs((cn[5]/periodTime)*10100/360);

	std::cout<<"##"<<maxSpeed[0]<<"##"<<maxSpeed[1]<<"##"<<maxSpeed[2]<<"##"<<maxSpeed[3]<<"##"<<maxSpeed[4]<<std::endl;

	//get max speed split 
	uint8_t canidlist2[10] = {1, 2, 3, 4, 5, 6}, cmd2[10] = {30, 30, 30, 30, 30, 30};
	sendCanCommand(6, canidlist2, cmd2, parameterlists2);



	//send speed and target
	uint32_t parameterlist2221[10] = {maxSpeed[0],maxSpeed[1],maxSpeed[2],maxSpeed[3],maxSpeed[4],maxSpeed[5]};
	//uint32_t parameterlist22221[10] = {-maxSpeed[0],-maxSpeed[1],-maxSpeed[2],-maxSpeed[3],-maxSpeed[4],-maxSpeed[5]};
	uint32_t parameterlist22221[10] = {maxSpeed[0],maxSpeed[1],maxSpeed[2],maxSpeed[3],maxSpeed[4],maxSpeed[5]};
	sendCanCommand(6, canidlist222, cmd222, parameterlist2221);
	usleep(100);
	sendCanCommand(6, canidlist222, cmd2222, parameterlist22221);
	usleep(50);


	for (int j=10; j<105; j++) 
	{
		double acceleration_ratio = (j - 5) / 100.0;  
		int tmp02[6] = {0};
		tmp02[0] = acceleration_ratio * maxSpeed[0];
		tmp02[1] = acceleration_ratio * maxSpeed[1];
		tmp02[2] = acceleration_ratio * maxSpeed[2];
		tmp02[3] = acceleration_ratio * maxSpeed[3];
		tmp02[4] = acceleration_ratio * maxSpeed[4];
		tmp02[5] = acceleration_ratio * maxSpeed[5];

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t parameterlist222b[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t parameterlist2222b[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		uint8_t canidlist222b[10] = {1, 2, 3, 4, 5, 6}, cmd222b[10] = {36, 36, 36, 36, 36, 36};
		sendCanCommand(6, canidlist222b, cmd222b, parameterlist222b);

		usleep(10);

		uint8_t cmd2222b[10] = {37, 37, 37, 37, 37, 37};
		sendCanCommand(6, canidlist222b, cmd2222b, parameterlist2222b);

		usleep(10);

		usleep(5000 * status);
	}

	usleep(periodTime*1000000-550000*status);

	//cn unit is du
	for (int j = 105; j > 5; j--) 
	{
		double deceleration_ratio = (105 - j) / 100.0;  
		int tmp02[6] = {0};
		tmp02[0] = j * maxSpeed[0] / 100;
		tmp02[1] = j * maxSpeed[1] / 100;
		tmp02[2] = j * maxSpeed[2] / 100;
		tmp02[3] = j * maxSpeed[3] / 100;
		tmp02[4] = j * maxSpeed[4] / 100;
		tmp02[5] = j * maxSpeed[5] / 100;

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t parameterlist222b[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t parameterlist2222b[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		uint8_t canidlist222b[10] = {1, 2, 3, 4, 5, 6}, cmd222b[10] = {36, 36, 36, 36, 36, 36};
		sendCanCommand(6, canidlist222b, cmd222b, parameterlist222b);

		usleep(10);

		uint8_t cmd2222b[10] = {37, 37, 37, 37, 37, 37};
		sendCanCommand(6, canidlist222b, cmd2222b, parameterlist2222b);

		usleep(10);

		usleep(5000 * status * deceleration_ratio);
	}

	std::cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"<<std::endl;

}

bool init_can()
{
	VCI_BOARD_INFO pInfo; 
	int count = 0;        
	VCI_BOARD_INFO pInfo1[50];
	int num = 0;
	fflush(stdin);
	fflush(stdout);
	printf(">>this is hello !\r\n"); 
	num = VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");
	printf("%d", num);
	printf(" PCS");
	printf("\n");
	int nDeviceType = 4;
	int nDeviceInd = 0;  
	int nCANInd = 0;    
	DWORD dwRel;
	VCI_INIT_CONFIG vic;
	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
	if (dwRel != 1)
	{
		std::cout << "open-fail:" << dwRel << std::endl;
		return FALSE;
	}
	else
		std::cout << "open success:1";

	vic.AccCode = 0x80000008;
	vic.AccMask = 0xFFFFFFFF;
	vic.Filter = 1;
	vic.Timing0 = 0x00;
	vic.Timing1 = 0x14;
	vic.Mode = 0;
	dwRel = VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vic);
	if (dwRel != 1)
	{
		std::cout << "init-fail:" << dwRel << std::endl;
		VCI_CloseDevice(nDeviceType, nDeviceInd);

		return FALSE;
	}
	else
		std::cout << "initsuccess:" << dwRel << std::endl;

	if (VCI_StartCAN(VCI_USBCAN2, 0, 0) != 1)
	{
		std::cout << "start-fail:" << dwRel << std::endl;
		VCI_CloseDevice(VCI_USBCAN2, 0);
	}
	else
		std::cout << "startsuccess:1" << std::endl;
}


std::string get_time()
{
	std::time_t now =std::time(nullptr);
	std::tm localTime=*std::localtime(&now);
	std::ostringstream oss;
	oss << std::put_time(&localTime,"%Y-%m-%d %H:%M:%S");
	return oss.str();
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
	uint32_t parameterlist222[10] = {tmp0,tmp0,tmp0,tmp0,tmp0,tmp0};
	uint32_t parameterlist2222[10] = {tmp1,tmp1,tmp1,tmp1,tmp1,tmp1};
	uint8_t canidlist222[10] = {1,2, 3,4,5,6}, cmd222[10] = {36,36,36,36,36, 36};
	sendCanCommand(6, canidlist222, cmd222, parameterlist222);
	usleep(50);
	uint8_t cmd2222[10] = {37,37,37,37,37,37};
	sendCanCommand(6, canidlist222, cmd2222, parameterlist2222);
	usleep(50);


	uint32_t parameterlists2[10]={0};
	uint8_t canidlist2[10] = {1, 2, 3, 4, 5, 6}, cmd2[10] = {30, 30, 30, 30, 30, 30};
	sendCanCommand(6, canidlist2, cmd2, parameterlists2);

	usleep(8000000);
	for (int i=0;i<5;i++)
	{

		ori_position[0]=0;
		ori_position[1]=0;
		ori_position[2]=0;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=0;
		next_position[2]=0;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		//moveRobotToTarget(ori_position,next_position);

		ori_position[0]=0;
		ori_position[1]=0;
		ori_position[2]=0;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=0;
		next_position[2]=1.9;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=0;
		ori_position[1]=0;
		ori_position[2]=1.9;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=0;
		next_position[2]=0.6;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);


		//1

		ori_position[0]=0;
		ori_position[1]=0;
		ori_position[2]=0.6;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=1.3;
		next_position[2]=1.9;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=0;
		ori_position[1]=1.3;
		ori_position[2]=1.9;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=0;
		next_position[2]=0.6;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		//2

		ori_position[0]=0;
		ori_position[1]=0;
		ori_position[2]=0.6;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0.5;
		next_position[1]=0;
		next_position[2]=1.9;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=0.5;
		ori_position[1]=0;
		ori_position[2]=1.9;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=-0.5;
		next_position[1]=0;
		next_position[2]=0.6;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=-0.5;
		ori_position[1]=0;
		ori_position[2]=0.6;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0.5;
		next_position[1]=0;
		next_position[2]=1.9;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=0.5;
		ori_position[1]=0;
		ori_position[2]=1.9;
		ori_position[3]=0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=-1.5;
		next_position[1]=0;
		next_position[2]=1.9;
		next_position[3]=1.0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=-1.5;
		ori_position[1]=0;
		ori_position[2]=1.9;
		ori_position[3]=1.0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=1.5;
		next_position[1]=0;
		next_position[2]=1.9;
		next_position[3]=-1.0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);

		ori_position[0]=1.5;
		ori_position[1]=0;
		ori_position[2]=1.9;
		ori_position[3]=-1.0;
		ori_position[4]=0;
		ori_position[5]=0;

		next_position[0]=0;
		next_position[1]=0;
		next_position[2]=0;
		next_position[3]=0;
		next_position[4]=0;
		next_position[5]=0;
		moveRobotToTarget(ori_position,next_position);
	}

	uint8_t canidlist[10]={1,2,3,4,5,6};

	int32_t res=sendSimpleCanCommand(6,canidlist,8);

	std::ofstream outfile("/home/ti5robot/info.txt",std::ios_base::app);

	if(outfile.is_open())
	{
		std::stringstream ss;
		ss << "[" << get_time() <<"]"<< "ID 1: "<<pose[1]<<",ID 2: "<<pose[2]<<",ID 3: "<<pose[3]<<",ID 4: "<<pose[4]<<",ID 5: "<<pose[5]<<",ID 6: "<<pose[6];
		//ROS_INFO_STREAM(ss.str());

		outfile << ss.str() <<std::endl;

		outfile.close();
	}


	while (ros::ok())
	{
		
		ros::spinOnce();
		loop_rate.sleep();
		uint8_t canidlist2[10] = {1, 2, 3, 4, 5, 6}, cmd2[10] = {30, 30, 30, 30, 30, 30};

		//sendSimpleCanCommand(6, canidlist2, 8);
	}


	return 0;
}
