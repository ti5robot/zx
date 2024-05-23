#include <iostream>
#include <string>
#include <unistd.h>
#include "controlcan/controlcan.h"

/*
export LD_LIBRARY_PATH=/home/ti5/controlcan:$LD_LIBRARY_PATH
g++ test.cpp -o test -Lcontrolcan -lcontrolcan
 */


bool init_can()
{
	VCI_BOARD_INFO pInfo, pInfo1[50];
	int count = 0, num = 0;
	printf(">>this is hello !\r\n");
	num = VCI_FindUsbDevice2(pInfo1);
	printf(">>USBCAN DEVICE NUM:");
	printf("%d PCS\n", num);
	int nDeviceType = 4;
	int nDeviceInd = 0;
	int nCANInd = 0;
	DWORD dwRel;
	VCI_INIT_CONFIG vic;
	dwRel = VCI_OpenDevice(nDeviceType, nDeviceInd, 0);
	if (dwRel != 1)
	{
		std::cout << "open-fail:" << dwRel << std::endl;
		return false;
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
		return false;
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


int32_t convertHexArrayToDecimal(const uint8_t hexArray[4])
{
	int32_t result = 0;
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


void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList)
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;

	for (int i = 0; i < numOfActuator; i++)
	{
		send[0].ID = i+1;
		send[0].Data[0] = commandList[i];
		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			int cnt = 5;
			int reclen = 0,ind=0;
			VCI_CAN_OBJ rec[3000];

			while((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) <= 0 && cnt) cnt--;
			if(cnt==0) std::cout<<"ops! ID "<<send[0].ID<<" failed after try 5 times."<<std::endl;
			else
			{	
				for (int j = 0; j < reclen; j++)
				{
					std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};

					std::int32_t decimal = convertHexArrayToDecimal(hexArray);
					std::cout <<"ID: "<<send[0].ID<<"       data: " << decimal << std::endl;
				}
			}

		}
		else
			break;
	}
}





void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
	VCI_CAN_OBJ send[1];
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 5;
	for (int i = 0; i < numOfActuator; i++)
	{
		send[0].ID = i+1;
		send[0].Data[0] = commandList[i];
		int res[4];
		toIntArray(parameterList[i], res, 4);
		for (int j = 1; j < 5; j++)
			send[0].Data[j] = res[j - 1];

		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			for (int cnt = 0; cnt < send[0].DataLen; cnt++)
				//printf("");	
				printf(" %02X",send[0].Data[cnt]);
			std::cout<<std::endl;
		}
		else
			break;
	}
}




int main()
{
	init_can();

	uint8_t canidlist[10]={1},cmd_pos[10]={30},cmd_get_pos[10]={8};
	uint32_t pos[10] = {static_cast<uint32_t>(int(4000000))};
	int cnt=10;
	while(cnt--)
	{
		if(cnt%2)
			pos[0]={static_cast<uint32_t>(int(4000000))};
		else 
			pos[0]={static_cast<uint32_t>(int(-4000000))};

		sendCanCommand(1,canidlist,cmd_pos,pos);
		sleep(1);
	}

	//sendSimpleCanCommand(1,canidlist,cmd_get_pos);



	return 0;
}



