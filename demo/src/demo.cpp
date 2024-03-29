#include<Ti5robot/Ti5robot.h>
#include<ros/ros.h>


#include<serial/serial.h>
#include<std_msgs/String.h>

#include<string>
#include<sstream>

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <chrono>  // 添加这行以包含 std::chrono 头文件
#include <thread>  // 添加这行以包含 std::this_thread 头文件

#include <geometry_msgs/Twist.h>

#include <cstdlib>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#define KEYCODE_1 0x61
#define KEYCODE_2 0x73
#define KEYCODE_3 0x64
#define KEYCODE_4 0x66
#define KEYCODE_5 0x67
#define KEYCODE_6 0x68


#define KEYCODE_21 0x7a
#define KEYCODE_22 0x78
#define KEYCODE_23 0x63
#define KEYCODE_24 0x76
#define KEYCODE_25 0x62
#define KEYCODE_26 0x6e

#define KEYCODE_27 0x6a
#define KEYCODE_28 0x6b
#define KEYCODE_29 0x6c

using namespace std;


static const uint16_t crc16tab[256]= {
        0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
        0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
        0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
        0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
        0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
        0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
        0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
        0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
        0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
        0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
        0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
        0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
        0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
        0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
        0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
        0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
        0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
        0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
        0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
        0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
        0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
        0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
        0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
        0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
        0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
        0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
        0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
        0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
        0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
        0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
        0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
        0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};


uint16_t calculateCRC16(uint8_t *data, size_t length) 
{
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < length; ++i) 
                crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ data[i]) & 0xFF];

        return crc;
}


std::string convertToString(const uint8_t data[], size_t size)
{
        std::string result;
        result.reserve(size);

        for (size_t i = 0; i < size; ++i) 
                result.push_back(static_cast<char>(data[i]));

        return result;
}

float convert_to_float(uint16_t high_byte, uint16_t low_byte) 
{
        std::cout<<high_byte<<low_byte<<std::endl;
        uint16_t combined = (high_byte << 8) | low_byte;

        int16_t signed_val = static_cast<int16_t>(combined);

        return static_cast<float>(signed_val) / 32768.0f;
}


void ProcessReceivedData(const std::array<uint8_t, 16>& buffer,Ti5robot& my) 
{
/*
        ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);

        spinner.start();
        static const std::string PLANNING_GROUP = "armgroup";
        moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
        Ti5robot my(nh, arm, PLANNING_GROUP);

        ros::Subscriber client_sub=nh.subscribe("/move_group/display_planned_path",1000,&Ti5robot::callback,&my);
*/



        if (buffer[0] != 0xAA || buffer[1] != 0x01)
                return;

        int16_t x = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
        int16_t y = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
        int16_t z = static_cast<int16_t>((buffer[6] << 8) | buffer[7]);
        int16_t roll = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
        int16_t pitch = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
        int16_t yaw = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);


        vector<double> pos={x/10000.0, y/10000.0, z/10000.0, roll/10000.0, pitch/10000.0, yaw/10000.0};
        //pos={0.061,-0.0472,0.4645,0,0,0};
        my.move_by_pos(pos);
        my.get_joint();
        my.get_pos();
	std::cout<<"!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        std::cout << "X: " << x << ", Y: " << y << ", Z: " << z
                << ", Roll: " << roll << ", Pitch: " << pitch
                << ", Yaw: " << yaw << std::endl;
}


int serial_write(serial::Serial &ser, std::string &serial_msg) 
{
        try {
                ser.write(serial_msg);
                return 0;
        } catch (const serial::SerialException& e) {
                ROS_ERROR_STREAM("Error while writing to serial: " << e.what());
                return -1;
        }
}


int serial_read(serial::Serial &ser, std::string &result) 
{
        auto start_time = std::chrono::steady_clock::now();

        int bytesRead = 0;
        while (bytesRead < 16) {


                if (ser.available()) {
                        result += ser.read(1);
                        bytesRead++;
                        start_time = std::chrono::steady_clock::now(); 
                } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                        auto current_time = std::chrono::steady_clock::now();
                        auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
                        if (elapsed_time >= 2000) {
                                result = "bbbb"; 
                                break;
                        }

                }
        }
}



int main(int argc, char **argv)
{


	ros::init(argc, argv,"pr2_base_keyboard");
        ros::NodeHandle n_;

        ros::Publisher vel_pub_;
        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        serial::Serial ser;
        geometry_msgs::Twist cmd;




	//ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);

        spinner.start();
        static const std::string PLANNING_GROUP = "armgroup";
        moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
        Ti5robot my(n_, arm, PLANNING_GROUP);

        ros::Subscriber client_sub=n_.subscribe("/move_group/display_planned_path",1000,&Ti5robot::callback,&my);



        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        bool dirty=false;
        ser.setPort("/dev/pts/2");        
        ser.setBaudrate(115200);               
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);        
        ser.setTimeout(to);                       

   
        try
        {
                ser.open();        
        }
        catch(const std::exception& e)
        {
                ROS_ERROR_STREAM("Unable to open port ");         
                return -1;
        }


        if( ser.isOpen() )
        {
                ROS_INFO_STREAM("Serial Port initialized. \n");         
        }
        else
        {
                ROS_ERROR_STREAM("Serial port is not open. Exiting...");
                return -1;
        }


        ros::Rate loop_rate(50);

    
        int func(0);
        while( ros::ok() )
        {
                std::string  resultTmp;
                std::string yesno;
                std::vector<uint8_t> data;

                serial_read(ser, resultTmp);

		std::cout<<"result:  "<<resultTmp<<std::endl;

                std::array<unsigned char, 16> arr;

                if (resultTmp.size() <= arr.size()) {
                        std::copy_n(resultTmp.begin(), resultTmp.size(), arr.begin());
                        std::fill(arr.begin() + resultTmp.size(), arr.end(), 0);

			for(int i=0;i<arr.size();i++)
				std::cout<<"i:  "<<arr[i]<<std::endl;

                        //ProcessReceivedData(arr,my);
                } else {
                        std::cout<<"11111"<<std::endl;
                }



	}
        return 0;
}

