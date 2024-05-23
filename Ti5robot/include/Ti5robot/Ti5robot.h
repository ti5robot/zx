#ifndef TI5ROBOT_H
#define TI5ROBOT_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string>
#include <iostream>
#include <termio.h>
#include "controlcan.h"
#include <csignal>
#include <cstdlib>
#include <thread>
#include <atomic>
#include <cstdint>
#include <serial/serial.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <typeinfo>
using namespace std;


class Ti5robot
{
public:
    Ti5robot(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP);

    bool init_can();
    bool init_serial(const std::string& port,int baudrate);
    void read_ser();
    int init_udp(int port);
    bool udp_read(int sock);
    bool write_ser(std::string& data);
    uint16_t calc_crc(const std::vector<uint8_t>& data,size_t length);
    std::vector<uint8_t> hexstring_uint8(const std::string& hexstring);

    int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);
    void toIntArray(int number, int *res, int size);

    std::vector<int> sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList);
    std::vector<int> sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
    void callback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
    

    std::vector<int> get_elec_pos();
    void clean_error();
    std::vector<int> get_error();
    void get_electric();
    void change_v(int v_);
    void move_up();

/*
    void move_by_keys();
    bool move_joint_by_key(int joint,double cnt);
    bool move_pos_by_key(int joint,double cnt);
*/
    bool move_by_joint(const vector<double> &joint_group_positions);
    bool move_joint(const vector<double> &joint_group_positions);
    bool move_by_pos(const vector<double> &pose);
    //bool move_line(const vector<double> &pose);
    //bool move_line(const vector<vector<double>> &posees);


    bool test_joint(const vector<double> &pose);
    void get_pos();
    void get_joint();
    char get_key();
    ~Ti5robot();

public:
    std::atomic<bool> running{true};
    int v = 130;
    string reference_frame;
    string end_effector_link;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *arm_;
    serial::Serial ser;
};

#endif
