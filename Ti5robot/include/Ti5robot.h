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
#include </home/ti5robot/controlcan/controlcan.h>
using namespace std;


class Ti5robot
{
public:
    Ti5robot(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP);
    bool init_can();
    int32_t convertHexArrayToDecimal(const uint8_t hexArray[4]);
    void toIntArray(int number, int *res, int size);
    void sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, int Command);
    void sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList);
    void callback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);
    void clean_error();
    void get_error();
    void get_electric();
    void change_v(int v_);
    bool move_by_joint(const vector<double> &joint_group_positions);
    bool move_joint(const vector<double> &joint_group_positions);
    bool move_by_pos(const vector<double> &pose);
    //bool move_line(const vector<double> &pose);
    //bool move_line(const vector<vector<double>> &posees);
    void get_pos();
    void get_joint();
    ~Ti5robot();

public:
    int v = 130;
    string reference_frame;
    string end_effector_link;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface *arm_;
};

#endif
