#include <Ti5robot/Ti5robot.h>
#include <ros/ros.h>


Ti5robot::Ti5robot(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP)
{
	init_can();
	this->arm_ = &arm;
	this->nh_ = nh;
	arm_->setGoalPositionTolerance(0.001);
	arm_->setGoalOrientationTolerance(0.01);
	arm_->setGoalJointTolerance(0.001);
	arm_->setMaxAccelerationScalingFactor(0.5);
	arm_->setMaxVelocityScalingFactor(0.5);

	const moveit::core::JointModelGroup *joint_model_group = arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	this->end_effector_link = arm_->getEndEffectorLink();
	this->reference_frame = "base_link";
	arm_->setPoseReferenceFrame(reference_frame);
	arm_->allowReplanning(true);
	arm_->setPlanningTime(5.0);
}


bool Ti5robot::init_can()
{
	VCI_BOARD_INFO pInfo, pInfo1[50];
	int count = 0, num = 0;
	fflush(stdin);
	fflush(stdout);
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


int32_t Ti5robot::convertHexArrayToDecimal(const uint8_t hexArray[4])
{
	int32_t result = 0;
	for (int i = 0; i < 4; i++)
		result = (result << 8) | hexArray[i];
	if (result > 0x7FFFFFFF)
		result -= 0x100000000;
	return result;
}


void Ti5robot::toIntArray(int number, int *res, int size)
{
	unsigned int unsignedNumber = static_cast<unsigned int>(number);
	for (int i = 0; i < size; ++i)
	{
		res[i] = unsignedNumber & 0xFF;
		unsignedNumber >>= 8;
	}
}


void Ti5robot::sendSimpleCanCommand(uint8_t numOfActuator, uint8_t *canIdList, int Command)
{
	VCI_CAN_OBJ send[1];
	send[0].ID = 1;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 1;
	for (int i = 0; i < 6; i++)
	{
		send[0].Data[0] = Command;
		pthread_t threadid;
		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			// printf("CAN1 TX ID:0x%08X", send[0].ID);
			if (send[0].ExternFlag == 0)
				// printf(" Standard ");
			if (send[0].ExternFlag == 1)
				// printf(" Extend   ");
			if (send[0].RemoteFlag == 0)
				// printf(" Data   ");
			if (send[0].RemoteFlag == 1)
				// printf(" Remote ");
				// printf("DLC:0x%02X", send[0].DataLen);
				// printf(" data:0x");

			for (int iii = 0; iii < send[0].DataLen; iii++)
				// printf(" %02X", send[0].Data[iii]);
				// printf("\n");
			int count = 0;
			int reclen = 0;
			VCI_CAN_OBJ rec[3000];
			int i, j, ind = 0;

			if ((reclen = VCI_Receive(VCI_USBCAN2, 0, ind, rec, 3000, 100)) > 0)
			{
				for (j = 0; j < reclen; j++)
				{
					std::uint8_t hexArray[4] = {rec[j].Data[4], rec[j].Data[3], rec[j].Data[2], rec[j].Data[1]};
					// std::cout << "ID: " << send[0].ID ;
					std::int32_t decimal = convertHexArrayToDecimal(hexArray);
					// std::cout << "Decimal: " << decimal << std::endl;
				}
			}
			send[0].ID += 1;
		}
		else
			break;
	}
}


void Ti5robot::sendCanCommand(uint8_t numOfActuator, uint8_t *canIdList, uint8_t *commandList, uint32_t *parameterList)
{
	VCI_CAN_OBJ send[1];
	send[0].ID = 1;
	send[0].SendType = 0;
	send[0].RemoteFlag = 0;
	send[0].ExternFlag = 0;
	send[0].DataLen = 5;
	for (int i = 0; i < 6; i++)
	{
		send[0].Data[0] = commandList[i];
		int res[4];
		toIntArray(parameterList[i], res, 4);
		int i_back = 4, m_run0 = 1, ret;
		for (int j = 1; j < 5; j++)
			send[0].Data[j] = res[j - 1];

		pthread_t threadid;
		if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1)
		{
			// printf("CAN1 TX ID:0x%08X",send[0].ID);
			printf("");
			if (send[0].ExternFlag == 0)
				printf("");
			// printf(" standard ");
			if (send[0].ExternFlag == 1)
				printf("");
			// printf(" Extend	");
			if (send[0].RemoteFlag == 0)
				printf("");
			// printf(" Remote	");
			if (send[0].RemoteFlag == 1)
				printf("");
			// printf(" Remote ");
			// printf("DLC:x%02X", send[0].DataLen);
			// printf(" data:0x");

			for (int cnt = 0; cnt < send[0].DataLen; cnt++)
				// printf(" %02X", send[0].Data[cnt]);
				printf("");
			// printf("\n");
			printf("");
			send[0].ID += 1;
		}
		else
			break;
	}
}


void Ti5robot::callback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
	int n = msg->trajectory[0].joint_trajectory.points.size();
	double cn[10] = {0}, ori_position[10] = {0}, next_position[10] = {0}, now_position[10] = {0}, maxVal = 0, status = 1;
	int maxSpeed[10] = {0};
	uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6}, cmd_pos[10] = {30, 30, 30, 30, 30, 30};
	uint8_t cmd_v_p[10] = {36, 36, 36, 36, 36, 36}, cmd_v_n[10] = {37, 37, 37, 37, 37, 37};
	uint32_t para_pos[10] = {0};

	int temp_p = 100, temp_n = -100;

	uint32_t para_v_p[10] = {temp_p, temp_p, temp_p, temp_p, temp_p, temp_p}, para_v_n[10] = {temp_n, temp_n, temp_n, temp_n, temp_n, temp_n};
	sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
	usleep(100);
	sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
	usleep(100);

	for (int cnt = 0; cnt < 6; cnt++)
	{
		ori_position[cnt] = msg->trajectory[0].joint_trajectory.points[0].positions[cnt] * 57.3;
		next_position[cnt] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[cnt] * 57.3;
		now_position[cnt] = msg->trajectory[0].joint_trajectory.points[n - 1].positions[cnt] * 57.3 * 101 * 65536 / 360;

		cn[cnt] = next_position[cnt] - ori_position[cnt];
		maxVal = maxVal < fabs(cn[cnt]) ? fabs(cn[cnt]) : maxVal;
	}
	para_pos[0] = static_cast<uint32_t>(int(-now_position[0]));
	para_pos[1] = static_cast<uint32_t>(int(-now_position[1]));
	para_pos[2] = static_cast<uint32_t>(int(-now_position[2]));
	para_pos[3] = static_cast<uint32_t>(int(now_position[3]));
	para_pos[4] = static_cast<uint32_t>(int(-now_position[4]));
	para_pos[5] = static_cast<uint32_t>(int(now_position[5]));

	double periodTime = maxVal / v;
	if (periodTime < 0.2)
		periodTime *= 3;
	else if (periodTime < 0.6)
		periodTime *= 2;

	for (int cnt = 0; cnt < 6; cnt++)
		maxSpeed[cnt] = abs((cn[cnt] / periodTime) * 10100 / 360);

	sendCanCommand(6, canidlist, cmd_pos, para_pos);

	for (int j = 10; j < 105; j++)
	{
		double acceleration_ratio = (j - 5) / 100.0;
		int tmp02[10] = {0};
		for (int cnt = 0; cnt < 6; cnt++)
			tmp02[cnt] = acceleration_ratio * maxSpeed[cnt];

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t para_v_p[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t para_v_n[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
		usleep(10);
		sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
		usleep(10);
		usleep(5000 * status);
	}

	usleep(periodTime * 1000000 - 550000 * status);

	for (int j = 105; j > 5; j--)
	{
		double deceleration_ratio = (105 - j) / 100.0;
		int tmp02[10] = {0};
		for (int cnt = 0; cnt < 6; cnt++)
			tmp02[cnt] = j * maxSpeed[cnt] / 100;

		int tmp12[6] = {-tmp02[0], -tmp02[1], -tmp02[2], -tmp02[3], -tmp02[4], -tmp02[5]};

		uint32_t para_v_p[10] = {tmp02[0], tmp02[1], tmp02[2], tmp02[3], tmp02[4], tmp02[5]};
		uint32_t para_v_n[10] = {tmp12[0], tmp12[1], tmp12[2], tmp12[3], tmp12[4], tmp12[5]};

		sendCanCommand(6, canidlist, cmd_v_p, para_v_p);
		usleep(10);
		sendCanCommand(6, canidlist, cmd_v_n, para_v_n);
		usleep(10);
		usleep(5000 * status * deceleration_ratio);
	}
}


void Ti5robot::clean_error()
{
	uint8_t canidlist[10] = {1, 2, 3, 4, 5, 6};
	sendSimpleCanCommand(6, canidlist, 11);
}


void Ti5robot::change_v(int v_)
{
	this->v = v_ / 1000 * 35.6;
}


bool Ti5robot::move_by_joint(const vector<double> &joint_group_positions)
{
	arm_->setJointValueTarget(joint_group_positions);
	arm_->move();
	sleep(3);
	return true;
}


bool Ti5robot::move_joint(const vector<double> &joint_group_positions)
{
	std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
	for (int i = 0; i < joint_group_positions.size(); i++)
		current_joint_values[i] += joint_group_positions[i];

	arm_->setJointValueTarget(current_joint_values);
	arm_->move();
	sleep(3);
	return true;
}


bool Ti5robot::move_by_pos(const vector<double> &pose)
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();

	arm_->setStartStateToCurrentState();
	arm_->setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);
	ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

	if (success)
	{
		arm_->execute(plan);
		sleep(3);
		return true;
	}
	return false;
}


/*
bool Ti5robot::move_line(const vector<double> &pose)
{
	vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();
	waypoints.push_back(target_pose);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;
	int attempts = 0;

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		arm_->execute(plan);
		sleep(3);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}


bool Ti5robot::move_line(const vector<vector<double>> &posees)
{
	vector<geometry_msgs::Pose> waypoints;
	for (int i = 0; i < posees.size(); i++)
	{
		geometry_msgs::Pose target_pose;
		target_pose.position.x = posees[i][0];
		target_pose.position.y = posees[i][1];
		target_pose.position.z = posees[i][2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		waypoints.push_back(target_pose);
	}

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;
	int attempts = 0;

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		arm_->execute(plan);
		sleep(3);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}
*/


void Ti5robot::get_pos()
{
	geometry_msgs::PoseStamped current_pose = this->arm_->getCurrentPose(this->end_effector_link);

	ROS_INFO("current pose:x:%f,y:%f,z:%f,Roll:%f,Pitch:%f,Yaw:%f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
			 current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);
}


void Ti5robot::get_joint()
{
	std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
	ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f", current_joint_values[0], current_joint_values[1], current_joint_values[2],
			 current_joint_values[3], current_joint_values[4], current_joint_values[5]);
}


Ti5robot::~Ti5robot()
{
	ros::shutdown();
}




