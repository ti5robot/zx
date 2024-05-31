#include<Ti5robot/Ti5robot.h>
#include<ros/ros.h>
#include<chrono>


using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"cpp_demo");
	ros::NodeHandle nh;
        ros::AsyncSpinner spinner(1);

        spinner.start();
        static const std::string PLANNING_GROUP = "armgroup";
        moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
        Ti5robot my(nh, arm, PLANNING_GROUP);

        ros::Subscriber client_sub=nh.subscribe("/move_group/display_planned_path",1000,&Ti5robot::callback,&my);

/*	vector<double> z={0.5, -0.2, 0.6, -0.4, 1, -0.8};
        my.move_by_joint(z);
        my.get_joint();
        my.get_pos();

	my.get_electric();

	int sock=my.init_udp(8088);

	my.udp_read(sock);

	my.init_serial("/dev/ttyUSB0",115200);
*/	

	//my.read_ser();

	//my.write_ser();
	
	/*
	int cnt=10;
	while(cnt--)
	{
		vector<double> z={0.5, -0.2, 0.6, -0.4, 1, -0.8};
		my.move_by_joint(z);
		my.get_joint();
		my.get_pos();

		cout<<"get_error "<<endl;
		my.get_error();

		vector<double> xyz={-2.36174548954e-06,-8.31433908653e-05,0.436347686674,-5.8718139674278595e-05, -5.824219744912165e-05, 6.269113268998034e-05};
		my.move_by_pos(xyz);
		my.get_joint();
		my.get_pos();

		cout<<"get_electric "<<endl;
		my.get_electric();
	
		vector<double> x={-0.3, 0.8, 1.2, 1, 0.6, 0};
                my.move_by_joint(x);
                my.get_joint();
                my.get_pos();

		vector<double> ff;
		double zx;
		for(int i=0;i<6;i++)
		{
			cin>>zx;
			ff.push_back(zx);
		}
		my.test_joint(ff);
		
	}
*/

	
	auto start_1 = std::chrono::high_resolution_clock::now();
	vector<double> z_1={0.5, -0.2, 0.6, -0.4, 1, -0.8};
	my.move_joint_in_time(z_1,1);
	std::cout<<"1		1"<<std::endl;
	sleep(5);
	auto end_1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double,std::milli> duration_1 = end_1 - start_1;
	std::cout<<"execution time:  "<<duration_1.count() / 1000<<" ms"<<std::endl;


        vector<double> z_2={-0.1039,-0.3095,0.4529,2.9514,-1.5004,1.9819};
        my.move_pos_in_time(z_2,5);
	std::cout<<"2		5"<<std::endl;
	sleep(5);


        vector<double> z_3={0.027,-0.0209,0.5962,-1.423,-0.0258,0.4652};
	//my.move_by_joint(z_3);
        my.move_pos_in_time(z_3,1);
	std::cout<<"3		1"<<std::endl;
	sleep(5);


	vector<double> z_4={-0.035,0.1381,0.3881,-0.1284,-0.5335,2.300};
        my.move_pos_in_time(z_4,5);
	std::cout<<"4		5"<<std::endl;
	sleep(5);


	vector<double> z_5={-0.0713,-0.1532,0.3557,-0.8125,0.1335,2.7254};
        my.move_pos_in_time(z_5,1);
	std::cout<<"5		1"<<std::endl;
	sleep(5);


	vector<double> z_6={0.3207,-0.2149,0.3950,0.1404,0.0634,1.1333};
        my.move_pos_in_time(z_6,5);
	std::cout<<"6		5"<<std::endl;
	sleep(5);


	vector<double> z_7={-0.1397,-0.4231,0.2956,0.2554,-0.9548,-0.6582};
        my.move_pos_in_time(z_7,1);
	std::cout<<"7		1"<<std::endl;
	sleep(5);


	vector<double> z_8={0.09629,-0.0149,0.6153,3.024,1.1979,1.5950};
        my.move_pos_in_time(z_8,5);
	std::cout<<"8		5"<<std::endl;
	sleep(5);

	while(ros::ok())
	{
		ros::spinOnce();
	}

        return 0;
	
}

