#include<Ti5robot/Ti5robot.h>
#include<ros/ros.h>

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
	my.init_serial("/dev/ttyUSB0",115200);

        ros::Subscriber client_sub=nh.subscribe("/move_group/display_planned_path",1000,&Ti5robot::callback,&my);

	vector<double> z={0.5, -0.2, 0.6, -0.4, 1, -0.8};
        my.move_by_joint(z);
        my.get_joint();
        my.get_pos();

	my.get_electric();

	int sock=my.init_udp(8088);

	my.udp_read(sock);

	

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

        return 0;
	*/
}

