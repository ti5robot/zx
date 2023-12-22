

# ROS接口使用说明（持续更新中）

项目github地址： [https://github.com/ti5robot/zx/tree/master](https://github.com/ti5robot/zx/tree/master).



![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/c8b7f6a53b334eceb53cdb307206ce2f.png#pic_center)
### 关于Ti5robot功能包介绍
Ti5robot功能包中主要实现了Ti5robot类，功能包中集成了函数具体实现的Ti5robot.cpp和相应头文件Ti5robot.h以及相应的ROS头文件。

 **Ti5robot.h**


 
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/984844a997c744de8074f14cbae112e2.png#pic_center)


#### 相应函数功能和参数说明

> **Ti5robot(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP);**
> 
> 该函数为TI5robot类的构造函数，ros::NodeHandle &nh 参数是一个ROS节点句柄的应用。ROS中的节点句柄是用于与ROS系统进行通信的主要方式之一，允许节点发布和订阅主题、 调用服务等。moveit::planning_interface::MoveGroupInterface &arm 为一个moveit库中的MoveGroupInterface对象的引用，moveit是用于机器人运动规划和控制的库，MoveGroupInterface用来管理机器人的运动规划和执行。const string &PLANNING_GROUP用于指定机器人规划组的名称，在moveit中，机器人的运动规划是根据器所属的规划组进行的。

> **bool move_by_joint(const vector<double> &joint_group_positions);**
> 
> 该函数实现了机械臂的关节运动，const vector<double> &joint_group_positions为一个类型为vector的变长数组，里面定义的数值为机械臂每个轴需要达到的角度数值，机械臂关节角度的范围为-3.14到3.14。

> **bool move_joint(const vector<double> &joint_group_positions);**
> 
> 该函数实现了机械臂的关节运动，const vector<double> &joint_group_positions为一个类型为vector的变长数组。该函数与move_by_joint(const vector<double> &joint_group_positions)的区别为：move_by_joint函数实现了机械臂各关节运动到的位置为joint_group_positions中数据的位置，move_joint实现了机械臂各关节运动到的位置为当前的角度加上传入的joint_group_positions中数据的位置。
> 例如当前机械臂六关节的位置分别为0.2、0.3、0.4、0.5、0.6、0.7,joint_group_positions中的数据为1、1、1、1、1、1，如果运行move_by_joint函数，则机械臂各关节角度为1、1、1、1、1、1，如果运行move_joint函数，则机械臂各关节角度为1.2、1.3、1.4、1.5、1.6、1.7.

> **bool move_by_pos(const vector<double> &pose);**
> 
> 该函数实现了机械臂的位姿运动，const vector<double> &pose 为一个类型为vector的变长数组，里面定义了机械臂末端位姿的x、y、z、roll、pitch、yaw的具体数值。x、y、z分别是绩溪北末端在基坐标系下的位置坐标，描述了机械臂末端在三个空间方向上的位置，roll、pitch、yaw分别代表绕x、y、z轴的旋转角度，描述了机械臂末端的姿态或方向。在某些位置，由于机械臂结构的限制或特定的姿态导致给出x、y、z、roll、pitch、yaw并不能解算出对应的机械臂关节角度，如果不能解算出相应的关节角度，该函数则会给出move_p:FAILED的提示。

> **void get_joint();**
> 
> 该函数实现了得到当前机械臂各关节的角度并显示在终端窗口中。

> **void get_pos();**
> 
> 该函数实现了得到当前机械臂末端位姿x、y、z、roll、pitch、yaw并显示在终端窗口中。


> **void change_v(int v_);**
> 
> 该函数实现了改变机械臂运动速度，程序中设置的机械臂默认速度为3651转速。

> **void clean_error();**
> 
>  该函数实现了清除电机错误。电机有时会出现宕机不运动情况，该函数可以清除电机错误，作用类似于重启电机。

 
#### 具体使用步骤说明：
**1.Ti5robot功能包配置**


由于与机械臂通信需要一个控制CAN的libcontrolcan.so库，请先确保Ti5robot功能包中的CMakeList.txt中的libcontrolcan.so库的路径是正确的，即根据下图中第33行的路径是否能够找到libcontrolcan.so。

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/142a628c68c64581a244afad7ac42c65.png#pic_center)


**2.创建新的功能包**


完成第一步之后就可以创建新的功能包来使用Ti5robot功能包中的函数。
首先执行 catkin_create_pkg  demo  roscpp  rospy  Ti5robot  std_msgs
该命令会创建一个demo功能包并生成CMakeList.txt、package.xml和src文件夹，在src文件夹中新建demo.cpp文件。
如下图所示

![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/50f1d7db6c11416d9a28b694fee47e8a.png#pic_center)



头文件中通过#include<Ti5robot/Ti5robot.h>来调用Ti5robot类的函数。
上图中的第6至15行实现了定义ros句柄，规划组，MoveGroupInterface对象并用来初始化一个叫my的Ti5robot类，并且实现订阅/move_group/display_planned_path。
初始化之后可以通过调用my.move_by_pos(p)这种方式来调用Ti5robot类中实现好了的函数。

**3.配置demo功能包的CMakeList.txt和package.xml**


上述的demo.cpp编写完成之后，就可以配置demo功能包的CMakeList.txt和package.xml

CMakeList.txt


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/e710cbb1739b45bf9e030a6c8df24c96.png#pic_center)




CMakeList.txt文件中find_package中要加上Ti5robot，catkin_package中CATKIN_DEPENDS中也要加上Ti5robot，一般来说，如果创建功能包catkin_create_pkg时就加上了Ti5robot的话CMakeList.txt中就不需要再手动添加，然后在CMakeList.txt文件后面完善add_dependencies和target_link_libraries相关即可。

package.xml


![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/ef907ae0b40b4c369fb940e574e21a8c.png#pic_center)

在package.xml文件中需要添加
```bash
<build_depend>Ti5robot</build_depend>
<build_export_depend>Ti5robot</build_export_depend>
<exec_depend>Ti5robot</exec_depend>
```

一般来说，如果创建功能包catkin_create_pkg时就加上了Ti5robot的话CMakeList.txt中就不需要再手动添加。


**4.编译运行**


完成上述工作之后，在ros的工作空间下运行catkin_make进行编译。
编译成功之后，记得运行source devel/setup.bash来使更改生效。

完成之后打开一个新的终端输入rosrun arm1 demo.launch （此处只以arm1为例）
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/8221c537575d4ae5a44789e576656822.png#pic_center)


然后在打开第二个终端，输入sudo chmod -R 777 /dev/
然后输入rosrun demo demo便可以看到机械臂的运动
![在这里插入图片描述](https://img-blog.csdnimg.cn/direct/6ed0562b325d439dbd7943f616dd60a7.png#pic_center)







