

# ROS接口使用说明（持续更新中）

项目github地址：[https://github.com/ti5robot/zx(https://github.com/ti5robot/zx)

### 关于Ti5_py功能包介绍
Ti5_py功能包中主要实现了Ti5_py类，功能包中集成了函数具体实现和CAN通信以及相应的ROS头文件。
使用时请将Ti5_py文件夹和controlcan文件夹下载到本地文件中。

#### 相应函数功能和参数说明

> **def move_by_joint(joint_group_positions)**
> 该函数实现了机械臂的关节运动，joint_group_positions可以是一个类型为list的python列表，里面定义的数值为机械臂每个轴需要达到的角度数值，机械臂关节角度的范围为-3.14到3.14。

> **def move_joint(joint_group_positions)**
> 该函数实现了机械臂的关节运动，joint_group_positions可以是一个类型为list的python列表。该函数与move_by_joint(joint_group_positions)的区别为：move_by_joint函数实现了机械臂各关节运动到的位置为joint_group_positions中数据的位置，move_joint实现了机械臂各关节运动到的位置为当前的角度加上传入的joint_group_positions中数据的位置。
> 例如当前机械臂六关节的位置分别为0.2、0.3、0.4、0.5、0.6、0.7,joint_group_positions中的数据为1、1、1、1、1、1，如果运行move_by_joint函数，则机械臂各关节角度为1、1、1、1、1、1，如果运行move_joint函数，则机械臂各关节角度为1.2、1.3、1.4、1.5、1.6、1.7.

> **def move_by_pos(pose)** 
> 该函数实现了机械臂的位姿运动，pose 可以是一个类型为list的python列表，里面定义了机械臂末端位姿的x、y、z、roll、pitch、yaw的具体数值。x、y、z分别是绩溪北末端在基坐标系下的位置坐标，描述了机械臂末端在三个空间方向上的位置，roll、pitch、yaw分别代表绕x、y、z轴的旋转角度，描述了机械臂末端的姿态或方向。在某些位置，由于机械臂结构的限制或特定的姿态导致给出x、y、z、roll、pitch、yaw并不能解算出对应的机械臂关节角度，如果不能解算出相应的关节角度，该函数则会给出false的提示。

> **def get_joint()**
> 该函数实现了得到当前机械臂各关节的角度并显示在终端窗口中。

> **def get_pos()**
> 该函数实现了得到当前机械臂末端位姿并显示在终端窗口中。


> **def change_v(v_)**
> 该函数实现了改变机械臂运动速度，程序中设置的机械臂默认速度为3651转速。

> **def clean_error()**
>  该函数实现了清除电机错误。电机有时会出现宕机不运动情况，该函数可以清除电机错误，作用类似于重启电机。

> **def get_error();**
> 
>  该函数实现了读取机械臂六个电机错误的电机错误。如果返回值为0说明电机没有发生错误。返回值为32位有符号类型，对应位为0表示无错误，为1表示出现错误：bit0代表软件错误，如电机运行时写入FLASH等，bit1代表过压，bit2代表欠压，bit4代表启动错误，bit5代表速度反馈错误，bit6代表过流，bit16代表编码器通讯错误，bit17代表电机温度过高，bit18代表电路板温度过高。

> **def get_electric();**
> 
> 该函数实现了读取机械臂六个电机的电流。


#### 具体使用步骤说明：
**1.Ti5_py功能包配置**


由于与机械臂通信需要一个控制CAN的libcontrolcan.so库，请先确保Ti5_py功能包中的Ti5_py.py文件中的init_can函数中的libcontrolcan.so库的路径是正确的，即根据所给出的路径是否能够找到libcontrolcan.so。




**2.创建新的功能包**


完成第一步之后就可以创建新的功能包来使用Ti5_py功能包中的函数。
首先执行 catkin_create_pkg  py_demo  roscpp  rospy  Ti5_py  std_msgs
该命令会创建一个py_demo功能包并生成CMakeList.txt、package.xml和src文件夹，在src文件夹中新建py_demo.py文件。




头文件中通过from Ti5_py import Ti5_py来调用Ti5_py功能包中的类和相应函数。
在main函数可通过ayay=Ti5_py()来生成一个对象,然后便可以调用所需的函数来实现相应的功能。

**3.配置py_demo功能包的CMakeList.txt**


上述的py_demo.py编写完成之后，就可以配置py_demo功能包的CMakeList.txt


CMakeList.txt文件中find_package中要加上Ti5_py，catkin_package中CATKIN_DEPENDS中也要加上Ti5_py，一般来说，如果创建功能包catkin_create_pkg时就加上了Ti5_py的话CMakeList.txt中就不需要再手动添加，然后在CMakeList.txt文件后面添加下面代码

```bash
catkin_install_python(PROGRAMS
	src/py_demo.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
```



**4.编译运行**


完成上述工作之后，在ros的工作空间下运行catkin_make进行编译。
编译成功之后，记得运行source devel/setup.bash来使更改生效。

完成之后打开一个新的终端输入rosrun arm1 demo.launch （此处只以arm1为例）


然后在打开第二个终端，输入sudo chmod -R 777 /dev/
然后输入rosrun py_demo py_demo.py便可以看到机械臂的运动









