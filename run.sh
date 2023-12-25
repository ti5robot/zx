#!/bin/bash


## startup command
## gnome-terminal -x bash -c "/home/ti5robot/run.sh;exec bash"

echo "please choose the program to run:"
echo "1. Run roslaunch arm1 demo.launch"
echo "2. Run roslaunch arm2 demo.launch"
echo "3. Run roslaunch arm3 demo.launch"
echo "5. Run roslaunch arm5 demo.launch"

read -p "Enter option (1,2,3 or 5): " choice


echo "please choose the program to run:  "
echo "1. Run serial_demo"
echo "2. Run serial_auto"

read -p "Enter option (1 or 2): " choice_2


case $choice in
	1)
		echo "Setting permissions and running arm1 demo.launch"
		gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash; roslaunch arm1 demo.launch"
		;;
	2)
		echo "Setting permissions and running arm2 demo.launch"
                gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash; roslaunch arm2 demo.launch"
                ;;
	3)
		echo "Setting permissions and running arm3 demo.launch"
		gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash; roslaunch arm3 demo.launch"
                ;;
	5)
		echo "Setting permissions and running arm5 demo.launch"
                gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash; roslaunch arm5 demo.launch"
                ;;
	*)
		echo "Invalid option,please enter 1,2,3 or 5"
		;;
esac

sleep 5

#echo "please choose the program to run:  "
#echo "1. Run serial_demo"
#echo "2. Run serial_auto"

#read -p "Enter option (1 or 2): " choice_2

case $choice_2 in
	1)
		password="dongguan"
		echo "Setting permissions and running serial_demo"
		echo "$password" | sudo -S chmod -R 777 /dev/
		gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash;rosrun serial_demo serial_demo;exec bash"
		;;
	2)
		password="dongguan"
		echo "Setting permissions and running serial_auto"
		echo "$password" | sudo -S chmod -R 777 /dev/
		gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash;source /home/ti5robot/devel/setup.bash;rosrun serial_demo serial_auto;exec bash"
		;;
	*)
		echo "Invalid opttion,please enter 1 or 2"
		;;
esac

