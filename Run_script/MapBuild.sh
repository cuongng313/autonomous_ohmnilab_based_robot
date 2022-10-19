export ROS_IP=192.168.0.103
export ROS_MASTER_URI=http://192.168.0.102:11311
cd ../XigloBot
source devel/setup.bash
roslaunch xiglo_launch xiglo_slam.launch


