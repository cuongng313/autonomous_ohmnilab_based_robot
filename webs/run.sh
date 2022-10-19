pkill roslaunch
#export ROS_MASTER_URI=http://192.168.0.102:11311

# export ROS_IP=192.168.100.9

#roslaunch rosbridge_server rosbridge_websocket.launch &
#rosrun tf2_web_republisher tf2_web_republisher
roslaunch /var/www/webs/launch.launch
