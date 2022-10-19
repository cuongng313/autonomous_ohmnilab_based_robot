# CycloBot
## Connect to the robot:
1. Turn on the robot
2. Check the ip address of the robot:
- Login: https://app.ohmnilabs.com/my-robots
- Get the ip address from the web interface
3. Connect to Ohmni:
- From ubuntu PC:
adb connect 10.111.194.222
adb shell
su
setprop ctl.stop tb-node
- Run the docker:
docker run -it --network host --privileged -e ROS_IP=10.111.194.222 -v /dev:/dev ohmnilabs:vinai4 bash
4. Working with Ohmni docker:
- setup environment:
source /opt/ros/melodic/setup.bash
- enable controller:
rosservice call /tb_control/enable_controllers "enable_controller: true
	pid_velocity_motor_left: true
	pid_velocity_motor_right: true"
- control velocity:
mmand that drives straightforward at velocity 0.1m/s
In Docker: rostopic pub -1 /tb_cmd_vel geometry_msgs/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
- view full terminal:
tmux attach -t work


ROS package install
Auto install all necessary library
    rosdep install --from-paths src --ignore-src -r -y
