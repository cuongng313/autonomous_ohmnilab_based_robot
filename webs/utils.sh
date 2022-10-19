server=root@192.168.0.102
ohmni_addr=192.168.0.102
## Android operation
function connect_ohmni() {
	adb connect $ohmni_addr:5555
}
function disconnect_ohmni() {
	adb disconnect
}
function kill_docker() {
	a=$(ssh $server "docker ps -q")
	echo $a
	ssh $server "docker kill $a"
}
function start_docker() {
	ssh $server "docker start quizzical_feynman"
	#adb shell "su -c 'docker start quizzical_feynman'"
}

function get_ip() {
	a=$(ip addr show wlo1 | grep "inet\b" | awk '{print $2}' | cut -d/ -f1)
	echo $a
}

function play_sound() {
	adb shell am broadcast -p io.vinai.greeting -a io.vinai.intent.SOUND --es sound "iloveyou"
}

function ohmni_enable() {
	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	## STOP the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 2 && tmux send-key C-c\"'"
	adb shell setprop ctl.restart tb-node
}
function ohmni_disable() {
	adb shell setprop ctl.stop tb-node
	echo "Ohmnilabs control was disabled"

	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	##START the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 2 && tmux send-key C-c\"'"
	adb shell "su -c 'docker exec -i $R bash -c \"tmux selectp -t 2 && tmux send-key roslaunch Space tb_control Space tb_control.launch C-m\"'"
}

function reset_lidar() {
	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	##START the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 1 && tmux send-key C-c\"'"
	adb shell "su -c 'docker exec -i $R bash -c \"tmux selectp -t 1 && tmux send-key roslaunch Space rplidar_ros Space rplidar.launch C-m\"'"
}
function reset_tb() {
	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	##START the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 2 && tmux send-key C-c\"'"
	adb shell "su -c 'docker exec -i $R bash -c \"tmux selectp -t 2 && tmux send-key roslaunch Space tb_control Space tb_control.launch C-m\"'"
}
function launch_cuong() {
	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	##START the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 3 && tmux send-key C-c\"'"
	adb shell "su -c 'docker exec -i $R bash -c \"tmux selectp -t 3 && tmux send-key roslaunch Space xiglo_launch Space xiglo_amcl.launch C-m\"'"
}
function stop_cuong() {
	R=$(adb shell "su -c 'docker ps -q'")
	echo $R
	##START the tb_control node
	adb shell "su -c 'docker exec $R bash -c \"tmux selectp -t 3 && tmux send-key C-c\"'"
}
function wait_roscore {
        while [ -z "$(rostopic list |grep rosout)" ]
        do
                echo "waiting for rosmaster ..."
                sleep 1;
        done
}

function run_nx() {
	ssh nx-advantech "pkill tmux; sleep 1; bash tmux.sh"
	echo "message sent successfully"
}
function stop_nx() {
	ssh nx-advantech "pkill tmux;"
	echo "message sent successfully"
}

export -f wait_roscore

param=$1
case $param in
	connect)
		echo "connecting to the android ..."
		connect_ohmni
		;;
	disconnect)
		echo "disconnecting to the android ..."
		disconnect_ohmni
		;;
	kill_docker)
		echo "killing the docker ..."
		kill_docker
		;;
	start_docker)
		echo "starting the docker ..."
		start_docker
		;;
	play_sound)
		echo "playing sound on android ..."
		play_sound
		;;
	ohmni_enable)
		echo "enabling the android ..."
		ohmni_enable
		;;
	ohmni_disable)
		echo "disabling the android ..."
		ohmni_disable
		;;
	reset_lidar)
		echo "reseting the lidar ..."
		reset_lidar
		;;
	reset_tb)
		echo "resetting the tb-control ..."
		reset_tb
		;;
	testing)
		echo $server
		;;
	launch_cuong)
		echo "launching cuong's program"
		launch_cuong
		;;
	stop_cuong)
		echo "stopping cuong's program"
		stop_cuong
		;;
	run_nx)
		echo "running nx facemask detection"
		run_nx
		;;
	stop_nx)
		echo "stop running nx facemask detection"
		stop_nx
		;;
	*) 
		echo "unknown option"
		;;
	
esac


