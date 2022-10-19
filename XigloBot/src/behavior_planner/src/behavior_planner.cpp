#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>  
#include <std_msgs/Int8.h> 
#include <FSM.h> 

int main(int argc, char** argv){
  ros::init(argc, argv, "behavior_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  /* ROS multithreading */
  ros::AsyncSpinner spinner(4);
  spinner.start();

  const double xOrigin = -9.77, yOrigin = 10.45, yawOrigin = -M_PI/2;
  const double xShowroom = -11, yShowroom = 12, yawShowroom = -M_PI/2;
  RobotState robotFSM(xOrigin, yOrigin, yawOrigin, xShowroom, yShowroom, yawShowroom);
  robotFSM.modeManager.data = 2;            // Mode default: 2 - manual

  /* ROS subscriber */
  ros::Subscriber subModeManager = n.subscribe("/mode_manager", 10, &RobotState::modeManagerCallback, &robotFSM);
  ros::Subscriber sub_desired_pose = n.subscribe("/pose_js", 1 , &RobotState::poseJsCallback, &robotFSM);
  ros::Subscriber sub_amcl = n.subscribe("/amcl_pose", 1 , &RobotState::poseAMCLCallback, &robotFSM);
  ros::Subscriber sub_face = n.subscribe("/nvidia_pose", 1, &RobotState::poseFaceCallback, &robotFSM);

  /* ROS publisher */
  ros::Publisher pubInitialPose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
  ros::Publisher TwistPub = n.advertise<geometry_msgs::Twist>("/tb_cmd_vel", 100);
  ros::Publisher pubOmniStatus = n.advertise<std_msgs::Int8>("/OhmniStatus",10);

  std_msgs::Int8 OmniStatus;
  OmniStatus.data = 0;
  
  bool checkStopCondition = 0;
  geometry_msgs::Twist twist;
  twist.angular.z = 0.12;
  bool checkRotate = 1;

  while (ros::ok())
  { 
    if (robotFSM.modeManager.data == 1) {
      robotFSM.AutoMode();         // Auto mode
      ROS_INFO ("Auto mode!!!");
      robotFSM.autoModeState = 0;
    }
    else if (robotFSM.modeManager.data == 2) {  
      ROS_INFO ("Manual mode!!! Please take control!");
      robotFSM.ManualMode();      // Manual mode 
    }
    else if (robotFSM.modeManager.data == 7) {
      geometry_msgs::PoseWithCovarianceStamped poseInitial;           //set Initial Position
      poseInitial = robotFSM.setInitialPose(12.7, 10.5, 0.0);
      pubInitialPose.publish(poseInitial);
      robotFSM.robotMode = 10;
    }
    else if (robotFSM.modeManager.data == 10) {}                      // Empty Mode
    else {
      robotFSM.PositionNavMode(robotFSM.modeManager.data);            // To position mode
      if(robotFSM.ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if (robotFSM.modeManager.data != 0 && robotFSM.modeManager.data != 6)
        {
          OmniStatus.data = 1;
          pubOmniStatus.publish(OmniStatus);
          OmniStatus.data = 0;
          pubOmniStatus.publish(OmniStatus);
        }
      }
    }

    robotFSM.modeManager.data = 10;

    /* Auto Mode */
    if (robotFSM.robotMode != 1) {
      robotFSM.autoModeState = 0;              // reset state automode
    }
    //Robot Finite State Machine
    if (robotFSM.robotMode == 1) {
      switch (robotFSM.autoModeState) {
        case 0:
          robotFSM.ToOrigin();
          robotFSM.autoModeState = robotFSM.fsm.getState();
          break;
        case 1:
          robotFSM.SelfRotandScan();
          TwistPub.publish(twist);
          if((robotFSM.yaw >= 0 || robotFSM.yaw <= -M_PI) && checkRotate == 1) {
            twist.angular.z = -twist.angular.z;
            checkRotate = 0;
          }
          if(robotFSM.yaw < 0 && robotFSM.yaw > -M_PI) {
            checkRotate = 1;
          }
          ROS_INFO("yaw = [%f]", robotFSM.yaw);
          robotFSM.autoModeState = robotFSM.fsm.getState();
          break;
        case 2:
          robotFSM.ToPerson();
          ROS_INFO("Move to person");
          robotFSM.autoModeState = robotFSM.fsm.getState();
          break;
        case 3: {  
          const double disThresh = 0.55;
          double distance = robotFSM.xPerson_ - 0.4;
          checkStopCondition = robotFSM.StopandVoice(disThresh, distance);
          robotFSM.autoModeState = robotFSM.fsm.getState();  
        } break;
        case 4:
          ROS_INFO("StopCondition: [%d]", checkStopCondition);
          if(checkStopCondition == 1) {
            OmniStatus.data = 1;
            pubOmniStatus.publish(OmniStatus);
            ros::Duration(10.0).sleep(); 
            OmniStatus.data = 0;
            pubOmniStatus.publish(OmniStatus);
            checkStopCondition = 0;
          }
          robotFSM.ToShowroom(); 
          robotFSM.autoModeState = robotFSM.fsm.getState();
          break;
        default:
          break;
      }
    }
    loop_rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}