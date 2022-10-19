#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>  
#include <std_msgs/Int8.h> 
#include <FSM.h>    


// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double poseAMCLx, poseAMCLy, poseAMCLyaw, poseAMCLw;
double poseFaceX, poseFaceY, poseFaceZ, poseFaceCheck;
double faceDisX = 0, faceDisY = 0, faceDisZ = 0;

int size = 0;
int checkMsg = 0;
bool timeOut = 0;

void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLyaw = msgAMCL->pose.pose.orientation.z;   
    poseAMCLw = msgAMCL->pose.pose.orientation.w;  
}

void poseFaceCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    poseFaceX = msg->position.x;
    poseFaceY = msg->position.y;
    poseFaceZ = msg->position.z;   
    poseFaceCheck = msg->orientation.w;  
    // if (poseFaceCheck == 1)
    // {
    //   checkMsg++;
    // }
}


move_base_msgs::MoveBaseGoal goal;
int OriginStatus = 0;

int main(int argc, char** argv){
  ros::init(argc, argv, "behavior_planner");
  ros::NodeHandle n;



  ros::Time currentTime;

  ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 1 , poseAMCLCallback);
  ros::Subscriber sub_face = n.subscribe("nvidia_pose", 1, poseFaceCallback);
  ros::Publisher TwistPub = n.advertise<geometry_msgs::Twist>("/tb_cmd_vel", 100);
  ros::Publisher pubOmniStatus = n.advertise<std_msgs::Int8>("/OhmniStatus",10);

  ros::Rate loop_rate(10);

  //tell the action client that we want to spin a thread by default
  // MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  // while(!ac.waitForServer(ros::Duration(5.0))){
  //   ROS_INFO("Waiting for the move_base action server to come up");
  // }

  std_msgs::Int8 OmniStatus;
  OmniStatus.data = 0;

  double xOrigin = 1.2, yOrigin = 2, yawOrigin = 0;
  double xShowroom = 1.2, yShowroom = 2.00, yawShowroom = 0;

  RobotState robotFSM(xOrigin, yOrigin, yawOrigin, xShowroom, yShowroom, yawShowroom);
  int stateStatus = 0;
  bool checkMoveResult = 0;
  bool checkStopCondition = 0;

  while (ros::ok())
  {
    switch (stateStatus)
    {
      case 0:
        robotFSM.ToOrigin();
        ros::Duration(2.0).sleep(); 
        stateStatus = robotFSM.fsm.getState();
        break;
      case 1:
        if (poseFaceCheck == 1)
        {
          robotFSM.checkMsg_++;
        }
        robotFSM.SelfRotandScan();
        TwistPub.publish(robotFSM.twist_);
        stateStatus = robotFSM.fsm.getState();
        break;
      case 2:
        robotFSM.xPerson_ = poseFaceX - 0.6;
        robotFSM.yPerson_ = poseFaceY - 0.2;
        robotFSM.yawPerson_ = 0.0;
        robotFSM.framePerson_ = "footprint";
        robotFSM.ToPerson();
        ROS_INFO("Move to person");
        stateStatus = robotFSM.fsm.getState();
        break;
      case 3: {  
        const double disThresh = 0.55;
        double distance = poseFaceX - 0.6;
        checkStopCondition = robotFSM.StopandVoice(disThresh, distance);
        stateStatus = robotFSM.fsm.getState();
        
      } break;
      case 4:
        ROS_INFO("StopCondition: [%d]", checkStopCondition);
        if(checkStopCondition == 1) //play sound
        {
          OmniStatus.data = 1;
          pubOmniStatus.publish(OmniStatus);
          ros::Duration(8.0).sleep(); 
          OmniStatus.data = 0;
          checkStopCondition = 0;
        }
        robotFSM.ToShowroom();
        ros::Duration(2.0).sleep(); 
        stateStatus = robotFSM.fsm.getState();
        break;

      default:
        break;
    }
    


    // ROS_INFO("Origin status: [%d]", OriginStatus);
    // ROS_INFO("active goal: [%d]", activatedGoal);
    // ROS_INFO("send enable: [%d]", goalSendEnable);
    // ROS_INFO("Local Face position: [%f] [%f] [%f]", poseFaceX, poseFaceY, LocalAngular);
    // ROS_INFO("checkMsg: [%d] \r\n", checkMsg);
    

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
