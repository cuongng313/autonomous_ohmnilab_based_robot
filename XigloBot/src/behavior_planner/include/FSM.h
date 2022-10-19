#include <ros/ros.h>
#include <std_msgs/Int8.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <math.h> 
#include <string>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FSMRobot {
private:
  int activeState = 0;
  
public:
  FSMRobot();
  void setState(int state);
  int getState();
};

class RobotState {
public:
  int robotMode = 0;
  FSMRobot fsm;
  int checkMsg_ = 0;
  std::string framePerson_ = "footprint";
  double xPerson_ = 0, yPerson_ = 0, yawPerson_ = 0;
  geometry_msgs::Twist twist_;
  move_base_msgs::MoveBaseGoal goalState;
  
  MoveBaseClient *ac;
  std_msgs::Int8 modeManager;

  double roll, pitch, yaw;

  int autoModeState = 0;

  RobotState(double xOrigin, double yOrigin, double yawOrigin,
           double xShowroom, double yShowroom, double yawShowroom);


  /* Robot State */
  void ToOrigin();
  void ToShowroom();
  void ToPosition(double x, double y, double yaw);
  void SelfRotandScan();
  void ToPerson();
  bool StopandVoice(double disThresh, double distance);

  /* function */
  move_base_msgs::MoveBaseGoal setupGoal(std::string frameID, double x, double y, double yaw);
  void setOriginPos(double x, double y, double yaw);
  void setShowroomPos(double x, double y, double yaw);

  /* mode */
  void AutoMode();
  void ManualMode();
  void PositionNavMode(int posNum);

  geometry_msgs::PoseWithCovarianceStamped setInitialPose(double x, double y, double yaw);

  /* subscriber callback */
  void modeManagerCallback(const std_msgs::Int8& modeMsg);
  void poseJsCallback(const geometry_msgs::Pose::ConstPtr& msgPose);
  void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
  void poseFaceCallback(const geometry_msgs::Pose::ConstPtr& msg);

private:
  std::string frameOrigin_, frameShowroom_;
  double xOrigin_ = 0, yOrigin_ = 0, yawOrigin_ = 0;
  double xShowroom_ = 0, yShowroom_ = 0, yawShowroom_ = 0;
  bool activatedPersonGoal_ = 0;
};



