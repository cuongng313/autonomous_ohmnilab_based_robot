#include <FSM.h>

/* Robot State in Auto Mode */
//////////////////////////////////////////////////////
FSMRobot::FSMRobot()
{
  ROS_INFO("robot state setup");
}
void FSMRobot::setState(int state){
  activeState = state;
}
int FSMRobot::getState(){
  return activeState;
}
/////////////////////////////////////////////////////////

  /* Robot State */
//////////////////////////////////////////////////////////
RobotState::RobotState(double xOrigin, double yOrigin, double yawOrigin, 
                  double xShowroom, double yShowroom, double yawShowroom){
  fsm.setState(0);
  //wait for the action server to come up
  ac = new MoveBaseClient("move_base", 1);
  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  setOriginPos(xOrigin, yOrigin, yawOrigin);
  setShowroomPos(xShowroom, yShowroom, yawShowroom);
}

/*Moving Mode */
void RobotState::ToOrigin(){
  goalState = setupGoal(frameOrigin_, xOrigin_, yOrigin_, yawOrigin_);
  ac->sendGoal(goalState);
  ROS_INFO("To Origin !!!");
  ac->waitForResult();
  ROS_INFO("At Origin !!!");
  ros::Duration(2.0).sleep();
  fsm.setState(1);
}

void RobotState::ToShowroom(){
  goalState = setupGoal(frameShowroom_, xShowroom_, yShowroom_, yawShowroom_);
  ac->sendGoal(goalState);
  ROS_INFO("To Showroom !!!");
  ac->waitForResult();
  ROS_INFO("At Showroom !!!");
  ros::Duration(2.0).sleep();
  fsm.setState(0);
}

void RobotState::ToPosition(double x, double y, double yaw){
  goalState = setupGoal(frameOrigin_, x, y, yaw);
  ac->sendGoal(goalState);
  ac->waitForResult();
  ROS_INFO("At Position !!!");
}

void RobotState::SelfRotandScan(){
  ROS_INFO("Scanning people");
  ROS_INFO("CheckMsg: [%d]", checkMsg_);
  if (checkMsg_ >= 10)
  {
    ROS_INFO("Detected people");
    checkMsg_ = 0;
    fsm.setState(2);
  }
  else{
    fsm.setState(1);
  }
}

void RobotState::ToPerson(){
  goalState = setupGoal(framePerson_, xPerson_, yPerson_, yawPerson_);
  if (activatedPersonGoal_ == 0)
  {
    ac->sendGoal(goalState);
    activatedPersonGoal_ = 1;
  } 
  fsm.setState(3);
}

bool RobotState::StopandVoice(double disThresh, double distance){
  // numMoveState == 1: Succeed
  if (ac->getState() == actionlib::SimpleClientGoalState::ABORTED)
  {
    ROS_INFO("Aborted");
    fsm.setState(0);
    activatedPersonGoal_ = 0;
    return false;
  }
  if ((distance > 0) && (distance < disThresh) || ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ac->cancelAllGoals();
    ROS_INFO("Reached person");
    fsm.setState(4);
    activatedPersonGoal_ = 0;
    return true;
  }
  else 
  {
    ROS_INFO ("Continue to person");
    fsm.setState(3);
    return false;
  }
}

/* supported function */
move_base_msgs::MoveBaseGoal RobotState::setupGoal(std::string frameID, double x, double y, double yaw){
  // Angular to Quaternion
  tf::Quaternion quat;
  quat = tf::createQuaternionFromYaw(yaw);
  geometry_msgs::Quaternion qMsg;
  tf::quaternionTFToMsg(quat, qMsg);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frameID;
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = qMsg;

  return goal;
}

void RobotState::setOriginPos(double x, double y, double yaw){
  frameOrigin_ = "map";
  xOrigin_ = x;
  yOrigin_ = y;
  yawOrigin_ = yaw;
}

void RobotState::setShowroomPos(double x, double y, double yaw){
  frameShowroom_ = "map";
  xShowroom_ = x;
  yShowroom_ = y;
  yawShowroom_ = yaw;
}

/* Robot mode*/
void RobotState::AutoMode(){
  RobotState::robotMode = 1;
}

void RobotState::ManualMode(){
  ac->cancelAllGoals();
  RobotState::fsm.setState(0);
  RobotState::robotMode = 2;
}

void RobotState::PositionNavMode(int posNum){
  RobotState::fsm.setState(0);
  RobotState::robotMode = posNum;
  switch (posNum)
  {
  case 0:
      ROS_INFO("To Home");
      RobotState::ToOrigin();
      break;
  case 6:
      ROS_INFO("To Showroom");
      RobotState::ToShowroom();
      break;
  case 3:
      ROS_INFO("To Position A");
      RobotState::ToPosition(-5.0, 11.5, -M_PI/4);
    break;
  case 4:
      ROS_INFO("To Position B");
      RobotState::ToPosition(-5.5, 9.1, -M_PI/2.5);
    break;
  case 5:
      ROS_INFO("To Position C");
      RobotState::ToPosition(-5.77, 4.7, -M_PI/2);
    break;
  default:
    break;
  }
}

geometry_msgs::PoseWithCovarianceStamped RobotState::setInitialPose(double x, double y, double yaw) {
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();;
    msg.header.frame_id = "/map";

    // set 2D position
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0;

    // set 2D orientation
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    msg.pose.pose.orientation.x = q[0];
    msg.pose.pose.orientation.y = q[1];
    msg.pose.pose.orientation.z = q[2];
    msg.pose.pose.orientation.w = q[3];
    msg.pose.covariance.assign(0);

    // set covariances
    msg.pose.covariance[6 * 0 + 0] = 0.25; // position x
    msg.pose.covariance[6 * 1 + 1] = 0.25; // position y
    msg.pose.covariance[6 * 5 + 5] = 0.1; // orientation
    return msg;
}

/* Callback function */
void RobotState::modeManagerCallback(const std_msgs::Int8& modeMsg)
{
  modeManager.data = modeMsg.data;
  if(modeMsg.data == 16){
    ROS_INFO("Emergency Stop!!!");
    ac->cancelAllGoals();
  }
}

void RobotState::poseJsCallback(const geometry_msgs::Pose::ConstPtr& msgPose)
{
  ROS_INFO("To target position!!!");
  ToPosition(msgPose->position.x, msgPose->position.y, msgPose->orientation.w);
}

void RobotState::poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{ 
  tf::Quaternion q(msgAMCL->pose.pose.orientation.x, msgAMCL->pose.pose.orientation.y, msgAMCL->pose.pose.orientation.z, msgAMCL->pose.pose.orientation.w); 
  tf::Matrix3x3 m(q);
  m.getRotation(q);
  m.getRPY(roll, pitch, yaw);
}

void RobotState::poseFaceCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    xPerson_ = msg->position.x - 0.25;
    yPerson_ = msg->position.y;
    yawPerson_ = atan2(msg->position.y, msg->position.x);    
    if (msg->orientation.w == 1 && fsm.getState() == 1)
    {
      checkMsg_++;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////

