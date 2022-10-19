#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry OdomData;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  tf::Quaternion quat(  msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w );
  // quat.normalize();
  double roll, pitch, yaw;
  tf::Matrix3x3 m(quat);
  m.getRPY(roll, pitch, yaw);

  OdomData.header.seq = msg->header.seq;
  OdomData.header.stamp = msg->header.stamp;

  OdomData.pose.pose.position.x = msg->pose.pose.position.x;
  OdomData.pose.pose.position.y = msg->pose.pose.position.y;
  OdomData.pose.pose.position.z = msg->pose.pose.position.z;

  OdomData.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  OdomData.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  OdomData.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  OdomData.pose.pose.orientation.w = msg->pose.pose.orientation.w;

  OdomData.twist.twist.linear.x = msg->twist.twist.linear.x;
  OdomData.twist.twist.linear.y = msg->twist.twist.linear.y;
  OdomData.twist.twist.angular.z = msg->twist.twist.angular.z;

  ROS_INFO("x: [%f]", OdomData.pose.pose.position.x);
  ROS_INFO("y: [%f]", OdomData.pose.pose.position.y);
  ROS_INFO("Quat.z: [%f]", OdomData.pose.pose.orientation.z);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_setup");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  static tf::TransformBroadcaster odomBroadcaster;

  geometry_msgs::TransformStamped odomTrans;
  nav_msgs::Odometry odom;
  ros::Time currentTime;

  ros::Subscriber odomSub = n.subscribe("/tb_control/wheel_odom", 1000 , odomCallback);
  ros::Publisher odomPub = n.advertise<nav_msgs::Odometry>("OdomWheel", 1000);

  while(n.ok()){
    // ROS_INFO("test");

      // broadcast transfrom Odom --> BaseLink
    // odomTrans.header.frame_id = "odom_wheel";
    // odomTrans.child_frame_id = "base_link";

    // geometry_msgs::Quaternion quat;
    // quat.x = OdomData.pose.pose.orientation.x;
    // quat.y = OdomData.pose.pose.orientation.y;
    // quat.z = OdomData.pose.pose.orientation.z;
    // quat.w = OdomData.pose.pose.orientation.w;


    // std::cout << "Odometry subscriber: x, y, angular: " << OdomData.pose.pose.position.x << "  "  <<  OdomData.pose.pose.position.y << "  " << OdomData.pose.pose.orientation.z << std::endl;

    // odomTrans.transform.translation.x = OdomData.pose.pose.position.x;
    // odomTrans.transform.translation.y = OdomData.pose.pose.position.y;
    // odomTrans.transform.translation.z = OdomData.pose.pose.position.z;
    // odomTrans.transform.rotation = quat;

    // odomBroadcaster.sendTransform(odomTrans);

    //   // publish odom
    // currentTime = ros::Time::now();
    // // odom.header.stamp = OdomData.header.stamp;
    // odom.header.stamp = currentTime;
    // odom.header.frame_id = "odom_wheel";
    // odom.child_frame_id = "base_link";

    // odom.pose.pose.position.x = OdomData.pose.pose.position.x;
    // odom.pose.pose.position.y = OdomData.pose.pose.position.y;
    // odom.pose.pose.position.z = 0;
    // odom.pose.pose.orientation = quat;

    // odom.twist.twist.linear.x = OdomData.twist.twist.linear.x;
    // odom.twist.twist.linear.y = OdomData.twist.twist.linear.y;
    // odom.twist.twist.linear.z = OdomData.twist.twist.linear.z;

    // odomPub.publish(odom);

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.2)),
        ros::Time::now(),"base_link", "laser"));

    // ros::spinOnce(); 
    r.sleep();
  }
  // return 0;
}