// Author(s): Kelvin Kang (kelvinkang@cmu.edu)
// This file is subject to the terms and conditions defined in the file 'LICENSE',
// which is part of this source code package

// include in the cpp node file should only be the .h file
// any additional include should go to .h file
#include "simple_node.h"

// convention is to put Constructor at the very top
SimpleNode::SimpleNode() : private_nh_("~")
{
  // put boiler plate ROS stuff in this function
  initRos();

  algo = new howde::MyAlgo(myVar1_);

  // initialise variables if you need to and add explanation to magic numbers
  myVar3_ = 5;
  myVar4_ = 10;
}

// convention is to put the most important function right after the constructor
void SimpleNode::run(double currVel_, sensor_msgs::Imu imu,
                  geometry_msgs::TwistWithCovarianceStamped& vehicleCmdMsg, int myVar1, double myVar4)
{
  // do really cool and important stuff here
  double x = algo->run(myVar1, myVar3_);

  // publish the result of your calculation / algo
  vehicleCmdMsg_.twist.twist.linear.x = currVel_;
  vehicleCmdMsg_.twist.twist.angular.z = 2.0;
  vehicleCmdPub_.publish(vehicleCmdMsg_);

  vehicleStatsMsg_.data = "Hello World";
  vehicleStatsPub_.publish(vehicleStatsMsg_);
}

// put boiler plate ros function at the bottom
void SimpleNode::timerCallback(const ros::TimerEvent& e)
{
  this->run(currVel_, currImu_, vehicleCmdMsg_, myVar1_, myVar4_);
}

void SimpleNode::odomCallback(const nav_msgs::Odometry& odomMsg)
{
  currVel_ = odomMsg.twist.twist.linear.x;
}

void SimpleNode::imuCallback(const sensor_msgs::Imu& imuMsg)
{
  currImu_ = imuMsg;
}

void SimpleNode::initRos()
{
  // param syntax is: param name in launch, variable name in source code, and default value
  private_nh_.param<int>("myVar1_", myVar1_, 1);
  private_nh_.param<double>("myVar2_", myVar2_, 5.0);
  private_nh_.param<double>("timerFreq_", timerFreq_, 20.0);  // Hz

  // set subscriber
  odomSub_ = nh_.subscribe("/odom", 1, &SimpleNode::odomCallback, this);
  imuSub_ = nh_.subscribe("/imu/data", 1, &SimpleNode::imuCallback, this);

  // set publisher
  vehicleCmdPub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/vehicle_cmd", 5);
  vehicleStatsPub_ = nh_.advertise<std_msgs::String>("/vehicle_status", 10);

  // set timers
  timer_ = nh_.createTimer(ros::Rate(timerFreq_), &SimpleNode::timerCallback, this);
}

// keep the main function as clean as possible
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_node");
  SimpleNode node;
  while (ros::ok())
    ros::spinOnce();
  return 0;
}