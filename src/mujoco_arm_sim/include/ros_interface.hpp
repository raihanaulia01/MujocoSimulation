#ifndef ROS_INTERFACE_H
#define ROS_INTERFACE_H

#include "control.hpp"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class RosInterface {
private:
  void commandCallback(const sensor_msgs::JointState::ConstPtr &message);
  ros::Subscriber commandSubscriber;
  ros::Publisher jointStatePublisher;
  RobotController &controller;
  const char *jointNames[4] = {"joint_1_hip", "joint_2_shoulder",
                               "joint_3_elbow", "joint_4_wrist"};

public:
  RosInterface(ros::NodeHandle &nodeHandle, RobotController &controller);
  void publishJointPos(const mjData *d);
};

#endif