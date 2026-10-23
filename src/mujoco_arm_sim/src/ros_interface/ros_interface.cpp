#include "ros_interface.hpp"

RosInterface::RosInterface(ros::NodeHandle &nodeHandle,
                           RobotController &controller)
    : controller(controller) {
  commandSubscriber = nodeHandle.subscribe(
      "/joint_command", 100, &RosInterface::commandCallback, this);
  
  jointStatePublisher = nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 100);
}

void RosInterface::commandCallback(const sensor_msgs::JointState::ConstPtr &message) {
  if (message->position.size() != 4) {
    ROS_WARN("Expected 4 joint positions, received %zu instead.", message->position.size());
    return;
  }

  for (int i = 0; i < 4; i++) {
    controller.jointPos[i] = message->position[i];
  }

  controller.checkJointLimits();
}

void RosInterface::publishJointPos(const mjData *d) {
  sensor_msgs::JointState message;
  for (int i = 0; i < 4; i++) {
    message.name.push_back(jointNames[i]);
    message.position.push_back(d->qpos[i]);
  }

  jointStatePublisher.publish(message);
}