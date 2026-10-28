#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "geometry_msgs/PointStamped.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <eigen3/Eigen/Dense>
#include <mujoco/mujoco.h>
#include <string>
#include <vector>

class KinematicsNode {
private:
  std::vector<std::string> jointNames;
  std::vector<double> current_q;

  ros::Subscriber jointStateSub;
  ros::Publisher jointCmdPublisher;
  ros::Subscriber targetPointSub;
  ros::Publisher pointPublisher;

  Eigen::Matrix4d getDHMatrix(double a, double alpha, double d, double theta);
  std::vector<Eigen::Matrix4d> getTransforms(const std::vector<double> &q);
  Eigen::MatrixXd getJacobian(const std::vector<Eigen::Matrix4d> &T);

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void targetPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

public:
  KinematicsNode(ros::NodeHandle &nh);
};

#endif