#include "kinematics_node.hpp"
#include <cmath>

KinematicsNode::KinematicsNode(ros::NodeHandle &nh) {
  current_q.resize(5, 0.0);
  jointNames = {"joint_1_hip", "joint_2_shoulder", "joint_3_elbow",
                "joint_4_wrist", "joint_5_roll"};

  jointStateSub = nh.subscribe("/joint_states", 1,
                               &KinematicsNode::jointStateCallback, this);
  targetPointSub = nh.subscribe("/target_point", 1,
                                &KinematicsNode::targetPointCallback, this);
  jointCmdPublisher =
      nh.advertise<sensor_msgs::JointState>("/joint_command", 100);
  pointPublisher =
      nh.advertise<geometry_msgs::PointStamped>("/end_effector", 100);
}

Eigen::Matrix4d KinematicsNode::getDHMatrix(double a, double alpha, double d,
                                            double theta) {
  Eigen::Matrix4d T;
  T << std::cos(theta), -std::sin(theta) * std::cos(alpha),
      std::sin(theta) * std::sin(alpha), a * std::cos(theta), std::sin(theta),
      std::cos(theta) * std::cos(alpha), -std::cos(theta) * std::sin(alpha),
      a * std::sin(theta), 0, std::sin(alpha), std::cos(alpha), d, 0, 0, 0, 1;
  return T;
}

std::vector<Eigen::Matrix4d>
KinematicsNode::getTransforms(const std::vector<double> &q) {
  std::vector<Eigen::Matrix4d> T(6);
  // DH Parameters:
  // a       alpha   d       tetha
  // 0       -90     0.15    q1
  // 0.45    0       -0.05   q2 - 90
  // 0.45    0       0       q3
  // 0       90      -0.1    q4 + 90
  // 0       0       0.05    q5
  
  T[0] = Eigen::Matrix4d::Identity(); // base
  T[1] = T[0] * getDHMatrix(0.0, -M_PI_2, 0.15, q[0]);
  T[2] = T[1] * getDHMatrix(0.45, 0, -0.05, q[1] - M_PI_2);
  T[3] = T[2] * getDHMatrix(0.45, 0, 0, q[2]);
  T[4] = T[3] * getDHMatrix(0.0, M_PI_2, -0.1, q[3] + M_PI_2);
  T[5] = T[4] * getDHMatrix(0.0, 0, 0.05, q[4]);

  return T;
}

void KinematicsNode::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr &msg) {
  if (msg->position.size() < 5)
    return;

  for (int i = 0; i < 5; i++) {
    current_q[i] = msg->position[i];
  }

  std::vector<Eigen::Matrix4d> T = getTransforms(current_q);

  geometry_msgs::PointStamped pointMsg;
  pointMsg.point.x = T[5](0, 3);
  pointMsg.point.y = T[5](1, 3);
  pointMsg.point.z = T[5](2, 3);

  pointPublisher.publish(pointMsg);
}

Eigen::MatrixXd
KinematicsNode::getJacobian(const std::vector<Eigen::Matrix4d> &T) {
  // 3x5 (xyz, joints 1-5) because this project has a 5DOF robot.
  // Using 6x5 or a full positional and rotational jacobian will
  // often fail to solve.
  Eigen::MatrixXd J(3, 5);

  // slice a 3x1 submatrix at position 0,3,
  // which is the position of the end effector
  Eigen::Vector3d On = T[5].block<3, 1>(0, 3);

  for (int i = 0; i < 5; i++) {
    Eigen::Vector3d zi = T[i].block<3, 1>(0, 2); // z axis
    Eigen::Vector3d Oi = T[i].block<3, 1>(0, 3); // position
    J.col(i) = zi.cross(On - Oi);
  }
  return J;
}

void KinematicsNode::targetPointCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  Eigen::Vector3d targetPos(msg->point.x, msg->point.y, msg->point.z);
  std::vector<double> q = current_q;

  const int maxIterations = 100;
  const double tolerance = 1e-5;
  const double damping = 0.1;

  for (int i = 0; i < maxIterations; i++) {
    std::vector<Eigen::Matrix4d> T = getTransforms(q);

    // slice a 3x1 submatrix at position 0,3,
    // which is the position of the end effector
    Eigen::Vector3d currentPos = T[5].block<3, 1>(0, 3);
    Eigen::Vector3d error = targetPos - currentPos;

    if (error.norm() < tolerance)
      break;

    Eigen::MatrixXd J = getJacobian(T);

    Eigen::MatrixXd Jt = J.transpose();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);

    // DLS: delta q = Jt * (J*Jt + damping^2 * I)^-1 * error
    Eigen::MatrixXd delta_q =
        (Jt * (J * Jt + (damping * damping) * I).inverse()) * error;

    for (int j = 0; j < 5; j++) {
      q[j] += delta_q(j);
    }
  }

  sensor_msgs::JointState commandMsg;
  commandMsg.name = jointNames;
  commandMsg.position = q;
  jointCmdPublisher.publish(commandMsg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinematics_node");
  ros::NodeHandle nh;
  KinematicsNode node(nh);
  ros::spin();
  return 0;
}