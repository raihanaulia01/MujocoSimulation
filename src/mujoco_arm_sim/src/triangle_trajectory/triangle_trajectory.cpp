#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

struct Segment {
  std::array<double, 5> start;
  std::array<double, 5> end;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "triangle_trajectory");
  ros::NodeHandle nodeHandle("~");
  ros::Publisher commandPublisher =
      nodeHandle.advertise<sensor_msgs::JointState>("/joint_command", 100);

  sensor_msgs::JointState command;
  command.name = {"joint_1_hip", "joint_2_shoulder", "joint_3_elbow",
                  "joint_4_wrist", "joint_5_roll"};

  std::array<double, 5> point1 = {0.00, 0.00, -0.63, -0.90, 0.00};
  std::array<double, 5> point2 = {0.00, 0.75, -1.96, -0.37, 0.00};
  std::array<double, 5> traject2 = {0.00, 0.07, -1.56, -0.06, 0.00};
  std::array<double, 5> point3 = {0.00, -0.60, -0.71, 0.06, 0.00};
  std::array<double, 5> traject3 = {0.00, -0.17, -0.91, -0.06, 0.00};

  // trajectory:
  // point1->point2, point2->traject2,
  // traject2->point3, point3->traject3, traject3->point1.
  const std::vector<Segment> trajectory = {{point1, point2},
                                           {point2, traject2},
                                           {traject2, point3},
                                           {point3, traject3},
                                           {traject3, point1}};

  const int duration = 1;
  const double rosRate = 100.0;
  ros::Rate rate(rosRate);

  while (ros::ok()) {
    for (const Segment &segment : trajectory) {
      // interpolation
      const int steps = duration * (int)rosRate;
      command.position.resize(5);
      for (int step = 1; step <= steps; step++) {
        double fraction = (double)step / (double)steps;

        for (int j = 0; j < 5; j++) {
          double interpolatedPos =
              segment.start[j] + fraction * (segment.end[j] - segment.start[j]);
          command.position[j] = interpolatedPos;
        }
        commandPublisher.publish(command);
        ros::spinOnce();
        rate.sleep();
      }
    }
  }
}