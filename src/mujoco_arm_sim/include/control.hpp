#ifndef CONTROL_H
#define CONTROL_H

#include <mujoco/mujoco.h>

const int jointCount = 5;

class RobotController {
private:
  const mjModel *m;
public:
  double jointPos[jointCount] = {0, 0, 0, 0, 0};
  RobotController(const mjModel *m);

  void checkJointLimits();
  void applyJointPos(mjData *d) const;
  void printJointPos();
};

#endif