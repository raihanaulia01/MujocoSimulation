#include "control.hpp"
#include <iostream>

RobotController::RobotController(const mjModel *m) : m(m) {}

void RobotController::applyJointPos(mjData *d) const {
  for (int i = 0; i < jointCount; i++) {
    d->ctrl[i] = jointPos[i];
  }
}

void RobotController::checkJointLimits() {
  // check joint limits, stored in i*2 for min and i*2 + 1 for max
  for (int i = 0; i < jointCount; i++) {
    double jointMin = m->jnt_range[i*2];
    double jointMax = m->jnt_range[i*2 + 1];

    if (jointPos[i] > jointMax)
      jointPos[i] = jointMax;
    else if (jointPos[i] < jointMin)
      jointPos[i] = jointMin;
  }
}

void RobotController::printJointPos() {
  for (int i = 0; i < jointCount; i++) {
    std::cout << "Joint " << i << ": " << jointPos[i] << " rad\n";
  }
  std::cout << std::endl;
}