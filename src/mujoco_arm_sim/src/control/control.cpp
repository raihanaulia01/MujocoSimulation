#include "control.hpp"
#include <iostream>

RobotController::RobotController(const mjModel *m) : m(m) {}

void RobotController::applyJointPos(mjData *d) const {
  for (int i = 0; i < 4; i++) {
    d->ctrl[i] = jointPos[i];
  }
}

void RobotController::checkJointLimits() {
  // check joint limits, stored in i*2 for min and i*2 + 1 for max
  for (int i = 0; i < 4; i++) {
    double jointMin = m->jnt_range[i*2];
    double jointMax = m->jnt_range[i*2 + 1];

    if (jointPos[i] > jointMax)
      jointPos[i] = jointMax;
    else if (jointPos[i] < jointMin)
      jointPos[i] = jointMin;
  }
}

void RobotController::printJointPos() {
  std::cout << "Joint " << 1 << ": " << jointPos[0] << " rad\n";
  std::cout << "Joint " << 2 << ": " << jointPos[1] << " rad\n";
  std::cout << "Joint " << 3 << ": " << jointPos[2] << " rad\n";
  std::cout << "Joint " << 4 << ": " << jointPos[3] << " rad\n\n";
}