/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_posture_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  solver().addConstraint(kinematicsConstraint_);
  solver().addConstraint(collisionConstraint_);
  // FIXME
  // solver().addConstraint(compoundJointConstraint_);

  postureTask_->stiffness(5.0);
  solver().addTask(postureTask_);

  mc_rtc::log::success("Posture sample controller initialized");
}

bool MCPostureController::run()
{
  return mc_control::MCController::run();
}

} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Posture", mc_control::MCPostureController)
