/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_posture_controller.h"

#include <mc_rtc/logging.h>

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
                                         double dt,

                                         const mc_rtc::Configuration & config,
                                         Backend backend)
: MCController(robot_module, dt, backend)
{

  if(config("ContactConstraint", true))
  {
    solver().addConstraintSet(std::make_shared<mc_solver::ContactConstraint>(dt));
  }
  if(config("KinematicsConstraint", true))
  {
    solver().addConstraintSet(std::make_shared<mc_solver::KinematicsConstraint>(robots(), 0, dt));
  }
  if(config("SelfCollisionConstraint", true))
  {
    solver().addConstraintSet(selfCollisionConstraint);
    auto selfColCstr = std::make_shared<mc_solver::CollisionsConstraint>(robots(), 0, 0, dt);
    selfColCstr->addCollisions(solver(), robot().module().minimalSelfCollisions());
    solver().addConstraintSet(selfColCstr);
  }
  if(config("CompoundJointConstraint", true))
  {
    solver().addConstraintSet(std::make_shared<mc_solver::CompoundJointConstraint>(robots(), 0, dt));
  }

  // Add the default posture task to the solver
  solver().addTask(postureTask.get());
  postureTask->stiffness(1.0);

  mc_rtc::log::success("MCPostureController init done");
}

bool MCPostureController::run()
{
  return mc_control::MCController::run();
}

} // namespace mc_control

MULTI_CONTROLLERS_CONSTRUCTOR("Posture",
                              mc_control::MCPostureController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "Posture_TVM",
                              mc_control::MCPostureController(rm, dt, config, mc_control::MCController::Backend::TVM))
