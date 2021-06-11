/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/Robots.h>

namespace mc_rbdyn
{

GripperSurface::GripperSurface(std::string_view name,
                               RobotFramePtr frame,
                               const std::vector<sva::PTransformd> & pointsFromOrigin,
                               const sva::PTransformd & X_b_motor,
                               double motorMaxTorque)
: Surface(name, frame), pointsFromOrigin_(pointsFromOrigin), X_b_motor_(X_b_motor), motorMaxTorque_(motorMaxTorque)
{
  points_.clear();
  for(sva::PTransformd & p : pointsFromOrigin_)
  {
    points_.push_back(p * frame->X_b_f());
  }
  if(points_.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Cannot create gripper surface {} for {} as it would have 0 points", name, frame->robot().name());
  }
}

std::string GripperSurface::type() const noexcept
{
  return "gripper";
}

std::shared_ptr<Surface> GripperSurface::copy(Robot & to) const
{
  if(!to.hasFrame(name()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No frame {} in destination robot {}", name(), to.name());
  }
  return std::make_shared<GripperSurface>(name(), to.frame(name()), pointsFromOrigin_, X_b_motor_, motorMaxTorque_);
}

} // namespace mc_rbdyn
