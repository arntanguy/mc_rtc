#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotInterface.h>

namespace mc_rbdyn
{
RobotInterface::RobotInterface(const mc_rbdyn::Robot & robot) : robot_(robot) {}

void RobotInterface::setProportionalGains(const std::vector<double> & pgains)
{
  if(pgains.size() != robot_.refJointOrder().size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[RobotInterface] Robot {} expects {} proportial gains, but {} were provided", robot_.name(),
        robot_.refJointOrder().size(), pgains.size());
  }
  for(size_t i = 0; i < pgains.size(); ++i)
  {
    setProportionalGain(i, pgains[i]);
  }
}

void RobotInterface::setIntegralGains(const std::vector<double> & igains)
{
  if(igains.size() != robot_.refJointOrder().size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[RobotInterface] Robot {} expects {} intergral gains, but {} were provided", robot_.name(),
        robot_.refJointOrder().size(), igains.size());
  }
  for(size_t i = 0; i < igains.size(); ++i)
  {
    setIntegralGain(i, igains[i]);
  }
}

void RobotInterface::setDerivativeGains(const std::vector<double> & dgains)
{
  if(dgains.size() != robot_.refJointOrder().size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[RobotInterface] Robot {} expects {} derivative gains, but {} were provided", robot_.name(),
        robot_.refJointOrder().size(), dgains.size());
  }
  for(size_t i = 0; i < dgains.size(); ++i)
  {
    setDerivativeGain(i, dgains[i]);
  }
}

} // namespace mc_rbdyn
