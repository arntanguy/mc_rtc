#pragma once
#include <mc_rbdyn/api.h>
#include <vector>

namespace mc_rbdyn
{

struct Robot;

/**
 * @brief Interface to the robot's low-level control
 */
struct MC_RBDYN_DLLAPI RobotInterface
{
  RobotInterface(const mc_rbdyn::Robot & robot);
  virtual ~RobotInterface() noexcept = default;

  void setProportionalGains(const std::vector<double> & pgains);
  virtual void setProportionalGain(size_t jidx, double pgain) = 0;

  void setIntegralGains(const std::vector<double> &);
  virtual void setIntegralGain(size_t jidx, double pgain) = 0;

  void setDerivativeGains(const std::vector<double> &);
  virtual void setDerivativeGain(size_t jidx, double pgain) = 0;

  virtual const std::vector<double> & getProportionalGains() const = 0;
  virtual const std::vector<double> & getIntegralGains() const = 0;
  virtual const std::vector<double> & getDerivativeGains() const = 0;

  virtual void setPowerStatus(size_t joint, bool poweron) = 0;
  virtual const std::vector<bool> & getPowerStatus() const = 0;
  virtual void setServoStatus(size_t joint, bool servoon) = 0;
  virtual const std::vector<bool> & getServoStatus() const = 0;

protected:
  const mc_rbdyn::Robot & robot_;
};

} // namespace mc_rbdyn
