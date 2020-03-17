#include <mc_tvm/Robots.h>

namespace mc_tvm
{

Robot & Robots::robot(const std::string & name)
{
  if(!has(name))
  {
    LOG_ERROR_AND_THROW(Robot::Exception, "No robot named " << name);
  }
  return robots_[indexByName_.at(name)];
}

const Robot & Robots::robot(const std::string & name) const
{
  if(!has(name))
  {
    LOG_ERROR_AND_THROW(Robot::Exception, "No robot named " << name);
  }
  return robots_[indexByName_.at(name)];
}

Robot & Robots::robot()
{
  if(robots_.empty())
  {
    LOG_ERROR_AND_THROW(Robot::Exception, "No main robot loaded, check your \"MainRobot\" configuration");
  }
  return robots_.front();
}

const Robot & Robots::robot() const
{
  if(robots_.empty())
  {
    LOG_ERROR_AND_THROW(Robot::Exception, "No main robot loaded, check your \"MainRobot\" configuration");
  }
  return robots_.front();
}

void Robots::add(const Robot & robot)
{
  if(has(robot.name()))
  {
    LOG_ERROR_AND_THROW(Robot::Exception,
                        "Attempted to add a robot named "
                            << robot.name()
                            << " but a robot with the same name already exists. Please rename your robot.");
  }
  robots_.push_back(robot);
  indexByName_[robots_.back().name()] = robots_.size() - 1;
}

void Robots::remove(const std::string & name)
{
  if(!has(name))
  {
    LOG_WARNING("No robot named " << name << " to remove.");
    return;
  }
  robots_.erase(std::next(robots_.begin(), indexByName_.at(name)));
  indexByName_.erase(name);
}

Robots::iterator Robots::begin() noexcept
{
  return robots_.begin();
}

Robots::const_iterator Robots::begin() const noexcept
{
  return robots_.begin();
}

Robots::const_iterator Robots::cbegin() const noexcept
{
  return robots_.cbegin();
}

Robots::iterator Robots::end() noexcept
{
  return robots_.end();
}

Robots::const_iterator Robots::end() const noexcept
{
  return robots_.end();
}

Robots::const_iterator Robots::cend() const noexcept
{
  return robots_.cend();
}

Robots::reverse_iterator Robots::rbegin() noexcept
{
  return robots_.rbegin();
}

Robots::const_reverse_iterator Robots::rbegin() const noexcept
{
  return robots_.rbegin();
}

Robots::const_reverse_iterator Robots::crbegin() const noexcept
{
  return robots_.crbegin();
}

Robots::reverse_iterator Robots::rend() noexcept
{
  return robots_.rend();
}

Robots::const_reverse_iterator Robots::rend() const noexcept
{
  return robots_.rend();
}

Robots::const_reverse_iterator Robots::crend() const noexcept
{
  return robots_.crend();
}

} // namespace mc_tvm
