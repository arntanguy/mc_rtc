#pragma once
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

struct MC_TVM_DLLAPI Robots
{
  /** @name Iterators
   *
   * These functions provide an iterator interface to Robots
   *
   * @{
   */
  using iterator = typename std::vector<Robot>::iterator;
  using const_iterator = typename std::vector<Robot>::const_iterator;
  using reverse_iterator = typename std::vector<Robot>::reverse_iterator;
  using const_reverse_iterator = typename std::vector<Robot>::const_reverse_iterator;
  using size_type = typename std::vector<Robot>::size_type;
  /** @} */

  Robots() noexcept = default;
  Robots(const Robots & rhs) noexcept = default;
  Robots & operator=(const Robots & rhs) noexcept = default;

  /** Access a robot by name
   *
   * @param name Robot name
   *
   * @return Reference to the robot if name is valid
   *
   * @throws std::runtime_error If the robot does not exist
   */
  Robot & robot(const std::string & name);

  /** Const-access to a robot by name
   * \see Robot & robot(const std::string & name);
   */
  const Robot & robot(const std::string & name) const;

  /** Access the main robot
   *
   * @return Main robot if it exists
   *
   * @throws RobotException if the robot does not exist
   */
  Robot & robot();

  /** Const-access to the main robot
   * \see Robot & robot();
   */
  const Robot & robot() const;

  /** Access the loaded robots
   *
   * @return Reference to the vector of loaded robots
   */
  inline std::vector<Robot> & robots()
  {
    return robots_;
  }

  /** Const-access the loaded robots
   *
   * @return Const-reference to the vector of loaded robots
   */
  inline const std::vector<Robot> & robots() const
  {
    return robots_;
  }

  /** Adds an existing robot
   * Robot names must be unique, this function will throw if you attempt to add
   * a robot whose name matches an exising one.
   *
   * @param robot Robot to copy
   * @warns RobotException if a robot with the same name already exists
   */
  void add(const Robot & robot);

  /** Removes a robot
   *
   * The robot name must exist, and removing the main robot is not allowed. If
   * you attempt to remove either of those, it'll warn and do nothing.
   *
   * @param name Name of the robot to remove
   */
  void remove(const std::string & name);

  /** Creates a robot in place
   *
   * This is more efficient then calling add(const Robot & robot)
   *
   * @param args Arguments to pass to the Robot contructor
   * @return Reference to the newly created robot
   * @throws If a robot with the same name already exists
   */
  template<typename... ArgsT>
  Robot & create(const std::string & name, const std::shared_ptr<Clock> & clock, ArgsT &&... args)
  {
    if(has(name))
    {
      LOG_ERROR_AND_THROW(Robot::Exception, "Failed to create robot named "
                                                << name << " as another robot with the same name already exists");
    }
    auto & r = robots_.emplace_back(name, clock, std::forward<ArgsT>(args)...);
    indexByName_[r.name()] = robots_.size() - 1;
    return r;
  }

  /** Checks whether a robot exists
   *
   * @param name Name of the robot
   *
   * @return True if the robot exists, false otherwise
   */
  inline bool has(const std::string & name) const noexcept
  {
    return indexByName_.count(name);
  }

  /** @name Iterators
   *
   * These functions provide an iterator interface to Robots
   *
   * @{
   */
  iterator begin() noexcept;
  const_iterator begin() const noexcept;
  const_iterator cbegin() const noexcept;

  iterator end() noexcept;
  const_iterator end() const noexcept;
  const_iterator cend() const noexcept;

  reverse_iterator rbegin() noexcept;
  const_reverse_iterator rbegin() const noexcept;
  const_reverse_iterator crbegin() const noexcept;

  reverse_iterator rend() noexcept;
  const_reverse_iterator rend() const noexcept;
  const_reverse_iterator crend() const noexcept;
  /** @} */

  /** Number of robots
   *
   * @return The number of robots
   */
  inline size_type size() const noexcept
  {
    return robots_.size();
  }

protected:
  // Vector of loaded robots
  std::vector<Robot> robots_;
  // Map of robot name -> index in robots_ map
  std::map<std::string, size_t> indexByName_;
};
} // namespace mc_tvm
