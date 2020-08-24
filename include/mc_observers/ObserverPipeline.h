/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
#include <mc_observers/api.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace mc_rbdyn
{
struct Robots;
}

namespace mc_observers
{

/**
 * @brief State observation pipeline
 *
 * Observers are responsible for estimating some of the robot properties from sensor measurements and/or fusing several
 * source of information (e.g the EncoderObserver estimates the joint position and velocity based on joint sensors, the
 * KinematicInertialObservers the state of the floating base from kinematics and IMU information, BodySensorObserver
 * uses BodySensor information to update the floating base, etc). The ObserverPipeline groups them into a \"State
 * observation pipeline\", that will run each observer sequentially. Some observers may be used to update the state of
 * the real robots instances used by the controller, while others may only be used for logging estimated values for
 * comparison purposes.
 */
struct MC_OBSERVERS_DLLAPI ObserverPipeline
{
  ObserverPipeline(mc_control::MCController & ctl, const std::string & name);
  ObserverPipeline(mc_control::MCController & ctl);
  virtual ~ObserverPipeline() = default;

  /* Load the observers */
  void create(const mc_rtc::Configuration & config, double dt);

  /* Initialize based on the current robot state */
  void reset();

  /* Run this observservation pipeline
   *
   * If an observer is unable to estimate the robot's state, it is expected to
   * return false. In this case, the pipeline execution is considered invalid,
   * and this status is reflected by the return value of this function. This
   * state may later be retrieved by success().
   *
   * @return True when the pipeline exectued properly
   * False otherwise (one or more observers failed to execute)
   **/
  bool run();

  /** @return True if the observers are running */
  inline bool runObservers() const
  {
    return runObservers_;
  }

  /**
   * @brief Whether to run the observers in this pipeline
   *
   * @param status True if the observers should be run
   */
  inline void runObservers(bool status)
  {
    runObservers_ = status;
  }

  /** @return True if the observers are updating the real robots instance. The
   * update does not occur if runObservers() is false. */
  inline bool updateObservers() const
  {
    return updateObservers_;
  }

  /**
   * @brief Whether to update the observers in this pipeline
   *
   * @param status True if the real robot instances should be update from the
   * observers's result. Update occurs only if runObservers() is true, and the
   * observer succeeded.
   */
  inline void updateObservers(bool status)
  {
    updateObservers_ = status;
  }

  /**
   * @brief Checks whether the last run of the pipeline succeeded
   *
   * @return True when the last call to run() succeeded
   */
  bool success() const
  {
    return success_;
  }

  /* Const accessor to an observer
   *
   * @param name Name of the observer
   * @throws std::runtime_error if the observer is not part of the pipeline
   */
  const Observer & observer(const std::string & name) const
  {
    auto it = std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                           [&name](const PipelineObserver & obs) { return obs.observer->name() == name; });
    if(it == pipelineObservers_.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Observer pipeline \"{}\" does not have any observer named \"{}\"", name_, name);
    }
    return *it->observer;
  }

  /**
   * @brief Checks whether this pipeline has an observer
   *
   * @param name Name of the observer
   *
   * @return True if the observer is in the pipeline
   */
  bool hasObserver(const std::string & name) const
  {
    return std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                        [&name](const PipelineObserver & obs) { return obs.observer->name() == name; })
           != pipelineObservers_.end();
  }

  /**
   * @brief Checks if there is an observer of a specific type in the pipeline
   *
   * There may be more than one
   *
   * @param type Type of the observer
   *
   * @return True if there is at least one observer of this type in the pipeline
   */
  bool hasObserverType(const std::string & type) const
  {
    return std::find_if(pipelineObservers_.begin(), pipelineObservers_.end(),
                        [&type](const PipelineObserver & obs) { return obs.observer->type() == type; })
           != pipelineObservers_.end();
  }

  const std::vector<mc_observers::ObserverPtr> observers() const
  {
    std::vector<mc_observers::ObserverPtr> observers;
    std::transform(pipelineObservers_.begin(), pipelineObservers_.end(), std::back_inserter(observers),
                   [](const PipelineObserver & pObs) { return pObs.observer; });
    return observers;
  }

  /* Non-const variant */
  Observer & observer(const std::string & name)
  {
    return const_cast<Observer &>(static_cast<const ObserverPipeline *>(this)->observer(name));
  }

  /*! \brief Short description of the pipeline */
  virtual const std::string & desc() const
  {
    return desc_;
  }

  /* Name used to identify this pipeline */
  const std::string & name() const
  {
    return name_;
  }

  void addToLogger(mc_rtc::Logger &);
  void removeFromLogger(mc_rtc::Logger &);
  void addToGUI(mc_rtc::gui::StateBuilder &);
  void removeFromGUI(mc_rtc::gui::StateBuilder &);

protected:
  mc_control::MCController & ctl_;
  std::string name_ = {"DefaultObserverPipeline"}; ///< Name of this pipeline
  /* Short descriptive description of the observer used for CLI logging */
  std::string desc_ = {""};
  bool runObservers_ = true; ///< Whether to run this pipeline
  bool updateObservers_ = true; ///< Whether to update real robots from estimated state.
  bool success_ = false; ///< Whether the pipeline successfully executed

  struct PipelineObserver
  {
    PipelineObserver(const mc_observers::ObserverPtr & observer, const mc_rtc::Configuration & config)
    : observer(observer)
    {
      config("update", update);
      config("log", log);
      config("gui", gui);
    }

    mc_observers::ObserverPtr observer = nullptr; //< Observer
    bool update = true; //< Whether to update the real robot instance from this observer
    bool log = true; //< Whether to log this observer
    bool gui = true; //< Whether to display the gui
    bool success = true; //< Whether this observer succeeded
  };

  /** Observers that will be run by the pipeline.
   *
   * The pair contains:
   * - The observer to run
   * - A boolean set to true if the observer updates the real robot instance
   *
   * Provided by MCGlobalController */
  std::vector<PipelineObserver> pipelineObservers_;
};

} // namespace mc_observers
