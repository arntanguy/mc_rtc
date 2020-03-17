/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_sensors/api.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <string>

namespace mc_sensors
{

/** Bare-bone generic Sensor description
 *
 * Inherit from this class to create your own sensor.
 */
struct MC_SENSORS_DLLAPI Sensor
{
  Sensor() noexcept {}
  Sensor(const std::string & name, const std::string & parentBody, const sva::PTransformd & X_parent_sensor) noexcept
  : name_(name), parent_(parentBody), X_parent_sensor_(X_parent_sensor)
  {
  }

  inline void name(const std::string & name) noexcept
  {
    name_ = name;
  }

  inline const std::string & name() const noexcept
  {
    return name_;
  }

  inline void parent(const std::string & parent) noexcept
  {
    parent_ = parent;
  }

  inline const std::string & parent() const noexcept
  {
    return parent_;
  }

  inline void parentToSensor(const sva::PTransformd & X_parent_sensor) noexcept
  {
    X_parent_sensor_ = X_parent_sensor;
  }

  const sva::PTransformd & parentToSensor() const noexcept
  {
    return X_parent_sensor_;
  }

protected:
  std::string name_;
  std::string parent_;
  sva::PTransformd X_parent_sensor_ = sva::PTransformd::Identity();
};

} // namespace mc_sensors
