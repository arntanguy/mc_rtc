#pragma once
#include <mc_rtc/Schema.h>

namespace mc_rbdyn::mujoco
{

// "RLEG_HIP_P": {
//       "actuator_type": "motor",
//       "nn_id": 2,
//       "id": 2,
//       "kp": "40.0",
//       "kd": "2.0",
//       "soft_torque_limit": "10.0"
//     },
struct JointMetadata
{
  MC_RTC_NEW_SCHEMA(JointMetadata)
  MC_RTC_SCHEMA_MEMBER(JointMetadata,
                       std::string,
                       actuatorType,
                       "Actuator type (actuator, motor, ...)",
                       mc_rtc::schema::ValueFlag::None,
                       "motor",
                       mc_rtc::schema::Choices{"motor", "actuator"})

#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(JointMetadata, __VA_ARGS__)
  MEMBER(double, kp, "Proportional gain")
  MEMBER(double, kd, "Derivative gain")
  MEMBER(double, softTorqueLimit, "Soft torque limit")
#undef MEMBER
};

struct ActuatorMetadata
{
  MC_RTC_NEW_SCHEMA(ActuatorMetadata)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata, std::string, type, "Actuator type", mc_rtc::schema::ValueFlag::None, "motor")
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata, double, gear, "Gear ratio", mc_rtc::schema::ValueFlag::None, 1.0)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata,
                       double,
                       ctrlrange_min,
                       "Control range min",
                       mc_rtc::schema::ValueFlag::None,
                       -1.0)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata,
                       double,
                       ctrlrange_max,
                       "Control range max",
                       mc_rtc::schema::ValueFlag::None,
                       1.0)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata,
                       double,
                       forcerange_min,
                       "Force range min",
                       mc_rtc::schema::ValueFlag::None,
                       -1.0)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata, double, forcerange_max, "Force range max", mc_rtc::schema::ValueFlag::None, 1.0)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata, bool, ctrllimited, "Control limited", mc_rtc::schema::ValueFlag::None, false)
  MC_RTC_SCHEMA_MEMBER(ActuatorMetadata, bool, forcelimited, "Force limited", mc_rtc::schema::ValueFlag::None, false)
};

struct GeomMetadata
{
  MC_RTC_NEW_SCHEMA(GeomMetadata)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, double, friction_slide, "Sliding friction", mc_rtc::schema::ValueFlag::None, 0.8)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, double, friction_spin, "Spinning friction", mc_rtc::schema::ValueFlag::None, 0.003)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, double, friction_roll, "Rolling friction", mc_rtc::schema::ValueFlag::None, 0.0001)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, double, margin, "Contact margin", mc_rtc::schema::ValueFlag::None, 0.001)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, int, group, "Collision group", mc_rtc::schema::ValueFlag::None, 0)
  MC_RTC_SCHEMA_MEMBER(GeomMetadata, std::string, material, "Material name", mc_rtc::schema::ValueFlag::None, "matgeom")
};

struct SiteMetadata
{
  MC_RTC_NEW_SCHEMA(SiteMetadata)
  MC_RTC_SCHEMA_MEMBER(SiteMetadata, std::string, name, "Site name", mc_rtc::schema::ValueFlag::None, "")
  MC_RTC_SCHEMA_MEMBER(SiteMetadata,
                       Eigen::Vector3d,
                       pos,
                       "Position",
                       mc_rtc::schema::ValueFlag::None,
                       Eigen::Vector3d::Zero())
  MC_RTC_SCHEMA_MEMBER(SiteMetadata,
                       Eigen::Vector4d,
                       quat,
                       "Orientation (quaternion)",
                       mc_rtc::schema::ValueFlag::None,
                       Eigen::Vector4d(1, 0, 0, 0))
  MC_RTC_SCHEMA_MEMBER(SiteMetadata,
                       Eigen::Vector3d,
                       size,
                       "Size",
                       mc_rtc::schema::ValueFlag::None,
                       Eigen::Vector3d::Zero())
  MC_RTC_SCHEMA_MEMBER(SiteMetadata, std::string, type, "Site type", mc_rtc::schema::ValueFlag::None, "sphere")
  MC_RTC_SCHEMA_MEMBER(SiteMetadata, int, group, "Site group", mc_rtc::schema::ValueFlag::None, 0)
};

struct MuJoCoMetadata
{
  std::map<std::string, JointMetadata> joint;
  std::map<std::string, ActuatorMetadata> actuator;
  std::map<std::string, GeomMetadata> geom;
  std::map<std::string, SiteMetadata> site;
};

}; // namespace mc_rbdyn::mujoco
