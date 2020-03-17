/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_sensors/BodySensor.h>
#include <mc_sensors/ForceSensor.h>
#include <mc_tvm/Robots.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"

BOOST_AUTO_TEST_CASE(TestRobotLoading)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  BOOST_REQUIRE(rm != nullptr);
  auto clock = std::make_shared<tvm::Clock>(0.005);
  auto robot = mc_tvm::Robot{rm->name, clock, rm};
  BOOST_REQUIRE(robot.name() == "jvrc1");
  BOOST_REQUIRE(robot.mbc().q.size() == rm->mbc.q.size());
  BOOST_REQUIRE(robot.mbc().alpha.size() == rm->mbc.alpha.size());
  BOOST_REQUIRE(robot.mbc().alphaD.size() == rm->mbc.alphaD.size());
  BOOST_REQUIRE(&robot.module() == rm.get());

  BOOST_REQUIRE_THROW(mc_tvm::Robot("ThrowRobotURDF", clock, "/invalid/path", false, {}, {{}}),
                      mc_tvm::Robot::Exception);
  auto urdfRobot = mc_tvm::Robot{"RobotURDF", clock, rm->urdf_path, false, {}, rm->stance()};

  BOOST_REQUIRE(urdfRobot.name() == "RobotURDF");
  BOOST_REQUIRE(urdfRobot.mbc().q.size() == rm->mbc.q.size());
  BOOST_REQUIRE(urdfRobot.mbc().alpha.size() == rm->mbc.alpha.size());
  BOOST_REQUIRE(urdfRobot.mbc().alphaD.size() == rm->mbc.alphaD.size());
}

BOOST_AUTO_TEST_CASE(TestRobots)
{
  mc_tvm::Robots robots;
  BOOST_REQUIRE(robots.size() == 0);
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto clock = std::make_shared<tvm::Clock>(0.005);
  robots.create(rm->name, clock, rm);
  BOOST_REQUIRE(robots.size() == 1);
  BOOST_REQUIRE(robots.has("jvrc1"));
  BOOST_REQUIRE_NO_THROW(robots.robot());
  BOOST_REQUIRE(robots.robot().name() == "jvrc1");
  BOOST_REQUIRE_NO_THROW(robots.robot("jvrc1"));
  BOOST_REQUIRE(robots.robot("jvrc1").name() == "jvrc1");
  // Cannot create a robot with the same name
  BOOST_REQUIRE_THROW(robots.create(rm->name, clock, rm), mc_tvm::Robot::Exception);
  BOOST_REQUIRE(robots.size() == 1);

  auto robot2 = mc_tvm::Robot{"robot2", clock, rm->urdf_path, false, {}, rm->stance()};
  robots.add(robot2);
  BOOST_REQUIRE(robots.size() == 2);
  BOOST_REQUIRE(robots.has("jvrc1"));
  BOOST_REQUIRE_NO_THROW(robots.robot());
  BOOST_REQUIRE(robots.robot().name() == "jvrc1");
  BOOST_REQUIRE(robots.has("robot2"));
  BOOST_REQUIRE_NO_THROW(robots.robot("robot2"));
  BOOST_REQUIRE(robots.robot("robot2").name() == "robot2");

  // Removing the main robot is not allowed
  robots.remove("jvrc1");
  BOOST_REQUIRE(!robots.has("jvrc1"));
  robots.remove("robot2");
  BOOST_REQUIRE(!robots.has("robot2"));
  BOOST_REQUIRE(robots.size() == 0);
}

BOOST_AUTO_TEST_CASE(TestSensors)
{
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto clock = std::make_shared<tvm::Clock>(0.005);
  auto robot = mc_tvm::Robot(rm->name, clock, rm);

  auto & bs = robot.createSensor<mc_sensors::BodySensor>("TestBodySensor", "ParentBody", sva::PTransformd::Identity());
  BOOST_REQUIRE(robot.sensors().has("TestBodySensor"));
  BOOST_REQUIRE(bs.name() == "TestBodySensor");
  BOOST_REQUIRE(bs.parent() == "ParentBody");
  BOOST_REQUIRE(bs.parentToSensor() == sva::PTransformd::Identity());

  robot.createSensor<mc_sensors::ForceSensor>("TestForceSensor", "ParentBody", sva::PTransformd::Identity());
  BOOST_REQUIRE(robot.sensors().has("TestForceSensor"));
  auto & get_fs = robot.sensor<mc_sensors::ForceSensor>("TestForceSensor");
  BOOST_REQUIRE(get_fs.name() == "TestForceSensor");
  BOOST_REQUIRE(get_fs.parent() == "ParentBody");
  BOOST_REQUIRE(get_fs.parentToSensor() == sva::PTransformd::Identity());
  BOOST_REQUIRE(get_fs.wrench() == sva::ForceVecd::Zero());

  struct PepperSpeaker : mc_sensors::Sensor
  {
    PepperSpeaker(const std::string & name, const std::string & parent, const sva::PTransformd & pose)
    : Sensor(name, parent, pose)
    {
    }
    /** Return the text to say and reset the internal state */
    std::string consume()
    {
      std::string out = "";
      std::swap(out, text_);
      return out;
    }
    /** Set text to say */
    void say(const std::string & text)
    {
      text_ = text;
    }

  protected:
    std::string text_;
  };
  auto & speaker = robot.createSensor<PepperSpeaker>("PepperSpeaker", "ParentBody", sva::PTransformd::Identity());
  BOOST_REQUIRE(robot.sensors().has("PepperSpeaker"));
  speaker.say("Hello, world");
  BOOST_REQUIRE(speaker.consume() == "Hello, world");
}
