/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_tvm/Robots.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"

BOOST_AUTO_TEST_CASE(TestRobotLoading)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  BOOST_REQUIRE(rm != nullptr);
  auto clock = std::make_shared<tvm::Clock>(0.005);
  auto robot = mc_tvm::Robot{clock, rm};
  BOOST_REQUIRE(robot.name() == "jvrc1");
  BOOST_REQUIRE(robot.mbc().q.size() == rm->mbc.q.size());
  BOOST_REQUIRE(robot.mbc().alpha.size() == rm->mbc.alpha.size());
  BOOST_REQUIRE(robot.mbc().alphaD.size() == rm->mbc.alphaD.size());
  BOOST_REQUIRE(&robot.module() == rm.get());

  BOOST_REQUIRE_THROW(mc_tvm::Robot(clock, "ThrowRobotURDF", "/invalid/path", false, {}, {{}}),
                      mc_tvm::Robot::Exception);
  auto urdfRobot = mc_tvm::Robot{clock, "RobotURDF", rm->urdf_path, false, {}, rm->stance()};

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
  robots.create(clock, rm);
  BOOST_REQUIRE(robots.size() == 1);
  BOOST_REQUIRE(robots.has("jvrc1"));
  BOOST_REQUIRE_NO_THROW(robots.robot());
  BOOST_REQUIRE(robots.robot().name() == "jvrc1");
  BOOST_REQUIRE_NO_THROW(robots.robot("jvrc1"));
  BOOST_REQUIRE(robots.robot("jvrc1").name() == "jvrc1");

  auto robot2 = mc_tvm::Robot{clock, "robot2", rm->urdf_path, false, {}, rm->stance()};
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
