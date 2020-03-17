/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_tvm/Robot.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"

BOOST_AUTO_TEST_CASE(TestRobotLoading)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  BOOST_REQUIRE(rm != nullptr);
  tvm::Clock clock{0.005};
  auto robot = mc_tvm::Robot{clock, rm};
  BOOST_REQUIRE(robot.name() == "jvrc1");
  BOOST_REQUIRE(robot.mbc().q.size() == rm->mbc.q.size());
  BOOST_REQUIRE(robot.mbc().alpha.size() == rm->mbc.alpha.size());
  BOOST_REQUIRE(robot.mbc().alphaD.size() == rm->mbc.alphaD.size());
  BOOST_REQUIRE(&robot.module() == rm.get());

  auto urdfRobot = mc_tvm::Robot{clock, "RobotURDF", rm->urdf_path, false, {}, rm->stance()};

  LOG_INFO("name " << urdfRobot.name());
  BOOST_REQUIRE(urdfRobot.name() == "RobotURDF");
  BOOST_REQUIRE(urdfRobot.mbc().q.size() == rm->mbc.q.size());
  BOOST_REQUIRE(urdfRobot.mbc().alpha.size() == rm->mbc.alpha.size());
  BOOST_REQUIRE(urdfRobot.mbc().alphaD.size() == rm->mbc.alphaD.size());
}
