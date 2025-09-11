/*
 * Copyright 2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */
#include <mc_rbdyn/MuJoCo.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>

#include <boost/test/unit_test.hpp>
#include <fstream>

BOOST_AUTO_TEST_CASE(TestRobotModuleMuJoCo)
{
  auto jointMetaData = mc_rbdyn::mujoco::JointMetadata{};
  jointMetaData.actuatorType = "actuator";

  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  tinyxml2::XMLDocument doc;
  auto mujoco_elem = rm->toMCJF(doc);
  doc.InsertEndChild(mujoco_elem);

  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);

  std::cout << printer.CStr() << std::endl;
  std::ofstream out("/tmp/mujoco_generated.xml");
  if(out.is_open())
  {
    out << printer.CStr();
    out.close();
    std::cout << "MuJoCo XML written to /tmp/mujoco_generated.xml" << std::endl;
  }
  else { std::cerr << "Failed to open /tmp/mujoco_generated.xml for writing" << std::endl; }
}
