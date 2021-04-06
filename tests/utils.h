/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <mc_solver/QPSolver.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <tests_config.h>

template<typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA> & a,
    const Eigen::DenseBase<DerivedB> & b,
    const typename DerivedA::RealScalar & rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar & atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

inline bool configureRobotLoader()
{
  static bool done = false;
  if(!done)
  {
    done = true;
    mc_rtc::Loader::debug_suffix = "";
    mc_rbdyn::RobotLoader::clear();
    mc_rbdyn::RobotLoader::update_robot_module_path({std::string(ROBOTS_BUILD_DIR)});
  }
  return true;
}

inline mc_rbdyn::RobotsPtr makeRobots()
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));
  auto robots = std::make_shared<mc_rbdyn::Robots>();
  robots->load(*rm, rm->name);
  robots->load(*env, env->name);
  return robots;
}

inline std::unique_ptr<mc_solver::QPSolver> makeSolver()
{
  auto robots = makeRobots();
  auto realRobots = std::make_shared<mc_rbdyn::Robots>(*robots);
  auto solver = std::make_unique<mc_solver::QPSolver>(
      robots, realRobots, std::make_shared<mc_rtc::Logger>(mc_rtc::Logger::Policy::NON_THREADED, "", ""),
      std::make_shared<mc_rtc::GUI>(), 0.005);
  return solver;
}

/** This file contains various functions that are useful in unit tests */

#ifdef WIN32
#  include <Windows.h>
/** WIN32 port of mkstemp */
inline int mkstemp(char * out)
{
  char tmp_dir[MAX_PATH + 1];
  GetTempPath(MAX_PATH + 1, tmp_dir);
  int ret = GetTempFileName(tmp_dir, "mkstemp", 0, out);
  if(ret == 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
#endif

/** Return a temporary file */
inline std::string getTmpFile()
{
#ifndef WIN32
  char fIn[17] = "/tmp/tConfXXXXXX";
#else
  char fIn[MAX_PATH + 1];
  memset(fIn, 0, MAX_PATH + 1);
#endif
  int err = mkstemp(fIn);
  if(err < 0)
  {
    std::cerr << "Failed to create temporary file, abort test" << std::endl;
    throw std::runtime_error("Failed to create file");
  }
  return fIn;
}

/** Make a temporary configuration file from the content provided */
inline std::string makeConfigFile(const std::string & data, const std::string & ext = ".json")
{
  std::string fIn = getTmpFile() + ext;
  std::ofstream ofs(fIn);
  ofs << data;
  return fIn;
}

inline Eigen::Quaterniond random_quat()
{
  Eigen::Quaterniond q{Eigen::Vector4d::Random()};
  q.normalize();
  return q;
}

inline sva::PTransformd random_pt()
{
  Eigen::Vector3d t = Eigen::Vector3d::Random();
  Eigen::Quaterniond q = random_quat();
  return {q, t};
}

inline sva::ForceVecd random_fv()
{
  return {Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
}

inline sva::MotionVecd random_mv()
{
  return {Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
}

inline double rnd()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<double> dis(-100.0, 100.0);
  return dis(gen);
}

inline size_t random_size()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<size_t> dis(10, 1024);
  return dis(gen);
}

inline bool random_bool()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<> dis(0, 1);
  return dis(gen);
}

inline Eigen::Vector6d random_dof()
{
  Eigen::Vector6d out = Eigen::Vector6d::Zero();
  for(int i = 0; i < 6; ++i)
  {
    if(random_bool())
    {
      out(i) = 1.0;
    }
  }
  return out;
}

inline std::vector<double> random_vector()
{
  std::vector<double> v(random_size());
  for(size_t i = 0; i < v.size(); ++i)
  {
    v[i] = rnd();
  }
  return v;
}

inline char random_char()
{
  static const std::array<char, 70> chars = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D',
                                             'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R',
                                             'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                             'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',
                                             'u', 'v', 'w', 'x', 'y', 'z', '.', ':', '<', '>', ';', '-', '_', ' '};
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<size_t> dis(0, chars.size() - 1);
  return chars[dis(gen)];
}

inline std::string random_string()
{
  std::string s(random_size(), 'a');
  std::generate_n(s.begin(), s.size(), random_char);
  return s;
}
