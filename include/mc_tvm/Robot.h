/*
 * Copyright 2017-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * Note: Large portions of this file were originally developped as part of the
 * TVM library.
 */

#pragma once
#include <mc_tvm/api.h>
#include <tvm/Clock.h>
#include <tvm/Variable.h>
#include <tvm/VariableVector.h>
#include <tvm/graph/abstract/Node.h>

#include <RBDyn/FD.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_tvm
{
using VariableVector = tvm::VariableVector;
using VariablePtr = tvm::VariablePtr;
using Clock = tvm::Clock;
namespace graph = tvm::graph;

/** Represent a Robot
 *
 * A robot is constructed by providing instances of MultiBodyGraph, MultiBody
 * and MultiBodyConfig that are created from each other. It provides signals
 * that are relevant for computing quantities related to a robot.
 *
 * Variables:
 * - q (split between free-flyer and joints)
 * - tau (see Outputs)
 *
 * Individual outputs:
 *
 * - FK: forward kinematics (computed by RBDyn::FK)
 * - FV: forward velocity (computed by RBDyn::FV), depends on FK
 * - FA: forward acceleration (computed by RBDyn::FA), depends on FV
 * - NormalAcceleration: update bodies' normal acceleration, depends on FA
 * - tau: generalized torque vector, this output isn't currently linked to
 *   any computation
 * - CoM: center of mass signal, depends on FK
 * - H: inertia matrix signal, depends on FV
 * - C: non-linear effect vector signal (Coriolis, gravity, external forces), depends on FV
 *
 * Meta outputs:
 *   These outputs are provided for convenience sake
 * - Geometry: depends on CoM (i.e. CoM + FK)
 * - Dynamics: depends on FA + normalAcceleration (i.e. everything)
 *
 */
struct MC_TVM_DLLAPI Robot : public graph::abstract::Node<Robot>
{
public:
  SET_OUTPUTS(Robot, FK, FV, FA, NormalAcceleration, tau, CoM, H, C, Geometry, Dynamics)
  SET_UPDATES(Robot, Time, FK, FV, FA, NormalAcceleration, CoM, H, C)

  /** Constructs a new robot instance
   *
   * \param clock Clock used in the ControlProblem
   * \param module RobotModule from which the Robot instance will be loaded
   */
  Robot(Clock & clock, std::shared_ptr<mc_rbdyn::RobotModule> module);

  /** Consts a new robot instance
   *
   * - The initial stance is initialized from RobotModule::stance()
   * - Velocities and accelerations are initialized to zero
   *
   * \param clock Clock used in the ControlProblem
   * \param name Robot name
   * \param module RobotModule from which the Robot instance will be loaded
   */
  Robot(Clock & clock, const std::string & name, std::shared_ptr<mc_rbdyn::RobotModule> module);

  /** \brief Overload constructor to load a robot with a given initial stance
   *
   * \param clock Clock used in the control problem
   * \param name Robot name
   * \param module RobotModule from which the Robot instance will be loaded
   * \param q Initial joint stance provided as a map of (joint name, joint value vector)
   */
  Robot(Clock & clock,
        const std::string & name,
        std::shared_ptr<mc_rbdyn::RobotModule> module,
        const std::map<std::string, std::vector<double>> & q);

  /** Create a robot directly from URDF
   *
   * @param clock Clock used in the control problem
   * @param name Robot name
   * @param urdfPath Path to the robot's URDF model
   * @param fixed True for fixed-base robots
   * @param filteredLinks Ignore these links
   * @param q Initial joint stance provided as a map of (joint name, joint value vector)
   */
  Robot(Clock & clock,
        const std::string & name,
        const std::string & urdfPath,
        bool fixed,
        const std::vector<std::string> & filteredLinks,
        const std::map<std::string, std::vector<double>> & q);

  /** Access the robot's name */
  inline const std::string & name() const
  {
    return name_;
  }

  /** Returns the associated robot module */
  inline const mc_rbdyn::RobotModule & module() const
  {
    return *module_;
  }

  /** Returns the robot's mass */
  inline double mass() const
  {
    return mass_;
  }

  /** Access q variable (const) */
  inline const VariableVector & q() const
  {
    return q_;
  }
  /** Access q variable */
  inline VariableVector & q()
  {
    return q_;
  }

  /** Access free-flyer variable (const) */
  inline const VariablePtr & qFreeFlyer() const
  {
    return q_ff_;
  }
  /** Access free-flyer variable */
  inline VariablePtr & qFreeFlyer()
  {
    return q_ff_;
  }

  /** Access joints variable (const) */
  inline const VariablePtr & qJoints() const
  {
    return q_joints_;
  }
  /** Access joints variable */
  inline VariablePtr & qJoints()
  {
    return q_joints_;
  }

  /** Access tau variable (const) */
  inline const VariablePtr & tau() const
  {
    return tau_;
  }
  /** Access tau variable */
  inline VariablePtr & tau()
  {
    return tau_;
  }

  /** Access the robot's related rbd::MultiBody (const) */
  inline const rbd::MultiBody & mb() const
  {
    return mb_;
  }
  /** Access the robot's related rbd::MultiBody */
  inline rbd::MultiBody & mb()
  {
    return mb_;
  }

  /** Access the robot's related rbd::MultiBodyConfig (const) */
  inline const rbd::MultiBodyConfig & mbc() const
  {
    return mbc_;
  }
  /** Access the robot's related rbd::MultiBodyConfig */
  inline rbd::MultiBodyConfig & mbc()
  {
    return mbc_;
  }

  /** Access the vector of normal acceleration expressed in the body's frame (const) */
  inline const std::vector<sva::MotionVecd> & normalAccB() const
  {
    return normalAccB_;
  }
  /** Access the vector of normal acceleration expressed in the body's frame */
  inline std::vector<sva::MotionVecd> & normalAccB()
  {
    return normalAccB_;
  }

  /** Access the joints' lower position limit (const) */
  inline const Eigen::VectorXd & lQBound() const
  {
    return lQBound_;
  }
  /** Access the joints' lower position limit */
  inline Eigen::VectorXd & lQBound()
  {
    return lQBound_;
  }

  /** Access the joints' upper position limit (const) */
  inline const Eigen::VectorXd & uQBound() const
  {
    return uQBound_;
  }
  /** Access the joints' upper position limit */
  inline Eigen::VectorXd & uQBound()
  {
    return uQBound_;
  }

  /** Access the joints' lower velocity limit (const) */
  inline const Eigen::VectorXd & lVelBound() const
  {
    return lVelBound_;
  }
  /** Access the joints' lower velocity limit */
  inline Eigen::VectorXd & lVelBound()
  {
    return lVelBound_;
  }

  /** Access the joints' upper velocity limit (const) */
  inline const Eigen::VectorXd & uVelBound() const
  {
    return uVelBound_;
  }
  /** Access the joints' upper velocity limit */
  inline Eigen::VectorXd & uVelBound()
  {
    return uVelBound_;
  }

  /** Access the robot's lower torque limit (const) */
  inline const Eigen::VectorXd & lTauBound() const
  {
    return lTauBound_;
  }
  /** Access the robot's lower torque limit */
  inline Eigen::VectorXd & lTauBound()
  {
    return lTauBound_;
  }

  /** Access the robot's upper torque limit (const) */
  inline const Eigen::VectorXd & uTauBound() const
  {
    return uTauBound_;
  }
  /** Access the robot's upper torque limit */
  inline Eigen::VectorXd & uTauBound()
  {
    return uTauBound_;
  }

  /** Access the inertia matrix */
  inline const Eigen::MatrixXd & H() const
  {
    return fd_.H();
  }
  /** Access the non-linear effect vector */
  /** Access the non-linear effect vector (coriolis, gravity, external force).*/
  inline const Eigen::VectorXd & C() const
  {
    return fd_.C();
  }

  /** Access the CoM position */
  inline const Eigen::Vector3d & com() const
  {
    return com_;
  }

  /** Access the transformation that allows to retrieve the original base of a body */
  inline const sva::PTransformd & bodyTransform(const std::string & b) const
  {
    return bodyTransforms_.at(b);
  }

private:
  Clock & clock_;
  uint64_t last_tick_ = 0;
  std::shared_ptr<mc_rbdyn::RobotModule> module_;
  std::string name_;
  double mass_;
  rbd::MultiBody mb_;
  rbd::MultiBodyConfig mbc_;
  std::vector<sva::MotionVecd> normalAccB_;
  Eigen::VectorXd lQBound_;
  Eigen::VectorXd uQBound_;
  Eigen::VectorXd lVelBound_;
  Eigen::VectorXd uVelBound_;
  Eigen::VectorXd lTauBound_;
  Eigen::VectorXd uTauBound_;
  rbd::ForwardDynamics fd_;
  std::map<std::string, sva::PTransformd> bodyTransforms_;
  VariablePtr q_ff_;
  VariablePtr q_joints_;
  VariableVector q_;
  VariableVector dq_;
  VariableVector ddq_;
  VariablePtr tau_;
  Eigen::Vector3d com_;

private:
  void computeNormalAccB();

  /** Update the Robot's variables based on the output of the solver
   *
   * It will:
   * 1. Put dot(q,2) into mbc.alphaD
   * 2. Run rbd::eulerIntegration with dt
   * 3. Output mbc.alpha into dot(q)
   * 4. Output mbc.q into q
   *
   * \param dt Integration timestep
   *
   */
  void updateTimeDependency();
  void updateFK();
  void updateFV();
  void updateFA();
  void updateNormalAcceleration();
  void updateH();
  void updateC();
  void updateCoM();
};

} // namespace mc_tvm
