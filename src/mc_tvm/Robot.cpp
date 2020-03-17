#include <mc_tvm/Robot.h>
#include <RBDyn/CoM.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <tvm/Space.h>
#include <tvm/exception/exceptions.h>

namespace mc_tvm
{

Robot::Robot(const std::string & name,
             const std::shared_ptr<Clock> & clock,
             const std::shared_ptr<mc_rbdyn::RobotModule> & module)
: Robot(name, clock, module, module->stance())
{
}

Robot::Robot(const std::string & name,
             const std::shared_ptr<Clock> & clock,
             const std::string & urdfPath,
             bool fixed,
             const std::vector<std::string> & filteredLinks,
             const std::map<std::string, std::vector<double>> & q)
: Robot(name,
        clock,
        [&urdfPath, &name, fixed, &filteredLinks]() {
          std::ifstream ifs(urdfPath);
          if(!ifs.good())
          {
            LOG_ERROR_AND_THROW(Exception, "Failed to open " + urdfPath + " for robot " + name);
          }
          std::stringstream ss;
          ss << ifs.rdbuf();
          auto res = mc_rbdyn_urdf::rbdyn_from_urdf(ss.str(), fixed, filteredLinks);
          auto module = std::make_shared<mc_rbdyn::RobotModule>(name, std::move(res));
          return module;
        }(),
        q)
{
}

Robot::Robot(const std::string & name,
             const std::shared_ptr<Clock> & clock,
             const std::shared_ptr<mc_rbdyn::RobotModule> & module,
             const std::map<std::string, std::vector<double>> & q)
: clock_(clock), last_tick_(clock->ticks()), module_(module), name_(name), mb_(module->mb), mbc_(module->mbc),
  normalAccB_(mbc_.bodyAccB.size()), fd_(mb_), bodyTransforms_(module->mbg.bodiesBaseTransform(mb_.body(0).name())),
  tau_(tvm::Space(mb_.nrDof()).createVariable("tau"))
{
  auto & mb = module->mb;
  auto & limits = module->limits;

  if(mb.nrJoints() > 0 && mb.joint(0).type() == rbd::Joint::Free)
  {
    q_ff_ = tvm::Space(6, 7, 6).createVariable(name_ + "_qFreeFlyer");
    q_joints_ = tvm::Space(mb.nrDof() - 6, mb.nrParams() - 7, mb.nrDof() - 6).createVariable(name_ + "_qJoints");
  }
  else
  {
    q_ff_ = tvm::Space(0).createVariable(name_ + "_qFreeFlyer");
    q_joints_ = tvm::Space(mb.nrDof(), mb.nrParams(), mb.nrDof()).createVariable(name_ + "_qJoints");
  }
  /** Bounds generic initialization */
  lQBound_.resize(q_joints_->size());
  lQBound_.setConstant(-tvm::constant::big_number);
  uQBound_ = -lQBound_;
  lVelBound_.resize(q_joints_->size());
  lVelBound_.setConstant(-tvm::constant::big_number);
  uVelBound_ = -lVelBound_;
  lTauBound_.resize(q_joints_->size() + q_ff_->space().tSize());
  lTauBound_.head(q_ff_->space().tSize()).setZero();
  lTauBound_.tail(q_joints_->size()).setConstant(-tvm::constant::big_number);
  uTauBound_ = -lTauBound_;
  /** Bounds initialization based on provided limits */
  {
    const auto & jIndexByName = mb_.jointIndexByName();
    auto map2bound = [this, &jIndexByName](const std::map<std::string, std::vector<double>> & bound, double mul,
                                           Eigen::VectorXd & out, int ffOffset,
                                           int (rbd::MultiBody::*posMethod)(int) const) {
      for(const auto & [jName, qi] : bound)
      {
        if(!jIndexByName.count(jName))
        {
          continue;
        }
        auto jIndex = jIndexByName.at(jName);
        auto pos = (mb_.*posMethod)(jIndex)-ffOffset;
        for(size_t i = 0; i < qi.size(); ++i)
        {
          out(pos + i) = mul * qi[i];
        }
      }
    };
    map2bound(limits.lower, 1, lQBound_, q_ff_->size(), &rbd::MultiBody::jointPosInParam);
    map2bound(limits.upper, 1, uQBound_, q_ff_->size(), &rbd::MultiBody::jointPosInParam);
    map2bound(limits.velocity, -1, lVelBound_, q_ff_->space().tSize(), &rbd::MultiBody::jointPosInDof);
    map2bound(limits.velocity, 1, uVelBound_, q_ff_->space().tSize(), &rbd::MultiBody::jointPosInDof);
    map2bound(limits.torque, -1, lTauBound_, 0, &rbd::MultiBody::jointPosInDof);
    map2bound(limits.torque, 1, uTauBound_, 0, &rbd::MultiBody::jointPosInDof);
  }
  /** Variables creation */
  q_.add(q_ff_);
  q_.add(q_joints_);
  dq_ = dot(q_, 1);
  ddq_ = dot(q_, 2);
  // Initialize joint values from the provided stance
  auto q_init = q_.value();
  const auto & jIndexByName = mb_.jointIndexByName();
  for(const auto & [jName, qi] : q)
  {
    if(!jIndexByName.count(jName))
    {
      continue;
    }
    auto jIndex = jIndexByName.at(jName);
    auto jParam = mb.jointPosInParam(jIndex);
    for(size_t i = 0; i < qi.size(); ++i)
    {
      q_init(jParam + static_cast<Eigen::Index>(i)) = qi[i];
    }
  }
  q_.value(q_init);
  // Initialize joint derivatives and torques to zero
  dq_.value(Eigen::VectorXd::Zero(dq_.value().size()));
  ddq_.value(Eigen::VectorXd::Zero(ddq_.value().size()));
  tau_->value(Eigen::VectorXd::Zero(tau_->value().size()));
  /** Signals */
  registerUpdates(Update::Time, &Robot::updateTimeDependency, Update::FK, &Robot::updateFK, Update::FV,
                  &Robot::updateFV, Update::FA, &Robot::updateFA, Update::NormalAcceleration,
                  &Robot::updateNormalAcceleration, Update::CoM, &Robot::updateCoM, Update::H, &Robot::updateH,
                  Update::C, &Robot::updateC);
  /** Input dependencies */
  addInputDependency(Update::Time, clock_, Clock::Output::Time);
  /** Output dependencies */
  addOutputDependency(Output::FK, Update::FK);
  addOutputDependency(Output::FV, Update::FV);
  addOutputDependency(Output::FA, Update::FA);
  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency(Output::CoM, Update::CoM);
  addOutputDependency(Output::H, Update::H);
  addOutputDependency(Output::C, Update::C);
  addOutputDependency(Output::Geometry, Update::CoM);
  addOutputDependency(Output::Dynamics, Update::H);
  addOutputDependency(Output::Dynamics, Update::C);
  addOutputDependency(Output::Dynamics, Update::NormalAcceleration);
  addOutputDependency(Output::Dynamics, Update::FA);
  /** Internal dependencies */
  addInternalDependency(Update::FK, Update::Time);
  addInternalDependency(Update::CoM, Update::FK);
  addInternalDependency(Update::FV, Update::FK);
  addInternalDependency(Update::H, Update::FV);
  addInternalDependency(Update::C, Update::FV);
  addInternalDependency(Update::FA, Update::FV);
  addInternalDependency(Update::NormalAcceleration, Update::FV);

  // Compute mass
  mass_ = 0;
  for(const auto & b : mb_.bodies())
  {
    mass_ += b.inertia().mass();
  }

  // Make sure initial robot quantities are well initialized
  updateFK();
  updateFV();
  updateFA();
  updateNormalAcceleration();
  if(mass_ > 0)
  {
    updateCoM();
  }
}

void Robot::updateTimeDependency()
{
  if(last_tick_ != clock_->ticks())
  {
    auto ddq = ddq_.value();
    rbd::vectorToParam(ddq, mbc_.alphaD);
    rbd::eulerIntegration(mb_, mbc_, clock_->dt());
    auto dq = dq_.value();
    rbd::paramToVector(mbc_.alpha, dq);
    dq_.value(dq);
    auto q = q_.value();
    rbd::paramToVector(mbc_.q, q);
    q_.value(q);
    last_tick_ = clock_->ticks();
  }
}

void Robot::updateFK()
{
  rbd::forwardKinematics(mb_, mbc_);
}

void Robot::updateFV()
{
  rbd::forwardVelocity(mb_, mbc_);
}

void Robot::updateFA()
{
  rbd::forwardAcceleration(mb_, mbc_);
}

void Robot::updateNormalAcceleration()
{
  computeNormalAccB();
}

void Robot::computeNormalAccB()
{
  // No need to compute that if the robot is not actuated
  if(mb_.nrDof() > 0)
  {
    const auto & pred = mb_.predecessors();
    const auto & succ = mb_.successors();
    for(int i = 0; i < mb_.nrJoints(); ++i)
    {
      const auto & X_p_i = mbc_.parentToSon[i];
      const auto & vj_i = mbc_.jointVelocity[i];
      const auto & vb_i = mbc_.bodyVelB[i];
      if(pred[i] != -1)
      {
        normalAccB_[succ[i]] = X_p_i * normalAccB_[pred[i]] + vb_i.cross(vj_i);
      }
      else
      {
        normalAccB_[succ[i]] = vb_i.cross(vj_i);
      }
    }
  }
}

void Robot::updateH()
{
  fd_.computeH(mb_, mbc_);
}

void Robot::updateC()
{
  fd_.computeC(mb_, mbc_);
}

void Robot::updateCoM()
{
  com_ = rbd::computeCoM(mb_, mbc_);
}

Robot::Exception::Exception(const std::string & msg) : msg(msg) {}

Robot::Exception::~Exception() noexcept
{
  if(msg.size())
  {
    LOG_ERROR(msg)
  }
}

const char * Robot::Exception::what() const noexcept
{
  return msg.c_str();
}

void Robot::Exception::silence() noexcept
{
  msg.resize(0);
}

} // namespace mc_tvm
