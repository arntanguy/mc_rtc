#pragma once

#include <mc_control/fsm/Executor.h>

namespace mc_control
{

namespace fsm
{

/** Implements a "meta" state
 *
 * This states plays its own FSM.
 *
 * Configuration entries:
 *
 * - Managed: if true, does not handle transitions
 * - transitions: a transition map, similiar to the FSM controller (required if Managed is false)
 * - StepByStep: same as FSM for the internal FSM (default: false)
 *
 * - configs: can contain additional configuration for the states in the FSM
 *
 */

struct MC_CONTROL_FSM_STATE_DLLAPI MetaState : State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void stop(Controller &) override;

  void teardown(Controller &) override;

  bool read_msg(std::string & msg) override;

protected:
  mc_rtc::Configuration config_;
  Executor executor_;
};

} // namespace fsm

} // namespace mc_control