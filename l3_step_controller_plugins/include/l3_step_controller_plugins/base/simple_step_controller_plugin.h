//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef L3_SIMPLE_STEP_CONTROLLER_PLUGIN_H__
#define L3_SIMPLE_STEP_CONTROLLER_PLUGIN_H__

#include <l3_step_controller_plugins/base/step_controller_plugin.h>

namespace l3_step_controller
{
using namespace l3;
using namespace l3_footstep_planning;
using namespace l3_step_controller;

/**
 * @brief Simple step controller plugin that converts an entire step plan to
 * another message and publishes it afterwards.
 * The template stub requires to specify the message type and only to override
 * the execute(...) method that converts each step and writes directly the result
 * in the msg_ member. If further initialization is neccessary, just override
 * preProcess(...) or process(...) but do not forget to call the implementation
 * provided by SimpleStepControllerPlugin.
 */
template <class MsgType>
class SimpleStepControllerPlugin : public StepControllerPlugin
{
public:
  SimpleStepControllerPlugin() {}

  bool initialize(const vigir_generic_params::ParameterSet& params = vigir_generic_params::ParameterSet()) override
  {
    if (!StepControllerPlugin::initialize(params))
      return false;

    publisher_ = nh_.advertise<MsgType>("step_controller", 1);

    return true;
  }

  msgs::ErrorStatus updateStepPlan(const msgs::StepPlan& step_plan) override
  {
    msgs::ErrorStatus status;

    if (step_plan.plan.steps.empty())
      return status;

    // Allow step plan updates only in READY state
    const StepControllerState& state = getState();
    if (state == READY)
    {
      msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

      status += step_plan_.fromMsg(step_plan);

      if (isOk(status))
      {
        updateQueueFeedback();

        ROS_INFO("[%s] Updated step queue from [%i; %i]. Current queue has steps in range [%i; %i].", getName().c_str(), step_plan.plan.steps.front().idx,
                 step_plan.plan.steps.back().idx, step_plan_.getSteps().firstStepIndex(), step_plan_.getSteps().lastStepIndex());

        return status;
      }
    }

    status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Controller is not READY (current: " + toString(state) + ")!");

    return status;
  }

  msgs::ErrorStatus initWalk() override
  {
    // clear message
    msg_ = MsgType();
    absolut_step_time_ = 0.0;

    // init feedback states
    msgs::ExecuteStepPlanFeedback feedback;
    feedback.header.stamp = ros::Time::now();
    feedback.last_performed_step_idx = -2;
    feedback.currently_executing_step_idx = -1;
    feedback.first_changeable_step_idx = 0;
    setFeedbackState(feedback);

    setState(ACTIVE);

    ROS_INFO("[%s] Start step plan execution.", getName().c_str());
    return msgs::ErrorStatus();
  }

  msgs::ErrorStatus process(const ros::TimerEvent& event) override
  {
    msgs::ErrorStatus status;

    if (getState() != ACTIVE)
      return status;

    // transform step plan into target message
    for (const StepQueue::Entry& e : step_plan_.getSteps())
    {
      Step::ConstPtr step = e.second;

      // calls execute that must be overwritten to fill the msg_ member
      status += this->executeStep(step);

      if (hasError(status))
        return status;
    }

    // publish transformed message
    publisher_.publish(msg_);

    setState(FINISHED);

    return status;
  }

protected:
  MsgType msg_;
  double absolut_step_time_;

private:
  ros::Publisher publisher_;
};
}  // namespace l3_step_controller

#endif
