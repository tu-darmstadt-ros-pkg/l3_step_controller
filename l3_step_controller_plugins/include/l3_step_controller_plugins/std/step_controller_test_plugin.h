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

#ifndef L3_STEP_CONTROLLER_TEST_PLUGIN_H__
#define L3_STEP_CONTROLLER_TEST_PLUGIN_H__

#include <ros/ros.h>

#include <l3_step_controller_plugins/base/step_controller_plugin.h>

namespace l3_step_controller
{
class StepControllerTestPlugin : public StepControllerPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<StepControllerTestPlugin> Ptr;
  typedef l3::SharedPtr<const StepControllerTestPlugin> ConstPtr;

  StepControllerTestPlugin();
  virtual ~StepControllerTestPlugin();

  /**
   * @brief We have to initialize all variables at beginning of new step plan
   * execution here.
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus initWalk() override;

  /**
   * @brief Simulates handling of walking engine and triggers in regular interval
   * a succesful execution of a step.
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus preProcess(const ros::TimerEvent& event) override;

  /**
   * @brief Handles fake execution of step. In fact it does nothing as accepting
   * the step.
   * @param Step to be executed
   * @return Status message (empty when no issues occured)
   */
  msgs::ErrorStatus executeStep(Step::ConstPtr step) override;

protected:
  ros::Time next_step_needed_time_;
};
}  // namespace l3_step_controller

#endif
