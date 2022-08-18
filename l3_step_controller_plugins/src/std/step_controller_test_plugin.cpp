#include <l3_step_controller_plugins/std/step_controller_test_plugin.h>

namespace l3_step_controller
{
StepControllerTestPlugin::StepControllerTestPlugin()
  : StepControllerPlugin()
{}

StepControllerTestPlugin::~StepControllerTestPlugin() {}

msgs::ErrorStatus StepControllerTestPlugin::initWalk()
{
  // init feedback states
  msgs::ExecuteStepPlanFeedback feedback;
  feedback.header.stamp = ros::Time::now();
  feedback.last_performed_step_idx = -2;
  feedback.currently_executing_step_idx = -1;
  feedback.first_changeable_step_idx = 0;
  setFeedbackState(feedback);

  next_step_needed_time_ = ros::Time::now();

  setState(ACTIVE);

  ROS_INFO("[StepControllerTestPlugin] Start fake execution.");

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepControllerTestPlugin::preProcess(const ros::TimerEvent& event)
{
  msgs::ErrorStatus status = StepControllerPlugin::preProcess(event);

  if (hasError(status) || getState() != ACTIVE)
    return status;

  // fake succesful execution of single step
  if (next_step_needed_time_ <= ros::Time::now())
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

    feedback.header.stamp = ros::Time::now();
    feedback.last_performed_step_idx++;

    // check for successful execution of queue
    if (step_plan_.getSteps().lastStepIndex() == feedback.last_performed_step_idx)
    {
      ROS_INFO("[StepControllerTestPlugin] Fake execution finished.");

      feedback.currently_executing_step_idx = -1;
      feedback.first_changeable_step_idx = -1;
      setFeedbackState(feedback);

      step_plan_.clear();
      updateQueueFeedback();

      setState(FINISHED);
    }
    // otherwise trigger fake execution of next step
    else
    {
      feedback.currently_executing_step_idx++;
      feedback.first_changeable_step_idx++;
      setFeedbackState(feedback);

      setNextStepIndexNeeded(feedback.currently_executing_step_idx);
    }
  }

  return status;
}

msgs::ErrorStatus StepControllerTestPlugin::executeStep(Step::ConstPtr step)
{
  double max_duration = 0;
  for (const Step::FootStep::MovingDataPair& p : step->footStep().getMovingLinks())
  {
    if (p.second->step_duration > max_duration)
      max_duration = p.second->step_duration;
  }
  next_step_needed_time_ = ros::Time::now() + ros::Duration(max_duration);
  ROS_INFO("[StepControllerTestPlugin] Fake execution of step %i", step->getStepIndex());

  return msgs::ErrorStatus();
}
}  // namespace l3_step_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_step_controller::StepControllerTestPlugin, l3_step_controller::StepControllerPlugin)
