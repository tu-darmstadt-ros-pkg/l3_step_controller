#include <l3_step_controller_plugins/base/step_controller_plugin.h>

namespace l3_step_controller
{
std::string toString(const StepControllerState& state)
{
  switch (state)
  {
    case NOT_READY:
      return "NOT_READY";
    case READY:
      return "READY";
    case ACTIVE:
      return "ACTIVE";
    case PAUSED:
      return "PAUSED";
    case FINISHED:
      return "FINISHED";
    case FAILED:
      return "FAILED";
    default:
      return "UNKNOWN";
  }
}

StepControllerPlugin::StepControllerPlugin()
  : vigir_pluginlib::Plugin("step_controller")
  , state_(NOT_READY)
{
  reset();
}

StepControllerPlugin::~StepControllerPlugin() {}

msgs::ErrorStatus StepControllerPlugin::init()
{
  msgs::ErrorStatus status = reset();
  setState(READY);
  return status;
}

const StepControllerState& StepControllerPlugin::getState() const
{
  SharedLock lock(plugin_mutex_);
  return state_;
}

const StepIndex& StepControllerPlugin::getNextStepIndexNeeded() const
{
  SharedLock lock(plugin_mutex_);
  return next_step_index_needed_;
}

const StepIndex& StepControllerPlugin::getLastStepIndexSent() const
{
  SharedLock lock(plugin_mutex_);
  return last_step_index_sent_;
}

const msgs::ExecuteStepPlanFeedback& StepControllerPlugin::getFeedbackState() const
{
  SharedLock lock(plugin_mutex_);
  return feedback_state_;
}

msgs::ErrorStatus StepControllerPlugin::reset()
{
  step_plan_.clear();

  msgs::ExecuteStepPlanFeedback feedback;
  feedback.last_performed_step_idx = -1;
  feedback.currently_executing_step_idx = -1;
  feedback.first_changeable_step_idx = -1;
  setFeedbackState(feedback);

  setNextStepIndexNeeded(-1);
  setLastStepIndexSent(-1);

  return msgs::ErrorStatus();
}

void StepControllerPlugin::setState(StepControllerState state)
{
  UniqueLock lock(plugin_mutex_);
  ROS_INFO("[%s] Switching state from '%s' to '%s'.", getName().c_str(), toString(this->state_).c_str(), toString(state).c_str());
  this->state_ = state;
  feedback_state_.controller_state = state;
}

void StepControllerPlugin::setNextStepIndexNeeded(const StepIndex& index)
{
  UniqueLock lock(plugin_mutex_);
  next_step_index_needed_ = index;
}

void StepControllerPlugin::setLastStepIndexSent(const StepIndex& index)
{
  UniqueLock lock(plugin_mutex_);
  last_step_index_sent_ = index;
}

void StepControllerPlugin::setFeedbackState(const msgs::ExecuteStepPlanFeedback& feedback)
{
  UniqueLock lock(plugin_mutex_);
  this->feedback_state_ = feedback;
}

void StepControllerPlugin::clearErrorStatusFeedback()
{
  UniqueLock lock(plugin_mutex_);
  feedback_state_.error_status = msgs::ErrorStatus();
}

void StepControllerPlugin::updateQueueFeedback()
{
  UniqueLock lock(plugin_mutex_);
  feedback_state_.queue_size = static_cast<int>(step_plan_.getSteps().size());
  feedback_state_.first_queued_step_idx = step_plan_.getSteps().firstStepIndex();
  feedback_state_.last_queued_step_idx = step_plan_.getSteps().lastStepIndex();
}

msgs::ErrorStatus StepControllerPlugin::updateStepPlan(const msgs::StepPlan& step_plan)
{
  msgs::ErrorStatus status;

  if (step_plan.plan.steps.empty())
    return status;

  // Allow step plan updates only in READY and ACTIVE state
  const StepControllerState& state = getState();
  if (state == READY || state == ACTIVE)
  {
    msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

    status += step_plan_.stitchStepPlan(step_plan);

    if (isOk(status))
    {
      // resets last_step_index_sent counter to trigger (re)executing steps in process()
      if (state == ACTIVE)
        setLastStepIndexSent(feedback.first_changeable_step_idx - 1);

      updateQueueFeedback();

      ROS_INFO("[%s] Updated step queue from [%i; %i]. Current queue has steps in range [%i; %i].", getName().c_str(), step_plan.plan.steps.front().idx,
               step_plan.plan.steps.back().idx, step_plan_.getSteps().firstStepIndex(), step_plan_.getSteps().lastStepIndex());

      return status;
    }
  }

  status += ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, getName(), "Controller is neither READY nor ACTIVE (current: " + toString(state) + ")!");

  return status;
}

msgs::ErrorStatus StepControllerPlugin::preProcess(const ros::TimerEvent& /*event*/)
{
  msgs::ErrorStatus status;

  // check if new walking request has been done
  if (getState() == READY && !step_plan_.getSteps().empty())
  {
    // check consisty
    if (step_plan_.getSteps().firstStepIndex() != 0)
    {
      setState(FAILED);
      return ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, getName(), "Step plan doesn't start with initial step (step_index = 0). Execution aborted!");
    }
    else
    {
      status += initWalk();
    }
  }

  return status;
}

msgs::ErrorStatus StepControllerPlugin::process(const ros::TimerEvent& /*event*/)
{
  msgs::ErrorStatus status;

  // execute steps
  if (getState() == ACTIVE)
  {
    // spool all requested steps
    while (getLastStepIndexSent() < getNextStepIndexNeeded())
    {
      // check if queue isn't empty
      if (step_plan_.getSteps().empty())
      {
        setState(FAILED);
        ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, getName(),
                         "Step %i " + boost::lexical_cast<std::string>(getNextStepIndexNeeded()) + " required but queue is empty. Execution aborted!");
        return status;
      }

      // determine next step index
      StepIndex next_step_idx = getLastStepIndexSent() + 1;
      Step::Ptr step;

      // retrieve next step
      if (!step_plan_.getSteps().getStep(next_step_idx, step))
      {
        break;
//        setState(FAILED);
//        ErrorStatusError(msgs::ErrorStatus::ERR_INCONSISTENT_STEP_PLAN, getName(),
//                         "Missing step " + boost::lexical_cast<std::string>(next_step_idx) + " in queue. Execution aborted!");
//        return status;
      }

      ROS_ASSERT(next_step_idx == step->getStepIndex());

      status += executeStep(step);

      // send step to walking engine
      if (hasError(status))
      {
        setState(FAILED);
        return status;
      }

      // increment last_step_index_sent
      setLastStepIndexSent(next_step_idx);
    }
  }

  return status;
}

msgs::ErrorStatus StepControllerPlugin::postProcess(const ros::TimerEvent& /*event*/)
{
  if (getState() == READY)
    return msgs::ErrorStatus();

  msgs::ExecuteStepPlanFeedback feedback = getFeedbackState();

  // garbage collection: remove already executed steps
  if (step_plan_.getSteps().firstStepIndex() <= feedback.last_performed_step_idx)
    step_plan_.getSteps().removeSteps(0, feedback.last_performed_step_idx);

  // update feedback
  updateQueueFeedback();

  return msgs::ErrorStatus();
}

msgs::ErrorStatus StepControllerPlugin::stop()
{
  UniqueLock lock(plugin_mutex_);

  ROS_INFO("[%s] Stop requested. Resetting walk controller.", getName().c_str());
  return reset();
}
}  // namespace l3_step_controller
