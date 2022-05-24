#include <l3_step_controller/step_controller.h>

namespace l3_step_controller
{
StepController::StepController(ros::NodeHandle& nh)
{
  vigir_pluginlib::PluginManager::addPluginClassLoader<StepControllerPlugin>("l3_step_controller_plugins", "l3_step_controller::StepControllerPlugin");

  // subscribe topics
  load_step_controller_plugin_sub_ = nh.subscribe("load_step_controller_plugin", 1, &StepController::loadStepControllerPlugin, this);
  execute_step_plan_sub_ = nh.subscribe("execute_step_plan", 1, &StepController::executeStepPlan, this);

  // publish topics
  feedback_pub_ = nh.advertise<msgs::ExecuteStepPlanFeedback>("execute_feedback", 1, true);

  // init action servers
  execute_step_plan_as_.reset(new ExecuteStepPlanActionServer(nh, "execute_step_plan", false));
  execute_step_plan_as_->registerGoalCallback(boost::bind(&StepController::executeStepPlanAction, this, boost::ref(execute_step_plan_as_)));
  execute_step_plan_as_->registerPreemptCallback(boost::bind(&StepController::executePreemptionAction, this, boost::ref(execute_step_plan_as_)));

  // start action servers
  execute_step_plan_as_->start();
}

StepController::~StepController() {}

bool StepController::initialize(ros::NodeHandle& nh, bool auto_spin)
{
  // init step controller plugin
  if (!loadPlugin(nh.param("step_controller_plugin", std::string("step_controller_plugin")), step_controller_plugin_))
    return false;

  return initialize(nh, step_controller_plugin_, auto_spin);
}

bool StepController::initialize(ros::NodeHandle& nh, StepControllerPlugin::Ptr step_controller_plugin, bool auto_spin)
{
  msgs::ErrorStatus status;

  step_controller_plugin_ = step_controller_plugin;

  // schedule main update loop
  if (auto_spin)
    update_timer_ = nh.createTimer(ros::Rate(nh.param("update_rate", 10.0)), &StepController::update, this);

  return !hasError(status);
}

msgs::ErrorStatus StepController::executeStepPlan(const msgs::StepPlan& step_plan)
{
  UniqueLock lock(controller_mutex_);

  if (!step_controller_plugin_)
    return ErrorStatusError(msgs::ErrorStatus::ERR_NO_PLUGIN_AVAILABLE, "StepController", "executeStepPlan: No step_controller_plugin available!");

  // An empty step plan will always trigger a soft stop
  if (step_plan.plan.steps.empty())
    step_controller_plugin_->stop();
  else
    return step_controller_plugin_->updateStepPlan(step_plan);

  return msgs::ErrorStatus();
}

void StepController::update(const ros::TimerEvent& event)
{
  UniqueLock lock(controller_mutex_);

  msgs::ErrorStatus status;

  if (!step_controller_plugin_)
  {
    ROS_ERROR_THROTTLE(5.0, "[StepController] update: No step_controller_plugin available!");
    return;
  }

  // Save (copy!) current state to be able to handle action server correctly;
  // We must not send setSucceeded/setAborted state while sending the
  // final feedback message in the same update cycle!
  const StepControllerState state = step_controller_plugin_->getState();

  if (state == NOT_READY || state == FINISHED || state == FAILED)
    status += step_controller_plugin_->init();
  else
  {
    // pre process
    if (isOk(status))
      status += step_controller_plugin_->preProcess(event);

    // process
    if (isOk(status))
      status += step_controller_plugin_->process(event);

    // post process
    if (isOk(status))
      status += step_controller_plugin_->postProcess(event);
  }

  lock.unlock();

  // publish feedback
  if (state == ACTIVE)
    publishFeedback(status);

  // update action server
  switch (state)
  {
    case FINISHED:
      if (execute_step_plan_as_->isActive())
      {
        msgs::ExecuteStepPlanResult result;
        result.controller_state = state;
        result.error_status = status;
        execute_step_plan_as_->setSucceeded(result);
      }
      break;

    case FAILED:
      if (execute_step_plan_as_->isActive())
      {
        msgs::ExecuteStepPlanResult result;
        result.controller_state = state;
        result.error_status = status;
        execute_step_plan_as_->setAborted(result);
      }
      break;

    default:
      break;
  }
}

void StepController::publishFeedback(const msgs::ErrorStatus& status) const
{
  msgs::ExecuteStepPlanFeedback feedback = step_controller_plugin_->getFeedbackState();
  feedback.error_status += status;

  // publish feedback
  feedback_pub_.publish(feedback);

  if (execute_step_plan_as_->isActive())
    execute_step_plan_as_->publishFeedback(feedback);

  step_controller_plugin_->clearErrorStatusFeedback();
}

// --- Subscriber calls ---

void StepController::loadStepControllerPlugin(const std_msgs::StringConstPtr& plugin_name) { loadPlugin(plugin_name->data, step_controller_plugin_); }

void StepController::executeStepPlan(const msgs::StepPlanConstPtr& step_plan) { executeStepPlan(*step_plan); }

//--- action server calls ---

void StepController::executeStepPlanAction(ExecuteStepPlanActionServerPtr as)
{
  const msgs::ExecuteStepPlanGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::ErrorStatus status = executeStepPlan(goal->step_plan);

  if (hasError(status))
  {
    msgs::ExecuteStepPlanResult result;
    if (step_controller_plugin_)
      result.controller_state = step_controller_plugin_->getState();
    result.error_status = status;
    execute_step_plan_as_->setAborted(result);
  }
}

void StepController::executePreemptionAction(ExecuteStepPlanActionServerPtr as)
{
  if (as->isActive())
    as->setPreempted();

  // step_controller_plugin->stop();
}
}  // namespace l3_step_controller
