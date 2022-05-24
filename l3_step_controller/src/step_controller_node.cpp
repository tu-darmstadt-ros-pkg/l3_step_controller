#include <l3_step_controller/step_controller_node.h>

#include <vigir_generic_params/parameter_manager.h>
#include <vigir_pluginlib/plugin_manager.h>

namespace l3_step_controller
{
StepControllerNode::StepControllerNode(ros::NodeHandle& nh)
{
  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  step_controller_.reset(new StepController(nh));
  step_controller_->initialize(nh, nh.param("auto_spin", true));
}

StepControllerNode::~StepControllerNode() {}
}  // namespace l3_step_controller

int main(int argc, char** argv)
{
  ros::init(argc, argv, "l3_step_controller");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  l3_step_controller::StepControllerNode step_controller_node(nh);

  ros::spin();

  return 0;
}
