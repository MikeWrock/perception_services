#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <perception_services/image_to_blackboard.hpp>
#include <perception_services/blackboard_image_to_file.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace perception_services
{
class PerceptionServicesBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    // moveit_studio::behaviors::registerBehavior<PerceptionServices>(factory, "PerceptionServices", shared_resources);
    moveit_studio::behaviors::registerBehavior<image_to_blackboard::WrockToBlackboard>(factory, "MikeWrockToBlackboard", shared_resources);
    moveit_studio::behaviors::registerBehavior<blackboard_image_to_file::BlackboardImageToFile>(factory, "BlackboardImageToFile", shared_resources);
    
  }
};
}  // namespace perception_services

PLUGINLIB_EXPORT_CLASS(perception_services::PerceptionServicesBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
