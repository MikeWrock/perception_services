#include <perception_services/image_to_blackboard.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>


namespace
{
const auto kLogger = rclcpp::get_logger("SetBlackboardFromTopic");
constexpr auto kPortIDInputTopic = "input_topic";
constexpr auto kPortIDOutputName = "output_port";
}  // namespace


namespace image_to_blackboard
{
ImageToBlackboard::ImageToBlackboard(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

ImageToBlackboard::~ImageToBlackboard()
{
  image_subscriber_.reset();
}


BT::PortsList ImageToBlackboard::providedPorts()
{
  return { // Inputs to this behavior is the input topic
           BT::InputPort<std::string>(kPortIDInputTopic),
           // Outputs from this behavior is the name of the port
           BT::OutputPort<sensor_msgs::msg::Image>(kPortIDOutputName),
  };
}

BT::NodeStatus ImageToBlackboard::onStart()
{
  const auto image_topic = getInput<std::string>(kPortIDInputTopic);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_topic); error)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Failed to get required values from input data ports:\n" << error.value());
    return BT::NodeStatus::FAILURE;
  }

  auto options = rclcpp::SubscriptionOptions();
  auto callback_group = shared_resources_->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = callback_group;
  callback_complete_ = false;
  image_subscriber_ = shared_resources_->node->create_subscription<sensor_msgs::msg::Image>(
      image_topic.value(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { subscriberCallback(msg); }, options);

  RCLCPP_ERROR_STREAM(kLogger, "Waiting for image on topic: " << image_topic.value() << " !!");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ImageToBlackboard::onRunning()
{
  if(callback_complete_) image_subscriber_.reset();
  return callback_complete_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void ImageToBlackboard::onHalted()
{
  image_subscriber_.reset();
}

void ImageToBlackboard::subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_ERROR(kLogger, "Received image");
  callback_complete_ = true;
  // Write to blackboard
  setOutput(kPortIDOutputName, *msg);

  // image_subscriber_.reset();
}


}  // namespace image_to_blackboard
