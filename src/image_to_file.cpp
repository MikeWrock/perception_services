#include <perception_services/image_to_file.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>


namespace
{
const auto kLogger = rclcpp::get_logger("SaveImageFromTopic");
constexpr auto kPortIDInputTopic = "input_topic";
constexpr auto kPortIDOutputName = "output_file";
}  // namespace


namespace image_to_file
{
ImageToFile::ImageToFile(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::StatefulActionNode>(name, config, shared_resources)
{
}

ImageToFile::~ImageToFile()
{
  image_subscriber_.reset();
}


BT::PortsList ImageToFile::providedPorts()
{
  return { // Inputs to this behavior is the input topic and filename
           BT::InputPort<std::string>(kPortIDInputTopic),
           BT::InputPort<std::string>(kPortIDOutputName),
  };
}

BT::NodeStatus ImageToFile::onStart()
{
  const auto image_topic = getInput<std::string>(kPortIDInputTopic);
  const auto filename = getInput<std::string>(kPortIDOutputName);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_topic,filename); error)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Failed to get required values from input data ports:\n" << error.value());
    return BT::NodeStatus::FAILURE;
  }

  auto options = rclcpp::SubscriptionOptions();
  auto callback_group = shared_resources_->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_callback_group(callback_group, shared_resources_->node->get_node_base_interface());
  options.callback_group = callback_group;
  callback_complete_ = false;
  image_subscriber_ = shared_resources_->node->create_subscription<sensor_msgs::msg::Image>(
      image_topic.value(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { subscriberCallback(msg); }, options);

  RCLCPP_ERROR_STREAM(kLogger, "Waiting for image on topic: " << image_topic.value() << " !!");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ImageToFile::onRunning()
{
  if(callback_complete_) image_subscriber_.reset();
  return callback_complete_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void ImageToFile::onHalted()
{
  image_subscriber_.reset();
}

void ImageToFile::subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_ERROR(kLogger, "Received image");
  callback_complete_ = true;
  // Write to file    
  const auto filename = getInput<std::string>(kPortIDOutputName);

  // Convert the image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(kLogger, "cv_bridge exception: %s", e.what());
      return;
  }
  cv::imwrite(filename.value(), cv_ptr->image);

  RCLCPP_ERROR(kLogger, "Saved image to file: %s", filename.value().c_str());
  
}


}  // namespace image_to_file
