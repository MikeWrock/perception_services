#include <perception_services/blackboard_image_to_file.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>


namespace
{
const auto kLogger = rclcpp::get_logger("SaveImageFromBlackboard");
constexpr auto kPortIDInputImage = "input_image";
constexpr auto kPortIDOutputName = "output_file";
}  // namespace


namespace blackboard_image_to_file
{
BlackboardImageToFile::BlackboardImageToFile(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}

BlackboardImageToFile::~BlackboardImageToFile()
{
}


BT::PortsList BlackboardImageToFile::providedPorts()
{
  return { // Inputs to this behavior is the input topic and filename
           BT::InputPort<sensor_msgs::msg::Image>(kPortIDInputImage),
           BT::InputPort<std::string>(kPortIDOutputName),
  };
}

BT::NodeStatus BlackboardImageToFile::tick()
{
  const auto blackboard_image = getInput<sensor_msgs::msg::Image>(kPortIDInputImage);
  const auto filename = getInput<std::string>(kPortIDOutputName);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(blackboard_image,filename); error)
  {
    RCLCPP_ERROR_STREAM(kLogger, "Failed to get image from input data port:\n" << error.value());
    return BT::NodeStatus::FAILURE;
  }
    // Convert the image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(blackboard_image.value(), sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
      RCLCPP_ERROR(kLogger, "cv_bridge exception: %s", e.what());
      return BT::NodeStatus::FAILURE;
  }
  cv::imwrite(filename.value(), cv_ptr->image);

  RCLCPP_INFO(kLogger, "Saved image to file: %s", filename.value().c_str());
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace blackboard_image_to_file
