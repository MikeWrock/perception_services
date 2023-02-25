#include <gtest/gtest.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
/**
 * @brief 
 */
TEST(SaveImageToBlackboard, test_perception_services)
{

  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_node");
  // auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // executor->add_callback_group(callback_group, node->get_node_base_interface());
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic", 10);

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = node->get_clock()->now();

  // GIVEN an instance of LoadPointCloudBehavior configured to load the .pcd file from the expected filepath
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("input_topic", "image_topic");
  // config.blackboard->set("output_port", "output");
  // (Note: this sets the port remapping rules so the keys on the blackboard are the same as the keys used by the behavior)
  config.input_ports.insert(std::make_pair("input_topic", "="));
  config.output_ports.insert(std::make_pair("output_port", "="));


  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("perception_services::PerceptionServicesBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto image_to_blackboard_behavior = factory.instantiateTreeNode("test_behavior_name", "ImageToBlackboard", config);

  // WHEN the behavior is ticked for the first time
  // THEN it returns RUNNING because it has started listening
  ASSERT_EQ(image_to_blackboard_behavior->executeTick(), BT::NodeStatus::RUNNING);

  msg->width = 640;
  msg->height = 480;
  msg->encoding = "rgb8";
  msg->is_bigendian = 0;
  msg->step = 640 * 3;
  msg->data.resize(msg->step * msg->height);
  pub->publish(std::move(msg));

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // WHEN the behavior is ticked a second time after waiting a short duration
  // THEN it returns SUCCESS because it has finished loading the cloud
  // ASSERT_EQ(image_to_blackboard_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  ASSERT_EQ(image_to_blackboard_behavior->executeTick(), BT::NodeStatus::SUCCESS);
  // THEN the cloud is written to the output data port, is of the expected size, and has the correct base frame ID
  sensor_msgs::msg::Image image_msg;
  try
  {
    image_msg = config.blackboard->get<sensor_msgs::msg::Image>("output_port");
  }
  catch (const std::exception& e)
  {
    FAIL() << "Failed to retrieve image message from blackboard: " << e.what();
  }

  EXPECT_EQ(image_msg.width, static_cast<unsigned int>(640));
  EXPECT_EQ(image_msg.height, static_cast<unsigned int>(480));

}


TEST(SaveBlackboardImageToFile, test_perception_services)
{

  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic", 10);
  const auto filename = "/opt/moveit_studio/user_ws/blackboard_file.jpg";

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = node->get_clock()->now();
  msg->width = 160;
  msg->height = 120;
  msg->encoding = "rgb8";
  msg->is_bigendian = 0;
  msg->step = 160 * 3;
  msg->data.resize(msg->step * msg->height);

  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.input_ports.insert(std::make_pair("input_image", "="));
  config.input_ports.insert(std::make_pair("output_file", "="));

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("perception_services::PerceptionServicesBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto blackboard_image_to_file_behavior = factory.instantiateTreeNode("test_behavior_name", "BlackboardImageToFile", config);


  ASSERT_EQ(blackboard_image_to_file_behavior->executeTick(), BT::NodeStatus::FAILURE);

  config.blackboard->set("input_image", *msg);

  ASSERT_EQ(blackboard_image_to_file_behavior->executeTick(), BT::NodeStatus::FAILURE);

  config.blackboard->set("output_file", filename);

  // WHEN the behavior is ticked for the first time
  // THEN it returns RUNNING because it has started listening
  ASSERT_EQ(blackboard_image_to_file_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  // Check that the file exists
  EXPECT_TRUE(std::filesystem::exists(filename));
  // Read the saved image file
  cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
 
 
  // Check that the image dimensions match expected values
  EXPECT_EQ(image.cols, 160);
  EXPECT_EQ(image.rows, 120);

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
