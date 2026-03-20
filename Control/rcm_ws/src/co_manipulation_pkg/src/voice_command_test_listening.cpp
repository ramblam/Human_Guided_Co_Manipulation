#include <thread>
#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "co_manipulation_pkg/voice_command_action_client.hpp"
#include "co_manipulation_pkg/own_controller.hpp"
#include "co_manipulation_pkg/voice_command_action.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv){
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("voice_tests", node_options);
  std::shared_ptr<rclcpp::Node> voice_command_server_node = std::make_shared<rclcpp::Node>("voice_command_server", node_options);

  // Create controller
  std::shared_ptr<OwnController> controller = std::make_shared<OwnController>(node);

  // Executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(voice_command_server_node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  auto logger = node->get_logger();
  VoiceCommandAction voice_command_action(voice_command_server_node, controller);

  RCLCPP_INFO(logger, "Voice command action created, started listening...");
  executor_thread.join();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}