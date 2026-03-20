#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rcm_pick_and_place/handover_controller.hpp"

using namespace rcm_handover;

void test_timer(std::shared_ptr<HandoverController>& controller, rclcpp::Node::SharedPtr& node, rclcpp::Time timer_start){
  using namespace std::chrono_literals;
  double ms_waited = (node->get_clock()->now() - timer_start).seconds() * 1s / 1ms;
  if(ms_waited > 10000.) {
    controller->cancel_handover();
  }
}

bool test(std::shared_ptr<HandoverController>& controller, rclcpp::Node::SharedPtr& node){
  using namespace std::chrono_literals;
  auto logger = node->get_logger();
  
  rclcpp::Time timer_start = node->now();

  std::function<void()> timer_cb = std::bind(&test_timer, controller, node, timer_start);
  auto timer = node->create_wall_timer(100ms, timer_cb);

  bool success = controller->compliant_handover(0.005);

  timer->cancel();

  return success;
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  std::string node_name = "rcm_handover";
  rclcpp::NodeOptions node_options;

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(node_name, node_options);

  RCLCPP_INFO(node->get_logger(), "RCM Pick and Place interface starting");

  // Executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));
  
  // Create controller
  std::shared_ptr<HandoverController> controller = std::make_shared<HandoverController>(node);

  // Optional test, configurable by parameter
  bool tests_enabled = true;
  node->declare_parameter("tests_enabled", true);
  node->get_parameter("tests_enabled", tests_enabled);

  if(tests_enabled){
    if(!test(controller, node)){
      RCLCPP_INFO(node->get_logger(), "Test function failed, exiting");
      rclcpp::shutdown();
    }
    else{
      RCLCPP_INFO(node->get_logger(), "Test function successful, RCM Handover interface ready");
    }
  }

  // Shutdown ROS
  executor_thread.join();
  rclcpp::shutdown();

  return 0;
}
