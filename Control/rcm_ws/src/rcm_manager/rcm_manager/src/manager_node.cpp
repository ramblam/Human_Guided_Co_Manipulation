#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rcm_manager/manager_base.hpp>

bool test(std::shared_ptr<rcm_manager::ManagerBase> manager){
  (void)manager;
  return false;
}

int main(int argc, char** argv){
  // Initialize ROS and create a Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("rcm_manager", node_options);

  // Executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  // Configurable manager plugin (should match the type)
  std::string plugin = "";
  node->declare_parameter("manager_plugin", "rcm_manager_cic_plugins/CartesianImpedanceControllerManager");
  node->get_parameter("manager_plugin", plugin);

  auto logger = node->get_logger();

  pluginlib::ClassLoader<rcm_manager::ManagerBase> manager_loader("rcm_manager", "rcm_manager::ManagerBase");
  std::shared_ptr<rcm_manager::ManagerBase> manager;
  
  try{
    manager = manager_loader.createSharedInstance(plugin);
    RCLCPP_INFO(logger, "Successfully loaded plugin %s", plugin.c_str());
  }
  
  catch(pluginlib::PluginlibException& ex){
    RCLCPP_ERROR(node->get_logger(), "Plugin %s failed to load. Error: %s\n", plugin.c_str(), ex.what());
    rclcpp::shutdown();
    executor_thread.join();

    return 1;
  }

  RCLCPP_INFO(logger, "Initializing plugin");
  manager->init(node);

  // Wait while executor is running
  executor_thread.join();
  rclcpp::shutdown();

  return 0;
}