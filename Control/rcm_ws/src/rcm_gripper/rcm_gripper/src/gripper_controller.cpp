#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <rcm_gripper/gripper_base.hpp>
#include <rcm_gripper/generic_end_effector_base.hpp>

bool test(std::shared_ptr<rcm_gripper::BasicGripper> gripper){
  (void)gripper;
  return false;
}

int main(int argc, char** argv){
  // Initialize ROS and create a Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("rcm_gripper", node_options);

  // Executor to spin the node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor));

  // Configurable plugin type
  std::string plugin_type = "";
  node->declare_parameter("gripper_plugin_type", "rcm_gripper::GenericEndEffector");
  node->get_parameter("gripper_plugin_type", plugin_type);

  // Configurable gripper plugin (should match the type)
  std::string plugin = "";
  node->declare_parameter("gripper_plugin", "rcm_gripper_dummy_plugins::DummyEndEffector");
  node->get_parameter("gripper_plugin", plugin);

  // Configurable services
  bool start_services = true;
  node->declare_parameter("start_services", true);
  node->get_parameter("start_services", start_services);

  // Configurable action for rcm_gripper::GenericEndEffector type plugins
  bool start_action = true;
  node->declare_parameter("start_action_server", true);
  node->get_parameter("start_action_server", start_action);


  auto logger = node->get_logger();

  
  pluginlib::ClassLoader<rcm_gripper::BasicGripper> gripper_loader("rcm_gripper", "rcm_gripper::BasicGripper");
  std::shared_ptr<rcm_gripper::BasicGripper> gripper;

  pluginlib::ClassLoader<rcm_gripper::GenericEndEffector> end_effector_loader("rcm_gripper", "rcm_gripper::GenericEndEffector");
  std::shared_ptr<rcm_gripper::GenericEndEffector> end_effector;

  try{
    if(plugin_type == "rcm_gripper::BasicGripper"){
      gripper = gripper_loader.createSharedInstance(plugin);
    }
    else if(plugin_type == "rcm_gripper::GenericEndEffector"){
      end_effector = end_effector_loader.createSharedInstance(plugin);
    }

    RCLCPP_INFO_STREAM(logger, "Successfully loaded plugin" << plugin << "of type " << plugin_type);
  }
  
  catch(pluginlib::PluginlibException& ex){
    RCLCPP_ERROR(node->get_logger(), "Gripper plugin failed to load. Error: %s\n", ex.what());
    rclcpp::shutdown();
    executor_thread.join();

    return 1;
  }

  if(plugin_type == "rcm_gripper::BasicGripper"){
    RCLCPP_INFO(logger, "Initializing plugin");
    gripper->init(node);

    if(start_services){
      RCLCPP_INFO(logger, "Initializing services");
      gripper->init_base_services();
    }
  }
  else if(plugin_type == "rcm_gripper::GenericEndEffector"){
    RCLCPP_INFO(logger, "Initializing plugin");
    end_effector->init(node);

    if(start_services){
      RCLCPP_INFO(logger, "Initializing services");
      end_effector->init_base_services();
    }
    if(start_action){
      RCLCPP_INFO(logger, "Initializing action server");
      end_effector->init_base_action();
    }
  }

  // Wait while executor is running
  executor_thread.join();
  rclcpp::shutdown();

  return 0;
}