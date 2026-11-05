// Standard rclcpp host for the pinned upstream ServoNode plus provenance.
#include <moveit_servo/servo_node.hpp>
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<moveit_servo::ServoNode>(rclcpp::NodeOptions());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
}
