#include "rclcpp/client.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/detail/get_direction__struct.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TestService : public rclcpp::Node {
public:
  TestService(); // Constructor

private:
  // Class members
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr cli_get_direction;

  // Class functions
  void sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

// <-------------------------------->
// Patrol classs function definitions
TestService::TestService() : Node("test_service_node") {
  RCLCPP_INFO(this->get_logger(), "TestService node created");

  // Create subscribers
  this->sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 5,
      std::bind(&TestService::sub_laser_clb, this, std::placeholders::_1));

  // Create client
  this->cli_get_direction =
      this->create_client<robot_patrol::srv::GetDirection>("/direction_server");
}

void TestService::sub_laser_clb(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Call service

  RCLCPP_INFO(this->get_logger(), "Lser clb");
}

// <-------------------------------->
// MAIN
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("test_service_node");

  rclcpp::spin_some(std::make_shared<TestService>());
  rclcpp::shutdown();

  return 0;
}