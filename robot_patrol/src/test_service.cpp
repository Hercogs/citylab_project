#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <future>
#include <memory>

class TestService : public rclcpp::Node {
public:
  TestService(); // Constructor

private:
  // Class members
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr cli_get_direction;

  rclcpp::CallbackGroup::SharedPtr client_clb_group;

  // Class functions
  void sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

// <-------------------------------->
// TestService classs function definitions
TestService::TestService() : Node("test_service_node") {
  RCLCPP_INFO(this->get_logger(),
              "TestService node created, waiting for service available");

  // Define callback groups
  this->client_clb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create subscribers
  this->sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 5,
      std::bind(&TestService::sub_laser_clb, this, std::placeholders::_1));

  // Create client
  this->cli_get_direction =
      this->create_client<robot_patrol::srv::GetDirection>(
          "/direction_server", rmw_qos_profile_services_default,
          this->client_clb_group);
}

void TestService::sub_laser_clb(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Call service only once

  static bool flag = false;
  if (flag)
    return;

  while (!cli_get_direction->wait_for_service(std::chrono::seconds(1))) {
    flag = true;
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
  request->laser_data = *msg;

  auto result_future = this->cli_get_direction->async_send_request(request);

  std::future_status status = result_future.wait_for(
      std::chrono::seconds(1)); // timeout to guarantee a graceful finish

  if (status == std::future_status::ready) {
    auto result = result_future.get();

    RCLCPP_INFO(this->get_logger(), "Answer: %s", result->direction.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "NOT received response");
  }
  

  rclcpp::shutdown();
}

// <-------------------------------->
// MAIN
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<TestService>();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(client_node);

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
