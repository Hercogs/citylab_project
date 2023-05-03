#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <list>
#include <memory>
#include <ostream>

class Patrol : public rclcpp::Node {
public:
  Patrol(); // Constructor

private:
  // Class members
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr cli_get_direction;
  rclcpp::CallbackGroup::SharedPtr client_clb_group;

  geometry_msgs::msg::Twist pub_velocity_msg;

  // Class functions
  void sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

// <-------------------------------->
// Patrol classs function definitions
Patrol::Patrol() : Node("patrol_v2_node") {
  RCLCPP_INFO(this->get_logger(), "Patrol v2 node created");

  // Define callback groups
  this->client_clb_group =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // Create client
  this->cli_get_direction =
      this->create_client<robot_patrol::srv::GetDirection>(
          "/direction_server", rmw_qos_profile_services_default,
          this->client_clb_group);

  // Create publishers
  this->pub_velocity =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);

  // Create subscribers
  this->sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 5,
      std::bind(&Patrol::sub_laser_clb, this, std::placeholders::_1));
}

void Patrol::sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // To avoid send service while previous is not finished, use flag
  static bool service_active_flag = false;
  if (service_active_flag)
    return;

  if (!cli_get_direction->wait_for_service()) {
    return; // Service not available
  }

  // Send service request
  service_active_flag = true;

  auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
  request->laser_data = *msg;

  auto result_future = this->cli_get_direction->async_send_request(request);

  std::future_status status = result_future.wait_for(
      std::chrono::milliseconds(500)); // timeout to guarantee a graceful finish

  // Check future status
  if (status == std::future_status::ready) {
    auto result = result_future.get();

    // Make decision based on service reponse
    if (result->direction == "forward") {
      this->pub_velocity_msg.linear.x = 0.1;
      this->pub_velocity_msg.angular.z = 0.0;
    } else if (result->direction == "left") {
      this->pub_velocity_msg.linear.x = 0.05;
      this->pub_velocity_msg.angular.z = 0.2;
    } else if (result->direction == "right") {
      this->pub_velocity_msg.linear.x = 0.05;
      this->pub_velocity_msg.angular.z = -0.2;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown response");
    }

  } else {
    RCLCPP_ERROR(this->get_logger(), "NOT received response");
    service_active_flag = false;
    return;
  }

  // Publish velocity
  this->pub_velocity->publish(this->pub_velocity_msg);
  service_active_flag = false;
}

// <-------------------------------->
// MAIN
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(patrol_node);
  exe.spin();
  rclcpp::shutdown();

  return 0;
}