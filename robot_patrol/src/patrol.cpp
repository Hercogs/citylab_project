#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cstddef>
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

  geometry_msgs::msg::Twist pub_velocity_msg;

  // Class functions
  void sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

// <-------------------------------->
// Patrol classs function definitions
Patrol::Patrol() : Node("patrol_node") {
  RCLCPP_INFO(this->get_logger(), "Patrol node created");

  // Create publishers
  this->pub_velocity =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);

  // Create subscribers
  this->sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 5,
      std::bind(&Patrol::sub_laser_clb, this, std::placeholders::_1));
}

void Patrol::sub_laser_clb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  std::list<float> scan_arr; // Create list for useful data points

  // Copy only fron of laser scan data
  std::copy(msg->ranges.begin() + 180, msg->ranges.begin() + 540,
            std::front_inserter(scan_arr));

/*
  RCLCPP_INFO(this->get_logger(), "Scan:");

  for (auto x : scan_arr) {
    std::cout << x << " " << std::endl;
  }
  std::cout << std::endl;
  */

  // Divide fron sector in 3 regions: left, middle and right
  // Find minimum value for each segment
  std::list<float>::iterator left_scan =
      std::min_element(scan_arr.begin(), std::next(scan_arr.begin(), 119));
  std::list<float>::iterator middle_scan = std::min_element(
      std::next(scan_arr.begin(), 120), std::next(scan_arr.begin(), 239));
  std::list<float>::iterator right_scan = std::min_element(
      std::next(scan_arr.begin(), 240), std::next(scan_arr.begin(), 359));

  // Print secot values for debugging
  RCLCPP_INFO(this->get_logger(), "Left: %d: %.2f", left_scan, *left_scan);
  RCLCPP_INFO(this->get_logger(), "Middle: %d %.2f", middle_scan, *middle_scan);
  RCLCPP_INFO(this->get_logger(), "Right: %d %.2f", right_scan, *right_scan);

  const float threshold_low = 0.30;
  const float threshold_high = 0.50;

  if (*middle_scan < threshold_low) {
    // Front sector has object critically close to robot
    if (*left_scan < *right_scan) {
      // Turn right
      this->pub_velocity_msg.linear.x = 0.0;
      this->pub_velocity_msg.angular.z = -0.2;
    } else {
      // Turn left
      this->pub_velocity_msg.linear.x = 0.0;
      this->pub_velocity_msg.angular.z = 0.2;
    }
  } else if (*middle_scan < threshold_high) {
    // Front sector has object in front of robot in some distance
    if (*left_scan < *right_scan) {
      // Turn slightly right
      this->pub_velocity_msg.linear.x = 0.1;
      this->pub_velocity_msg.angular.z =
          -0.25 - (threshold_high - *middle_scan); // Simple P controller
    } else {
      // Turn sliughtly left
      this->pub_velocity_msg.linear.x = 0.1;
      this->pub_velocity_msg.angular.z =
          0.25 + (threshold_high - *middle_scan); // Simple P controller
    }
  } else {
    this->pub_velocity_msg.linear.x = 0.1;
    this->pub_velocity_msg.angular.z = 0.0;
  }

  // Publish velocity
  this->pub_velocity->publish(this->pub_velocity_msg);
}

// <-------------------------------->
// MAIN
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();

  return 0;
}