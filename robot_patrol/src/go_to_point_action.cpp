#include "geometry_msgs/msg/detail/point32__struct.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "robot_patrol/action/go_to_point.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <thread>

class GoToPoint : public rclcpp::Node {
public:
  GoToPoint();

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity;
  geometry_msgs::msg::Twist pub_velocity_msg;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;
  nav_msgs::msg::Odometry sub_odometry_data;

  rclcpp_action::Server<robot_patrol::action::GoToPoint>::SharedPtr
      action_server;

  bool action_server_busy = false;

  void sub_odometry_clb(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const robot_patrol::action::GoToPoint::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
          goal_handle);

  void handle_accepted(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
          goal_handle);

  void execute_action(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
          goal_handle);

  // Convert quaternion to Euler
  float get_yaw_angle(tf2::Quaternion q) const;

  // Get disance bet ween 2 points
  float get_distance(geometry_msgs::msg::Point32 x1,
                     geometry_msgs::msg::Point32 x2) const;

  // Get slope bet ween 2 points
  float get_slope(geometry_msgs::msg::Point32 x1,
                  geometry_msgs::msg::Point32 x2) const;

  // Convert 360 degree scale to 180 degrees
  float degrees_360_180(float degrees) const;
};

// <-------------------------------->
// GoToPoint classs function definitions
GoToPoint::GoToPoint() : Node("action_node") {
  RCLCPP_INFO(this->get_logger(), "Action node created");

  // Create publishers
  this->pub_velocity =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);

  // Create subscribers
  this->sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 5,
      std::bind(&GoToPoint::sub_odometry_clb, this, std::placeholders::_1));

  // Create action server
  using namespace std::placeholders;
  this->action_server =
      rclcpp_action::create_server<robot_patrol::action::GoToPoint>(
          this, "go_to_point", std::bind(&GoToPoint::handle_goal, this, _1, _2),
          std::bind(&GoToPoint::handle_cancel, this, _1),
          std::bind(&GoToPoint::handle_accepted, this, _1));
}

void GoToPoint::sub_odometry_clb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  this->sub_odometry_data = *msg; // Save data
}

rclcpp_action::GoalResponse GoToPoint::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const robot_patrol::action::GoToPoint::Goal> goal) {
  // RCLCPP_INFO(this->get_logger(), "Received action with goal: TODO");
  (void)uuid;
  (void)goal;

  // TODO handle multiple requests
  if (action_server_busy) {
    RCLCPP_WARN(this->get_logger(),
                "Multiple action is not possible. Please cancel previous "
                "action goal to send new one!");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    action_server_busy = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}

rclcpp_action::CancelResponse GoToPoint::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GoToPoint::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
        goal_handle) {
  using namespace std::placeholders;

  std::thread(std::bind(&GoToPoint::execute_action, this, _1), goal_handle)
      .detach();
}

void GoToPoint::execute_action(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPoint>>
        goal_handle) {

  // Set up variables
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<robot_patrol::action::GoToPoint::Feedback>();
  auto result = std::make_shared<robot_patrol::action::GoToPoint::Result>();
  result->status = false;

  rclcpp::Rate loop_rate(2);

  // Print setpoint
  RCLCPP_INFO(this->get_logger(),
              "Executing action goal x: %.2f, y: %.2f, angle: %.2f",
              goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.z);

  geometry_msgs::msg::Point32 current_pos, goal_pos;
  goal_pos = goal->goal_pos;

  int rate_cnt = 0; // For controlling feedback rate

  for (size_t i = 0; i < 240; i++) { // Limit execuation to 2 minutes
    rate_cnt++;
    // Check if cancel request is made
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Goal is canceled");
      // Stop robot
      pub_velocity_msg.linear.x = 0.0;
      pub_velocity_msg.angular.z = 0.0;
      pub_velocity->publish(pub_velocity_msg);

      action_server_busy = false;
      goal_handle->canceled(result);

      return;
    }

    // Save current pos
    current_pos.x = sub_odometry_data.pose.pose.position.x;
    current_pos.y = sub_odometry_data.pose.pose.position.y;
    // Convert to tf2::Quaternion
    tf2::Quaternion q(sub_odometry_data.pose.pose.orientation.x,
                      sub_odometry_data.pose.pose.orientation.y,
                      sub_odometry_data.pose.pose.orientation.z,
                      sub_odometry_data.pose.pose.orientation.w);

    // Get orientation in degrees
    current_pos.z = this->get_yaw_angle(q);

    // Save current pos in feedback meesage
    feedback->current_pos = current_pos;

    if (rate_cnt > 1) {
      goal_handle->publish_feedback(feedback);
      rate_cnt = 0;
    }

    // RCLCPP_INFO(this->get_logger(), "Publish feedback");

    // Claculate distance and slope
    float distance, slope;
    distance = this->get_distance(current_pos, goal_pos);
    slope = this->get_slope(current_pos, goal_pos);

    RCLCPP_DEBUG(this->get_logger(), "D: %.2f, S: %.2f", distance, slope);

    // Implemment logic

    float distance_err = 0.1;
    float angle_err = 5.0;

    if (std::abs(distance) < distance_err &&
        std::abs(degrees_360_180(goal_pos.z - current_pos.z)) < angle_err) {
      RCLCPP_INFO(this->get_logger(),
                  "Robot reached goal state-> x: %.2f y: %.2f z: %.2f",
                  current_pos.x, current_pos.y, current_pos.z);
      result->status = true;
      break;
    } else if (std::abs(distance) >= distance_err) {
      if (std::abs(degrees_360_180(slope - current_pos.z)) > angle_err) {
        // Only turn the robot
        RCLCPP_DEBUG(this->get_logger(), "Turning: %.2f",
                     degrees_360_180(slope - current_pos.z));
        if (degrees_360_180(slope - current_pos.z) > 0) {
          pub_velocity_msg.linear.x = 0.0;
          pub_velocity_msg.angular.z =
              0.08 + 0.001 * std::abs(degrees_360_180(slope - current_pos.z));
        } else {
          pub_velocity_msg.linear.x = 0.0;
          pub_velocity_msg.angular.z =
              -0.08 - 0.001 * std::abs(degrees_360_180(slope - current_pos.z));
        }
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Moving");
        pub_velocity_msg.linear.x = 0.05;
        if (std::abs(degrees_360_180(slope - current_pos.z)) > angle_err) {
          if (slope - current_pos.z > 0) {
            pub_velocity_msg.angular.z = 0.1;
          } else if (degrees_360_180(slope - current_pos.z) < 0) {
            pub_velocity_msg.angular.z = -0.1;
          }
        } else {
          pub_velocity_msg.angular.z = 0.0;
        }
      }
    } else {
      if (std::abs(degrees_360_180(goal_pos.z - current_pos.z)) > angle_err) {
        RCLCPP_DEBUG(this->get_logger(), "Turning at target");
        if (degrees_360_180(goal_pos.z - current_pos.z) > 0) {
          pub_velocity_msg.linear.x = 0.0;
          pub_velocity_msg.angular.z =
              0.07 +
              0.001 * std::abs(degrees_360_180(goal_pos.z - current_pos.z));
        } else {
          pub_velocity_msg.linear.x = 0.0;
          pub_velocity_msg.angular.z =
              -0.07 -
              0.001 * std::abs(degrees_360_180(goal_pos.z - current_pos.z));
        }
      }

      // RCLCPP_INFO(this->get_logger(), "Stopped");
      // pub_velocity_msg.linear.x = 0.0;
      // pub_velocity_msg.angular.z = 0.0;
    }

    pub_velocity->publish(pub_velocity_msg);
    loop_rate.sleep();
  }

  // Stop robot
  pub_velocity_msg.linear.x = 0.0;
  pub_velocity_msg.angular.z = 0.0;
  pub_velocity->publish(pub_velocity_msg);

  goal_handle->succeed(result);

  action_server_busy = false;
}

float GoToPoint::get_yaw_angle(tf2::Quaternion q) const {
  tf2::Matrix3x3 m(q);

  double roll, pitch, yaw;

  m.getRPY(roll, pitch, yaw);

  return (yaw * 180.00 / 3.14);
}

float GoToPoint::get_distance(geometry_msgs::msg::Point32 x1,
                              geometry_msgs::msg::Point32 x2) const {
  float distance =
      std::sqrt(std::pow(x2.x - x1.x, 2) + std::pow(x2.y - x1.y, 2));

  return distance;
}

float GoToPoint::get_slope(geometry_msgs::msg::Point32 x1,
                           geometry_msgs::msg::Point32 x2) const {
  float slope_rad, slope_deg;
  slope_rad = (x2.y - x1.y) / (x2.x - x1.x);

  if (x2.x < x1.x && x2.y < x1.y) {
    // 3rd quadrant
    slope_deg = std::atan(slope_rad) * 180.0 / 3.14 + 180;
  } else if (x2.x < x1.x && x2.y > x1.y) {
    // 2nd quadrant
    slope_deg = std::atan(slope_rad) * 180.0 / 3.14 + 180;
  } else {
    slope_deg = std::atan(slope_rad) * 180.0 / 3.14;
  }

  // RCLCPP_INFO(this->get_logger(), "Slope in degrees: %.2f", slope_deg);

  // Reduce to 180 degrees because of odometry data
  return (degrees_360_180(slope_deg));
}

float GoToPoint::degrees_360_180(float degrees) const {

  float scaled_degrees;

  // RCLCPP_INFO(this->get_logger(), "IN degrees: %.2f", degrees);

  if (degrees > 180.0) {
    scaled_degrees = degrees - 360.0;
  } else if (degrees < -180.0) {
    scaled_degrees = degrees + 360.0;
  } else {
    scaled_degrees = degrees;
  }

  // RCLCPP_INFO(this->get_logger(), "OUT degrees: %.2f", scaled_degrees);

  // Why these line do not work???
  // scaled_degrees = (degrees > 180.0) ? degrees - 360.0 : degrees;
  // scaled_degrees = (degrees < -180.0) ? degrees + 360.0 : degrees;

  return scaled_degrees;
}

// <-------------------------------->
// MAIN
int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  std::shared_ptr<GoToPoint> action_node = std::make_shared<GoToPoint>();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(action_node);
  exe.spin();
  rclcpp::shutdown();

  return 0;
}