
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
public:
  DirectionService();

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr srv_get_direction;

  void srv_get_direction_clb(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> req,
      const std::shared_ptr<robot_patrol::srv::GetDirection::Response> res);
};

// Class DirectionService function definition
//<-------------------->
DirectionService::DirectionService() : Node("service_node") {
  RCLCPP_INFO(this->get_logger(), "Service node created");

  // Create service
  this->srv_get_direction =
      this->create_service<robot_patrol::srv::GetDirection>(
          "/direction_server",
          std::bind(&DirectionService::srv_get_direction_clb, this,
                    std::placeholders::_1, std::placeholders::_2));
}

void DirectionService::srv_get_direction_clb(
    const std::shared_ptr<robot_patrol::srv::GetDirection::Request> req,
    const std::shared_ptr<robot_patrol::srv::GetDirection::Response> res) {

  RCLCPP_INFO(this->get_logger(), "Service called");

  std::list<float> scan_arr; // Create list for useful data points
  // Copy only fron of laser scan data
  std::copy(req->laser_data.ranges.begin() + 180,
            req->laser_data.ranges.begin() + 540,
            std::front_inserter(scan_arr));

  // Divide fron sector in 3 regions: left, middle and right
  // Find minimum value for each segment
  std::list<float>::iterator left_scan =
      std::min_element(scan_arr.begin(), std::next(scan_arr.begin(), 119));
  std::list<float>::iterator middle_scan = std::min_element(
      std::next(scan_arr.begin(), 120), std::next(scan_arr.begin(), 239));
  std::list<float>::iterator right_scan = std::min_element(
      std::next(scan_arr.begin(), 240), std::next(scan_arr.begin(), 359));

  // Check safest sector and return answer
  if (*left_scan > *middle_scan && *left_scan > *right_scan) {
    res->direction = "left";
  } else if (*middle_scan > *left_scan && *middle_scan > *right_scan) {
    res->direction = "forward";
  } else {
    res->direction = "right";
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr my_node = std::make_shared<rclcpp::Node>("my_node");
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();

  return 0;
}