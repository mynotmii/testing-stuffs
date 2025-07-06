#include <memory>
#include <vector>

#include "rat3_controllers.hpp"
#include "odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

RoverController::RoverController(const rclcpp::NodeOptions &options)
: Node("rover_controller", options),
  odometry_(std::make_shared<odometry>()),
  wheel_separation_(0.5), // Example value, adjust as needed
  left_wheel_radius_(0.1), // Example value, adjust as needed
  right_wheel_radius_(0.1) // Example value, adjust as needed
{
    // Initialize the odometry with wheel parameters
    odometry_->setWheelParameters(wheel_separation_, left_wheel_radius_, right_wheel_radius_);

    // Create a subscriber for the cmd_vel topic
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist  {
        "cmd_vel",
        rclcpp::QoS(10),
        std::bind(&RoverController::cmdVelCallback, this, std::placeholders::_1)
    }

    // TODO: Create a publisher for the odometry topic

};