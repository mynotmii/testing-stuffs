#ifndef ROVER_CONTROLLERS_HPP
#define ROVER_CONTROLLERS_HPP

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rclrcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "odometry.hpp"

namespace rover_controllers
{
    class RoverController : public rclcpp::Node
    {
    public:
        RoverDiffDriveController(const std::string &name);
        ~RoverDiffDriveController() override;

        void setOdometry(const Odometry &odometry);
        void setTargetVelocity(const geometry_msgs::msg::Twist &target_velocity);

    private:
        void controlLoop();

        Odometry odometry_;
        geometry_msgs::msg::Twist target_velocity_;
        rclcpp::TimerBase::SharedPtr control_timer_;
    };
} // namespace rover_controllers 
#endif // ROVER_CONTROLLERS_HPP
