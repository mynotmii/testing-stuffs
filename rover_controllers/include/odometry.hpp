
#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <cmath>

#include <rclcpp/time.hpp>

class odometry
{
public:
    // Update the position and orientation based on linear and angular velocities
    void updatePosition(double linear_velocity, double angular_velocity, const rclcpp::Time &current_time)
    {
        double dt = (current_time - last_update_time).seconds();
        if (dt <= 0.0) return; // Avoid division by zero

        x += linear_velocity * std::cos(theta) * dt;
        y += linear_velocity * std::sin(theta) * dt;
        theta += angular_velocity * dt;

        last_update_time = current_time;
    }

    // Reset the odometry values
    void reset()
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        linear_velocity = 0.0;
        angular_velocity = 0.0;
        last_update_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    // Getters for position and orientation
    double getX() const { return x; }
    double getY() const { return y; }
    double getTheta() const { return theta; }
    double getLinearVelocity() const { return linear_velocity; }
    double getAngularVelocity() const { return angular_velocity; }
    
    // Setters for linear and angular velocities
    void setLinearVelocity(double vel) { linear_velocity = vel; }
    void setAngularVelocity(double vel) { angular_velocity = vel; }

    // Set wheel parameters
    void setWheelParameters(double separation, double left_radius, double right_radius);

    // Get the current time
    rclcpp::Time getCurrentTime() const{ return last_update_time; }
    
private:
    // Position and orientation
    double x{0.0};  // [m]
    double y{0.0}; // [m]
    double theta{0.0}; // [rad]

    // Linear and angular velocities
    double linear_velocity{0.0}; // [m/s]
    double angular_velocity{0.0}; // [rad/s]

    // Wheel parameters
    double wheel_separation{0.0}; // [m]
    double left_wheel_radius{0.0}; // [m]
    double right_wheel_radius{0.0}; // [m]

    // Time of the last update
    rclcpp::Time last_update_time;
    

};

#endif // ODOMETRY_HPP