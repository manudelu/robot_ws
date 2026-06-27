#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Core>

class Controller : public rclcpp::Node {
    public: 
        Controller(const std::string& name);

    private:
        void velCallback(const geometry_msgs::msg::TwistStamped& msg);

        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

        double wheel_radius_;
        double wheel_separation_;
        Eigen::Matrix2d speed_conversion_;
};

#endif