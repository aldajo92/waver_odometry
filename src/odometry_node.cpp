#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class OdometryFromVelocity : public rclcpp::Node {
public:
    OdometryFromVelocity() : Node("waver_odometry_node"), x_(0.0), y_(0.0), theta_(0.0) {
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        declare_parameter("odom_topic", "/odom");
        declare_parameter("vel_topic", "/vel_from_i2c");

        get_parameter("odom_frame", odom_frame_);
        get_parameter("base_frame", base_frame_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("vel_topic", vel_topic_);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
        vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            vel_topic_, 10,
            std::bind(&OdometryFromVelocity::velocityCallback, this, std::placeholders::_1));

        last_time_ = now();
    }

private:
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double v = msg->linear.x;
        double w = msg->angular.z;

        double delta_x = v * std::cos(theta_) * dt;
        double delta_y = v * std::sin(theta_) * dt;
        double delta_theta = w * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = w;

        odom_pub_->publish(odom);
    }

    double x_, y_, theta_;
    std::string odom_frame_, base_frame_, odom_topic_, vel_topic_;
    rclcpp::Time last_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryFromVelocity>());
    rclcpp::shutdown();
    return 0;
}
