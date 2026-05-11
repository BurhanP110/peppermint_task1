#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cstddef>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode()
        : Node("controller_node"),
          error_(0.0),
          min_distance_(kDefaultMinDistance),
          is_stopped_(false) {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        error_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/error", 10, std::bind(&ControllerNode::error_callback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ControllerNode::scan_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ControllerNode::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "C++ Controller Node started.");
    }

private:
    static constexpr double kStopDistance = 0.2;
    static constexpr double kSearchSentinelError = 9999.0;
    static constexpr double kAngularKp = 0.002;
    static constexpr double kForwardSpeed = 0.15;
    static constexpr double kSearchAngularSpeed = 0.3;
    static constexpr double kDefaultMinDistance = 10.0;
    static constexpr std::size_t kFrontWindowWidth = 10;
    void error_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        error_ = msg->data;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        min_distance_ = compute_front_min_distance(*msg);
    }

    void control_loop() {
        cmd_pub_->publish(compute_command());
    }

    bool is_obstacle_too_close() const {
        return min_distance_ <= kStopDistance;
    }

    bool should_search() const {
        return error_ == kSearchSentinelError;
    }

    geometry_msgs::msg::Twist make_stop_command() const {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    geometry_msgs::msg::Twist make_search_command() const {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = kSearchAngularSpeed;
        return cmd;
    }

    geometry_msgs::msg::Twist make_tracking_command() const {
        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = kAngularKp * error_;
        cmd.linear.x = kForwardSpeed;
        return cmd;
    }

    geometry_msgs::msg::Twist compute_command() {
        if (is_obstacle_too_close()) {
            if (!is_stopped_) {
                RCLCPP_INFO(this->get_logger(), "Reached object! Distance: %.2fm. Stopping.", min_distance_);
                is_stopped_ = true;
            }
            return make_stop_command();
        }

        is_stopped_ = false;
        if (should_search()) {
            return make_search_command();
        }
        return make_tracking_command();
    }

    bool is_valid_range(double range, const sensor_msgs::msg::LaserScan & scan) const {
        return range > scan.range_min && range < scan.range_max;
    }

    double compute_front_min_distance(const sensor_msgs::msg::LaserScan & scan) const {
        double current_min = scan.range_max;
        const std::size_t total_ranges = scan.ranges.size();
        if (total_ranges == 0) {
            return current_min;
        }

        const std::size_t head_end = std::min(kFrontWindowWidth, total_ranges - 1);
        for (std::size_t i = 0; i <= head_end; ++i) {
            if (is_valid_range(scan.ranges[i], scan)) {
                current_min = std::min(current_min, static_cast<double>(scan.ranges[i]));
            }
        }

        const std::size_t tail_start =
            (total_ranges > kFrontWindowWidth) ? (total_ranges - kFrontWindowWidth) : 0;
        for (std::size_t i = tail_start; i < total_ranges; ++i) {
            if (is_valid_range(scan.ranges[i], scan)) {
                current_min = std::min(current_min, static_cast<double>(scan.ranges[i]));
            }
        }

        return current_min;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double error_;
    double min_distance_;
    bool is_stopped_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
