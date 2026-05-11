#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("vision_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/error", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "C++ Vision Node started. Looking for green spheres...");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image;
            
            // Convert to HSV
            cv::Mat hsv_frame;
            cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

            // Define green color bounds (same as your Python prototype)
            cv::Scalar lower_green(35, 50, 50);
            cv::Scalar upper_green(85, 255, 255);

            // Thresholding
            cv::Mat mask;
            cv::inRange(hsv_frame, lower_green, upper_green, mask);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            std_msgs::msg::Float64 error_msg;

            if (!contours.empty()) {
                // Find largest contour
                double max_area = 0;
                size_t max_contour_idx = 0;
                for (size_t i = 0; i < contours.size(); i++) {
                    double area = cv::contourArea(contours[i]);
                    if (area > max_area) {
                        max_area = area;
                        max_contour_idx = i;
                    }
                }

                // Calculate center using moments
                cv::Moments M = cv::moments(contours[max_contour_idx]);
                if (M.m00 > 0) {
                    int cx = M.m10 / M.m00;
                    int image_center_x = frame.cols / 2;
                    error_msg.data = static_cast<double>(image_center_x - cx);
                } else {
                    error_msg.data = 9999.0;
                }
            } else {
                // Sentinel value for "not found"
                error_msg.data = 9999.0;
            }

            publisher_->publish(error_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
