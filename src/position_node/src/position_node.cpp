#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>

class PositionNode : public rclcpp::Node {
public:
    PositionNode() : Node("position_node") {
        threshold_ = this->declare_parameter<double>("color_threshold", 230.0);
        
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                process(msg);
            });
        pub_ = this->create_publisher<geometry_msgs::msg::Point>("/object_position", 10);
    }

private:
    void process(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->encoding != "bgr8") return;
        
        long sum_x = 0, sum_y = 0, count = 0;
        
        for (int y = 0; y < msg->height; y++) {
            for (int x = 0; x < msg->width; x++) {
                int idx = (y * msg->width + x) * 3;
                int brightness = (msg->data[idx] + msg->data[idx+1] + msg->data[idx+2]) / 3;
                
                if (brightness > threshold_) {
                    sum_x += x;
                    sum_y += y;
                    count++;
                }
            }
        }
        
        geometry_msgs::msg::Point position;
        
        if (count > 0) {
            position.x = (double)sum_x / count;
            position.y = (double)sum_y / count;
            position.z = (double)count;
        } else {
            position.x = -1;
            position.y = -1;
            position.z = 0;
        }
        
        pub_->publish(position);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;
    double threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionNode>());
    rclcpp::shutdown();
    return 0;
}
