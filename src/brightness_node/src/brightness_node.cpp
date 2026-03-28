#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

class BrightnessNode : public rclcpp::Node {
public:
    BrightnessNode() : Node("brightness_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                process(msg);
            });
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/light_status", 10);
        threshold_ = this->declare_parameter<double>("brightness_threshold", 100.0);
        RCLCPP_INFO(this->get_logger(), "Started with threshold: %f", threshold_);
    }

private:
    void process(const sensor_msgs::msg::Image::SharedPtr msg) {
        
        if (msg->encoding != "bgr8") return;
        
        long sum = 0;
        int pixels = msg->width * msg->height;
        
        
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            
            sum += (msg->data[i] + msg->data[i+1] + msg->data[i+2]) / 3;
        }
        
        double brightness = (double)sum / pixels;
        bool light = brightness > threshold_;
        
        auto msg_out = std_msgs::msg::Bool();
        msg_out.data = light;
        publisher_->publish(msg_out);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    double threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessNode>());
    rclcpp::shutdown();
    return 0;
}