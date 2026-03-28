#include <rclcpp/rclcpp.hpp>
#include <relbot_msgs/msg/relbot_motors.hpp>
#include <geometry_msgs/msg/point.hpp>

class ObjectFollower : public rclcpp::Node {
public:
    ObjectFollower() : Node("object_follower") {
       
        motor_pub_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
        
       
        position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_position", 10,
            std::bind(&ObjectFollower::position_callback, this, std::placeholders::_1));
        
       
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ObjectFollower::control_callback, this));
  
        this->declare_parameter("gain", 0.005);
        this->declare_parameter("base_speed", 0.5);
        this->declare_parameter("max_speed", 2.0);
        this->declare_parameter("image_width", 300.0);
        this->declare_parameter("dead_zone", 20.0);  
        
        // State
        latest_x_ = -1.0;
        latest_y_ = -1.0;
        object_detected_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Object Follower Started");
    }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_x_ = msg->x;
        latest_y_ = msg->y;
        object_detected_ = (msg->x >= 0);
        
        if (object_detected_) {
            RCLCPP_DEBUG(this->get_logger(), "Object at: x=%.1f, y=%.1f", msg->x, msg->y);
        }
    }
    
    void control_callback() {
        auto motor_msg = relbot_msgs::msg::RelbotMotors();
        
        if (!object_detected_) {
            
            motor_msg.left_wheel_vel = 0.0;
            motor_msg.right_wheel_vel = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "No object detected");
            motor_pub_->publish(motor_msg);
            return;
        }
        
        
        double gain = this->get_parameter("gain").as_double();
        double base_speed = this->get_parameter("base_speed").as_double();
        double max_speed = this->get_parameter("max_speed").as_double();
        double img_width = this->get_parameter("image_width").as_double();
        double dead_zone = this->get_parameter("dead_zone").as_double();
        
        
        double img_center = img_width / 2.0;
        double error_x = latest_x_ - img_center;
        
        
        RCLCPP_INFO(this->get_logger(), "Object x=%.1f, center=%.1f, error=%.1f", 
                   latest_x_, img_center, error_x);
        
        
        if (std::abs(error_x) < dead_zone) {
           
            motor_msg.left_wheel_vel = base_speed;
            motor_msg.right_wheel_vel = base_speed;
            RCLCPP_INFO(this->get_logger(), "CENTERED - going straight");
        } 
        else {
            
            double turn = gain * error_x;
            
            
            if (turn > max_speed) turn = max_speed;
            if (turn < -max_speed) turn = -max_speed;
            
            
            if (error_x > 0) {
                
                motor_msg.left_wheel_vel = base_speed + std::abs(turn);
                motor_msg.right_wheel_vel = base_speed - std::abs(turn);
                RCLCPP_INFO(this->get_logger(), "TURNING RIGHT: left=%.2f, right=%.2f", 
                           motor_msg.left_wheel_vel, motor_msg.right_wheel_vel);
            } 
            else {
                
                motor_msg.left_wheel_vel = base_speed - std::abs(turn);
                motor_msg.right_wheel_vel = base_speed + std::abs(turn);
                RCLCPP_INFO(this->get_logger(), "TURNING LEFT: left=%.2f, right=%.2f", 
                           motor_msg.left_wheel_vel, motor_msg.right_wheel_vel);
            }
            
            
            if (motor_msg.left_wheel_vel < 0.1) motor_msg.left_wheel_vel = 0.1;
            if (motor_msg.right_wheel_vel < 0.1) motor_msg.right_wheel_vel = 0.1;
        }
        
       
        if (motor_msg.left_wheel_vel > max_speed) motor_msg.left_wheel_vel = max_speed;
        if (motor_msg.right_wheel_vel > max_speed) motor_msg.right_wheel_vel = max_speed;
        
        motor_pub_->publish(motor_msg);
    }
    
  
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
   
    double latest_x_, latest_y_;
    bool object_detected_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectFollower>());
    rclcpp::shutdown();
    return 0;
}