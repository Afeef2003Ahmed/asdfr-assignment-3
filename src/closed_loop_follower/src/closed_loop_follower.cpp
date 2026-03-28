#include <rclcpp/rclcpp.hpp>
#include <relbot_msgs/msg/relbot_motors.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


class ClosedLoopFollower : public rclcpp::Node {
public:
    ClosedLoopFollower() : Node("closed_loop_follower") {
        
        motor_pub_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
        
        
        position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_position", 10,
            std::bind(&ClosedLoopFollower::position_callback, this, std::placeholders::_1));
        
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/output/robot_pose", 10,
            std::bind(&ClosedLoopFollower::pose_callback, this, std::placeholders::_1));
        
        
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ClosedLoopFollower::control_callback, this));
        
        
        this->declare_parameter("tau", 1.0);           
        this->declare_parameter("max_speed", 2.0);  
        this->declare_parameter("pixel_to_meter", 0.001);
        this->declare_parameter("img_center", 150.0);  
        
       
        latest_x_pixel_ = -1.0;
        current_x_ = 0.0;
        current_theta_ = 0.0;
        setpoint_x_ = 0.0;
        setpoint_theta_ = 0.0;
        object_detected_ = false;
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Closed Loop Follower Started");
    }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        latest_x_pixel_ = msg->x;
        object_detected_ = (msg->x >= 0);
    }
    
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x_ = msg->pose.position.x;
        current_theta_ = msg->pose.orientation.z;
    }
    
    void control_callback() {
        auto motor_msg = relbot_msgs::msg::RelbotMotors();
        
        if (!object_detected_) {
            motor_msg.left_wheel_vel = 0.0;
            motor_msg.right_wheel_vel = 0.0;
            motor_pub_->publish(motor_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "No object detected");
            return;
        }
        
        
        double tau = this->get_parameter("tau").as_double();
        double max_speed = this->get_parameter("max_speed").as_double();
        double pixel_to_meter = this->get_parameter("pixel_to_meter").as_double();
        double img_center = this->get_parameter("img_center").as_double();
        
        
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt > 0.1) dt = 0.02;  
        
        
        double error_x = (latest_x_pixel_ - img_center) * pixel_to_meter;
        double error_theta = 0.0;  
        
       
        double dx_dt = error_x / tau;
        double dtheta_dt = error_theta / tau;
       
        setpoint_x_ += dx_dt * dt;
        setpoint_theta_ += dtheta_dt * dt;
        
        double speed = dx_dt;
        
        
        if (speed > max_speed) speed = max_speed; 
        if (speed < -max_speed) speed = -max_speed;
        
        

        double min_speed = 0.2;
        if (std::abs(speed) < min_speed && std::abs(speed) > 0.01) { 
            speed = (speed > 0) ? min_speed : -min_speed;
        }
        
        motor_msg.left_wheel_vel = speed;
        motor_msg.right_wheel_vel = speed;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "error=%.3f m, speed=%.2f, setpoint_x=%.3f", 
            error_x, speed, setpoint_x_);
        
        motor_pub_->publish(motor_msg);
    }
    
   
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    

    double latest_x_pixel_;
    double current_x_, current_theta_;
    double setpoint_x_, setpoint_theta_;
    bool object_detected_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClosedLoopFollower>());
    rclcpp::shutdown();
    return 0;
}