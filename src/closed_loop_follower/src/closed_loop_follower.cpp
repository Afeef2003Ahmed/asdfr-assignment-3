// closed_loop_follower.cpp - Fixed with proper theta control
#include <rclcpp/rclcpp.hpp>
#include <relbot_msgs/msg/relbot_motors.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class ClosedLoopFollower : public rclcpp::Node {
public:
    ClosedLoopFollower() : Node("closed_loop_follower") {
        // Publisher for motor commands
        motor_pub_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
        
        // Subscribe to object position (from moving camera)
        position_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_position", 10,
            std::bind(&ClosedLoopFollower::position_callback, this, std::placeholders::_1));
        
        // Subscribe to robot pose
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/output/robot_pose", 10,
            std::bind(&ClosedLoopFollower::pose_callback, this, std::placeholders::_1));
        
        // Control loop at 50 Hz (20ms)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ClosedLoopFollower::control_callback, this));
        
        // Parameters for full state control
        this->declare_parameter("tau_x", 1.0);      // Time constant for x
        this->declare_parameter("tau_theta", 0.8);  // Time constant for theta
        this->declare_parameter("pixel_to_meter", 0.002);  // Conversion factor
        this->declare_parameter("image_width", 300.0);
        this->declare_parameter("max_speed", 2.0);
        
        // State variables
        latest_x_pixel_ = -1.0;
        current_x_ = 0.0;
        current_theta_ = 0.0;
        setpoint_x_ = 0.0;
        setpoint_theta_ = 0.0;
        object_detected_ = false;
        last_time_ = this->now();
        img_center_ = this->get_parameter("image_width").as_double() / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Closed Loop Follower Started - Full state control with theta");
        RCLCPP_INFO(this->get_logger(), "Control: x_error = (pixel_pos - center) * %.3f, theta_error = 0", 
                   this->get_parameter("pixel_to_meter").as_double());
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
        
        // Get parameters
        double tau_x = this->get_parameter("tau_x").as_double();
        double tau_theta = this->get_parameter("tau_theta").as_double();
        double pixel_to_meter = this->get_parameter("pixel_to_meter").as_double();
        double max_speed = this->get_parameter("max_speed").as_double();
        
        // Calculate time step
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt > 0.1) dt = 0.02;  // Cap dt
        last_time_ = now;
        
        // Error in x (position error)
        double error_x_pixel = latest_x_pixel_ - img_center_;
        double error_x_meter = error_x_pixel * pixel_to_meter;
        
        // Error in theta (orientation error) - this is the crucial part!
        // The camera position relative to robot gives theta error
        // For a differential drive robot, we need to align with the object
        double error_theta = 0.0;
        
        // Method 1: Use pixel offset to compute required rotation
        // If object is to the right (positive error), need positive theta
        // This creates differential steering
        double theta_correction = 0.0;
        if (std::abs(error_x_pixel) > 30.0) {  // If object is significantly off-center
            theta_correction = error_x_pixel / img_center_ * 1.5;  // Map to [-1.5, 1.5] rad
        }
        
        // Combined error for theta
        // We want robot to orient towards the object
        error_theta = theta_correction;
        
        // First-order controller for x
        double dx_dt = error_x_meter / tau_x;
        
        // First-order controller for theta
        double dtheta_dt = error_theta / tau_theta;
        
        // Update setpoints using Euler integration
        setpoint_x_ += dx_dt * dt;
        setpoint_theta_ += dtheta_dt * dt;
        
        // Limit setpoints
        if (setpoint_x_ > 5.0) setpoint_x_ = 5.0;
        if (setpoint_x_ < -5.0) setpoint_x_ = -5.0;
        if (setpoint_theta_ > 3.14) setpoint_theta_ = 3.14;
        if (setpoint_theta_ < -3.14) setpoint_theta_ = -3.14;
        
        // Convert to wheel velocities (differential drive)
        // v = linear velocity, ω = angular velocity
        double v = dx_dt;
        double omega = dtheta_dt;
        
        // Robot kinematics: left = v - ω*L/2, right = v + ω*L/2
        // L is wheel separation (~0.25m for RELbot)
        double L = 0.25;
        
        motor_msg.left_wheel_vel = v - (omega * L / 2.0);
        motor_msg.right_wheel_vel = v + (omega * L / 2.0);
        
        // Limit speeds
        if (motor_msg.left_wheel_vel > max_speed) motor_msg.left_wheel_vel = max_speed;
        if (motor_msg.left_wheel_vel < -max_speed) motor_msg.left_wheel_vel = -max_speed;
        if (motor_msg.right_wheel_vel > max_speed) motor_msg.right_wheel_vel = max_speed;
        if (motor_msg.right_wheel_vel < -max_speed) motor_msg.right_wheel_vel = -max_speed;
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "Error: x=%.2f m, θ=%.2f rad | Vel: L=%.2f, R=%.2f", 
            error_x_meter, error_theta, motor_msg.left_wheel_vel, motor_msg.right_wheel_vel);
        
        motor_pub_->publish(motor_msg);
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // State variables
    double latest_x_pixel_;
    double current_x_, current_theta_;
    double setpoint_x_, setpoint_theta_;
    bool object_detected_;
    rclcpp::Time last_time_;
    double img_center_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClosedLoopFollower>());
    rclcpp::shutdown();
    return 0;
}