#include <rclcpp/rclcpp.hpp>
#include <relbot_msgs/msg/relbot_motors.hpp>
#include <cmath>

class SequenceController : public rclcpp::Node {
public:
    SequenceController() : Node("sequence_controller"), step_(0), rotation_start_time_(this->now()) {
       
        pub_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
        
        // Main timer for sequence
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Check every 100ms for better control
            std::bind(&SequenceController::timer_callback, this));
        
        // Timer for logging
        log_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SequenceController::log_callback, this));
        
        start_time_ = this->now();
        rotation_start_time_ = start_time_;
        step_start_time_ = start_time_;
        
        // step durations (in seconds)
        step_durations_[0] = 4.0;   // Forward 1
        step_durations_[1] = 6.0;   // Right turn
        step_durations_[2] = 4.0;   // Forward 2
        step_durations_[3] = 6.0;   // Left turn
        step_durations_[4] = 4.0;   // Forward 3
        step_durations_[5] = 6.0;   // Backward
        step_durations_[6] = 3.0;   // Stop
        
        RCLCPP_INFO(this->get_logger(), "Sequence Controller Started - Extended trajectory with 360-degree turns");
        RCLCPP_INFO(this->get_logger(), "Forward: 8 seconds, Turns: 6 seconds, Backward: 8 seconds");
        RCLCPP_INFO(this->get_logger(), "Expected turning angle at 1.0 rad/s: 6 rad ≈ 344 degrees");
        
    }

private:
    void timer_callback() {
        auto msg = relbot_msgs::msg::RelbotMotors();
        auto now = this->now();
        double step_elapsed = (now - step_start_time_).seconds();
        
        
        if (step_elapsed >= step_durations_[step_ % 7]) {
            step_++;
            step_start_time_ = now;
            
            
            RCLCPP_INFO(this->get_logger(), "--- Step %d started at %.3f seconds ---", 
                       step_ % 7, (now - start_time_).seconds());
        }
        
        
        int current_step = step_ % 7;
        
        switch(current_step) {
            case 0:  // Forward 1
                msg.left_wheel_vel = 2.5;
                msg.right_wheel_vel = 2.5;
                break;
                
            case 1:  // Turn Right
                msg.left_wheel_vel = 7;
                msg.right_wheel_vel = -7;
                break;
                
            case 2:  // Forward 2
                msg.left_wheel_vel = -2.5;
                msg.right_wheel_vel = -2.5;
                break;
                
            case 3:  // Turn left
                msg.left_wheel_vel = -7;
                msg.right_wheel_vel = 7;
                break;
                
            case 4:  // Forward 3
                msg.left_wheel_vel = 2.5;
                msg.right_wheel_vel = 2.5;
                break;
                
            case 5:  // Backward
                msg.left_wheel_vel = 7.0;
                msg.right_wheel_vel = -7.0;
                break;
                
            case 6:  // Stop 
                msg.left_wheel_vel = 0.0;
                msg.right_wheel_vel = 0.0;
                break;
        }
        
        
        if (current_step == 1 || current_step == 3) {
            double turn_duration = step_elapsed;
            double turn_rate = 2.5;  // rad/s
            double angle_deg = turn_rate * turn_duration * (180.0 / M_PI);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Turn step %d: %.1f seconds, angle ≈ %.0f degrees", 
                current_step, turn_duration, angle_deg);
        }
        
        pub_->publish(msg);
    }
    
    void log_callback() {
        auto now = this->now();
        double elapsed = (now - start_time_).seconds();
        int current_step = step_ % 7;
        double step_elapsed = (now - step_start_time_).seconds();
        
        RCLCPP_INFO(this->get_logger(), 
                    "Time: %.1f s | Step: %d | Duration in step: %.1f/%.1f s",
                    elapsed, current_step, step_elapsed, step_durations_[current_step]);
    }
    

    
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;
    int step_;
    rclcpp::Time start_time_;
    rclcpp::Time step_start_time_;
    rclcpp::Time rotation_start_time_;
    double step_durations_[7];
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequenceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}