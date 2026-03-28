#include <rclcpp/rclcpp.hpp>
#include <relbot_msgs/msg/relbot_motors.hpp>

class SequenceController : public rclcpp::Node {
public:
    SequenceController() : Node("sequence_controller"), step_(0) {
       
        pub_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
        

        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&SequenceController::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Sequence Controller Started - Full 5-step trajectory");
        print_step();
    }

private:
    void timer_callback() {
        auto msg = relbot_msgs::msg::RelbotMotors();
        
        switch(step_ % 5) {
            case 0: 
                msg.left_wheel_vel = 2.0;
                msg.right_wheel_vel = 2.0;
                RCLCPP_INFO(this->get_logger(), "STEP 1: FORWARD - left: 2.0, right: 2.0");
                break;
                
            case 1: 
                msg.left_wheel_vel = 1.0;
                msg.right_wheel_vel = -1.0;
                RCLCPP_INFO(this->get_logger(), "STEP 2: TURN RIGHT - left: 1.0, right: -1.0");
                break;
                
            case 2:
                msg.left_wheel_vel = -1.0;
                msg.right_wheel_vel = 1.0;
                RCLCPP_INFO(this->get_logger(), "STEP 3: TURN LEFT - left: -1.0, right: 1.0");
                break;
                
            case 3:
                msg.left_wheel_vel = -2.0;
                msg.right_wheel_vel = -2.0;
                RCLCPP_INFO(this->get_logger(), "STEP 4: BACKWARD - left: -2.0, right: -2.0");
                break;
                
            case 4: 
                msg.left_wheel_vel = 0.0;
                msg.right_wheel_vel = 0.0;
                RCLCPP_INFO(this->get_logger(), "STEP 5: STOP - left: 0.0, right: 0.0");
                break;
        }
        
        pub_->publish(msg);
        step_++;
    }
    
    void print_step() {
        RCLCPP_INFO(this->get_logger(), "Starting sequence: Forward → Right → Left → Backward → Stop (2 seconds each)");
    }
    
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}