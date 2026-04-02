/**
 * @file closed_loop_controller_node.cpp
 * @brief Assignment 1.2.3 – Closed-loop control of the RELbot Simulator.
 *
 * Subscribes to the object position (pixel coordinates, published by the
 * position_node) derived from /output/moving_camera.  Computes a Twist
 * setpoint using a first-order controller and publishes it to the
 * relbot_adapter, which drives the RELbot Simulator.
 *
 * Controller law (see Assignment Manual eq. 1.2):
 *   x_dot_set = (1/tau) * (x_light - x_RELbot)
 *   x_set     = integral of x_dot_set  (Forward Euler)
 *
 * Because the object position reported by position_node is already expressed
 * in the moving-camera frame, the pixel offset from the image centre IS the
 * error (x_light - x_RELbot):
 *   - Horizontal offset (dx)  → θz error  → angular.z of Twist
 *   - Vertical offset   (dy)  → x  error  → linear.x  of Twist
 *
 * Maintainer: <your-email@student.utwente.nl>
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController()
  : Node("closed_loop_controller"),
    x_set_(0.0),
    theta_set_(0.0),
    has_object_(false)
  {
    // ----- Parameters --------------------------------------------------------
    tau_         = this->declare_parameter<double>("tau",          1.0);
    dt_          = this->declare_parameter<double>("dt",           0.05);  // s (20 Hz)
    img_width_   = this->declare_parameter<double>("image_width",  320.0);
    img_height_  = this->declare_parameter<double>("image_height", 240.0);
    max_lin_vel_ = this->declare_parameter<double>("max_linear_vel",  0.2);  // m/s
    max_ang_vel_ = this->declare_parameter<double>("max_angular_vel", 1.0);  // rad/s

    RCLCPP_INFO(this->get_logger(),
      "ClosedLoopController started. tau=%.2f s, dt=%.3f s", tau_, dt_);

    // ----- Subscriber: object position from position_node -------------------
    sub_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/object_position", 10,
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
        onObjectPosition(msg);
      });

    // ----- Publisher: Twist setpoint → relbot_adapter -----------------------
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/input/twist", 10);

    // ----- Control loop timer ------------------------------------------------
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt_ * 1000.0)),
      std::bind(&ClosedLoopController::controlLoop, this));
  }

private:
  // --------------------------------------------------------------------------
  void onObjectPosition(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    latest_pos_ = *msg;
    // position_node publishes (-1,-1, 0) when no object is detected
    has_object_ = (msg->z > 0.0);
  }

  // --------------------------------------------------------------------------
  void controlLoop()
  {
    auto twist = geometry_msgs::msg::Twist();

    if (!has_object_) {
      // No object visible: stop the robot
      x_set_     = 0.0;
      theta_set_ = 0.0;
      pub_twist_->publish(twist);   // publishes zeros
      return;
    }

    // Pixel offset from image centre = error in camera frame
    // Positive dx → object is to the RIGHT  → rotate CCW  (positive angular.z in ROS)
    // Positive dy → object is BELOW centre  → move forward (positive linear.x)
    const double cx = img_width_  / 2.0;
    const double cy = img_height_ / 2.0;

    const double error_theta = -(latest_pos_.x - cx) / cx;  // normalised [-1..1]
    const double error_x     = -(latest_pos_.y - cy) / cy;  // normalised [-1..1]

    // First-order controller: rate of change proportional to error
    const double theta_dot = (1.0 / tau_) * error_theta;
    const double x_dot     = (1.0 / tau_) * error_x;

    // Forward Euler integration → integrated setpoint
    theta_set_ += theta_dot * dt_;
    x_set_     += x_dot     * dt_;

    // The Twist velocity command IS the rate (not integrated position setpoint).
    // We send the velocity directly so the adapter's limits apply naturally.
    twist.angular.z = std::clamp(theta_dot, -max_ang_vel_, max_ang_vel_);
    twist.linear.x  = std::clamp(x_dot,    -max_lin_vel_, max_lin_vel_);

    pub_twist_->publish(twist);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "Object at (%.1f, %.1f) | error_theta=%.3f error_x=%.3f | "
      "cmd: linear.x=%.3f angular.z=%.3f",
      latest_pos_.x, latest_pos_.y,
      error_theta, error_x,
      twist.linear.x, twist.angular.z);
  }

  // --------------------------------------------------------------------------
  // Subscriptions / Publications
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_twist_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Latest detection result
  geometry_msgs::msg::Point latest_pos_;
  bool has_object_;

  // Integrated setpoint state (kept for reference / logging)
  double x_set_;
  double theta_set_;

  // Parameters
  double tau_;
  double dt_;
  double img_width_;
  double img_height_;
  double max_lin_vel_;
  double max_ang_vel_;
};

// ----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
