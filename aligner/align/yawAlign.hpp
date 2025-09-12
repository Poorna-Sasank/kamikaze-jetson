#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

class PIDController {
public:
    PIDController(float p, float i, float d, float max_output)
        : kp(p), ki(i), kd(d), max_output(max_output) {}

    float update(float error, float dt_s) {
        integral += error * dt_s;
        integral = std::clamp(integral, -max_output, max_output);
        float derivative = (error - prev_error) / dt_s;
        prev_error = error;
        return std::clamp(kp * error + ki * integral + kd * derivative, -max_output, max_output);
    }

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
    }

private:
    float kp, ki, kd, max_output;
    float integral = 0.0f;
    float prev_error = 0.0f;
};

class yawAlign : public px4_ros2::ModeBase {
public:
    explicit yawAlign(rclcpp::Node& node);

    void bboxCenterCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;
    enum class State { Idle, GotoPos, Align } _state;

private:
    void loadParameters();
    bool checkTargetTimeout();
    bool positionReached(const Eigen::Vector3f& target) const;
    void switchToState(State state);
    std::string stateName(State state);

    rclcpp::Node& _node;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _bbox_center_sub;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    // State _state = State::Align;
    Eigen::Vector4f _bbox_center = Eigen::Vector4f::Zero();
    rclcpp::Time _last_bbox_time;
    bool _target_lost_prev = true;

    PIDController _yaw_pid;
    PIDController _forward_pid;  // Added forward PID controller

    // Store last valid commands to continue when target is lost for 2 seconds
    Eigen::Vector3f _last_valid_velocity = Eigen::Vector3f::Zero();
    float _last_valid_yaw_speed = 0.0f;
    bool _has_valid_command = false;
    rclcpp::Time _target_lost_time;
    float _param_hold_command_duration = 2.0f;  // Hold previous command for 2 seconds

    // Yaw control parameters
    float _param_yaw_p_gain = 1.3f;
    float _param_yaw_i_gain = 0.001f;
    float _param_yaw_d_gain = 0.2f;
    float _param_max_yaw_speed = 20.0f;

    // Forward control parameters (area-based)
    float _param_forward_p_gain = 2.0f;
    float _param_forward_i_gain = 0.01f;
    float _param_forward_d_gain = 0.3f;
    float _param_max_forward_speed = 10.0f;

    // float _param_max_bbox_area = 25000.0f;  // Target bbox area
    // float _param_min_bbox_area = 40000.0f;  // Tolerance for bbox area
    float _param_max_bbox_area = 5000.0f;  // Target bbox area
    float _param_min_bbox_area = 4500.0f;  // Tolerance for bbox area


    float _param_target_timeout = 1.0f;

    const float _image_center_x = 960.0f;
    const float _image_center_y = 540.0f;
};