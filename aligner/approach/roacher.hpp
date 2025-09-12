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
    PIDController(float p, float max_output)
        : kp(p), max_output(max_output) {}

    float update(float error) { // Removed unused dt_s parameter
        return std::clamp(kp * error, -max_output, max_output);
    }

    void reset() {}

    void setGains(float p, float max_out) {
        kp = p;
        max_output = max_out;
    }

private:
    float kp, max_output;
};

class InterceptMode : public px4_ros2::ModeBase {
public:
    explicit InterceptMode(rclcpp::Node& node);

    void bboxCenterCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;
    enum class State { Idle, AlignAndApproach, ContinueApproach, Search };

private:
    struct LastTargetData {
        Eigen::Vector4f bbox_data = Eigen::Vector4f::Zero();
        rclcpp::Time timestamp;
        bool is_valid = false;
    };

    void loadParameters();
    bool checkTargetTimeout();
    void switchToState(State state);
    std::string stateName(State state);
    float calculateVerticalSpeed(const Eigen::Vector4f& active_bbox); // New method for vertical speed calculation

    rclcpp::Node& _node;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _bbox_center_sub;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    State _state = State::Idle;
    Eigen::Vector4f _bbox_data = Eigen::Vector4f::Zero();
    rclcpp::Time _last_bbox_time;
    bool _target_lost_prev = true;
    bool _proximity_lock = false;
    LastTargetData _last_target;

    // Search state variables
    float _search_start_yaw = 0.0f;
    rclcpp::Time _search_start_time;
    bool _search_direction_cw = true; // true for clockwise, false for counter-clockwise
    float _current_search_yaw = 0.0f;

    // New variable for tracking previous Y error
    float _prev_error_y = 0.0f;

    PIDController _yaw_pid;
    PIDController _lateral_pid;
    PIDController _height_pid;

    float _param_yaw_p_gain = 3.0f;
    float _param_max_yaw_speed = 4.0f;

    float _param_lateral_p_gain = 15.0f;
    float _param_max_lateral_speed = 30.0f;

    float _param_height_p_gain = 0.5f;
    float _param_max_height_speed = 11.0f;

    float _param_forward_speed = 23.0f;
    float _param_target_timeout = 1.0f;
    float _param_continue_timeout = 1.0f;

    // Search parameters
    float _param_search_yaw_speed = 0.5f; // rad/s - yaw speed during search
    float _param_search_timeout = 10.0f; // seconds - max time to search before giving up
    float _param_search_hover_speed = 2.0f; // forward speed while searching

    float _param_min_height = -1.0f;
    float _param_max_height = -2.0f;
    float _param_min_bbox_area = 1000.0f;
    float _param_max_bbox_area = 120000.0f;
    float _param_proximity_area_threshold = 900000.0f;

    float _param_dive_vmin = 0.5f;
    float _param_dive_k = 0.0005f;
    float _param_dive_sstart = 1000.0f;
    float _param_ref_forward_speed = 5.0f;

    const float _image_center_x = 960.0f;
    const float _image_center_y = 540.0f;
};