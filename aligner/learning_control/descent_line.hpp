#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <vector>

class YawAlign : public px4_ros2::ModeBase
{
public:
    explicit YawAlign(rclcpp::Node& node);

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

    // PID controller for position
    Eigen::Vector3f calculatePID(const Eigen::Vector3f& target_position, 
                               const Eigen::Vector3f& current_position, 
                               const Eigen::Vector3f& current_velocity,
                               float dt_s);

    enum class State {
        Idle,
        GotoPosition
    };

private:
    void loadParameters();
    float calculateHeadingRate(const Eigen::Vector3f& velocity_setpoint, float dt_s);
    bool positionReached(const Eigen::Vector3f& target) const;
    void switchToState(YawAlign::State state);
    std::string stateName(YawAlign::State state);

    // ROS2
    rclcpp::Node& _node;

    // PX4 ROS2
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    // Data
    State _state = State::GotoPosition;
    std::vector<Eigen::Vector3f> _waypoints;
    size_t _current_waypoint_idx = 0;

    // PID Parameters
    float _param_pos_p_gain = 1.0f;  // Aggressive P for quick response
    float _param_pos_i_gain = 0.02f; // Moderate I for steady-state correction
    float _param_pos_d_gain = 0.5f;  // High D for damping
    float _param_max_velocity = 5.0f; // Increased max velocity
    float _param_max_heading_rate = 2.0f; // Fast heading adjustment

    // PID state
    Eigen::Vector3f _position_error_integral = Eigen::Vector3f::Zero();
    Eigen::Vector3f _position_error_prev = Eigen::Vector3f::Zero();

    // Heading control
    float _target_heading = 0.0f;
    bool _heading_initialized = false;
};