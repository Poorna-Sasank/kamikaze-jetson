#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/geodesic.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <memory>

class GoToGlobalPos : public px4_ros2::ModeBase
{
public:
    enum class State
    {
        Idle,
        startPose,
        strikePose,
        hover
    };

    explicit GoToGlobalPos(rclcpp::Node &node);

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float /*dt_s*/) override;

private:

    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::GotoGlobalSetpointType> _goto_setpoint;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    State _state = State::startPose;
    Eigen::Vector3d start_global_pose;

    bool positionReached(const Eigen::Vector3d &target) const;
    void switchToState(State state);
    std::string stateName(State state);
};

// void GoToGlobalPos::updateSetpoint(float dt_s) {
//     static float elapsed_time = 0.0f;
//     elapsed_time += dt_s;

//     // auto pos = _vehicle_global_position->position();
//     // Eigen::Vector3f glob_pos = pos;
//     // if (elapsed_time >= 1.0f) {
//     //     RCLCPP_INFO()
//     // }

//     // Fixed: Use double instead of float for Eigen::Vector3 to match expected type
//     Eigen::Vector3d target_pos(13.05861, 80.25844, 8.0);
//     _goto_setpoint->update(target_pos);
// }