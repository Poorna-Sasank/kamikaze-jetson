#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <cmath>

class CircleLoiter : public px4_ros2::ModeBase {
public:
    explicit CircleLoiter(rclcpp::Node& node);

    void onActivate() override;
    void onDeactivate() override;
    void updateSetpoint(float dt_s) override;

private:
    void loadParameters();

    rclcpp::Node& _node;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

    float _theta = 0.0f;
    Eigen::Vector3f _center = Eigen::Vector3f(0.0f, 10.5f, -10.0f); // Fixed center
    Eigen::Vector3f _point_of_interest = Eigen::Vector3f(0.0f, 5.25f, 0.0f); // Point to look at

    // Parameters
    float _param_radius = 5.0f;
    float _param_omega = 0.5f; // Angular speed in rad/s
};