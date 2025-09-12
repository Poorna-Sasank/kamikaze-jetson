#include "loiter_control.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <Eigen/Core>
#include <cmath>

static const std::string kModeName = "CircleLoiter";
static const bool kEnableDebugOutput = true;

CircleLoiter::CircleLoiter(rclcpp::Node &node)
    : ModeBase(node, kModeName),
      _node(node)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    loadParameters();
}

void CircleLoiter::loadParameters()
{
    _node.declare_parameter<float>("radius", _param_radius);
    _node.declare_parameter<float>("omega", _param_omega);
    _node.declare_parameter<float>("poi_x", _point_of_interest.x());
    _node.declare_parameter<float>("poi_y", _point_of_interest.y());
    _node.declare_parameter<float>("poi_z", _point_of_interest.z());

    _node.get_parameter("radius", _param_radius);
    _node.get_parameter("omega", _param_omega);
    _node.get_parameter("poi_x", _point_of_interest.x());
    _node.get_parameter("poi_y", _point_of_interest.y());
    _node.get_parameter("poi_z", _point_of_interest.z());
}

void CircleLoiter::onActivate()
{
    _theta = 0.0f;
    
    RCLCPP_INFO(_node.get_logger(), 
                "CircleLoiter activated with center at (%f, %f, %f), looking at (%f, %f, %f)", 
                _center.x(), _center.y(), _center.z(),
                _point_of_interest.x(), _point_of_interest.y(), _point_of_interest.z());
    RCLCPP_INFO(_node.get_logger(), "Radius: %f meters, Angular speed: %f rad/s", 
                _param_radius, _param_omega);
}

void CircleLoiter::onDeactivate() 
{
    RCLCPP_INFO(_node.get_logger(), "CircleLoiter deactivated");
}

void CircleLoiter::updateSetpoint(float dt_s)
{
    _theta += _param_omega * dt_s;

    // Calculate new position on circle
    Eigen::Vector3f pos = _center;
    pos.x() += _param_radius * std::cos(_theta);
    pos.y() += _param_radius * std::sin(_theta);
    pos.z() = _center.z();

    // Calculate yaw to face the point of interest (NED frame)
    const Eigen::Vector2f vehicle_to_poi_xy = _point_of_interest.head(2) - pos.head(2);
    float yaw = std::atan2(vehicle_to_poi_xy(1), vehicle_to_poi_xy(0)); // Yaw in radians

    // Avoid unstable yaw near the point of interest
    if (vehicle_to_poi_xy.norm() < 0.1f) {
        yaw = NAN; // Let PX4 maintain current yaw
    }

    // Update trajectory setpoint with position and yaw
    px4_ros2::TrajectorySetpoint setpoint{};
    setpoint.position_ned_m_x = pos.x();
    setpoint.position_ned_m_y = pos.y();
    setpoint.position_ned_m_z = pos.z();
    setpoint.yaw_ned_rad = yaw;
    _trajectory_setpoint->update(setpoint);

    if (kEnableDebugOutput) {
        RCLCPP_DEBUG(_node.get_logger(), 
                    "Position: [%.2f, %.2f, %.2f], Theta: %.2f rad, Yaw: %.2f rad", 
                    pos.x(), pos.y(), pos.z(), _theta, yaw);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<CircleLoiter>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}