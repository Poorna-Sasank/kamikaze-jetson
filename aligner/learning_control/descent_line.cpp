#include "descent_line.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <algorithm>

static const std::string kModeName = "PositionControlCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

YawAlign::YawAlign(rclcpp::Node &node)
    : ModeBase(node, kModeName), _node(node)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    loadParameters();
}

void YawAlign::loadParameters()
{
    // Declare PID parameters
    _node.declare_parameter<float>("pos_p_gain", _param_pos_p_gain);
    _node.declare_parameter<float>("pos_i_gain", _param_pos_i_gain);
    _node.declare_parameter<float>("pos_d_gain", _param_pos_d_gain);
    _node.declare_parameter<float>("max_velocity", _param_max_velocity);
    _node.declare_parameter<float>("max_heading_rate", _param_max_heading_rate);

    // Declare waypoint parameters as vectors for a square trajectory
    _node.declare_parameter<std::vector<double>>("waypoints_x", {-3.0, -6.0, -9.0});
    _node.declare_parameter<std::vector<double>>("waypoints_y", {0.0, 0.0, 0.0});
    _node.declare_parameter<std::vector<double>>("waypoints_z", {-7.0, -5.0, -2.0});

    // Load parameters
    _node.get_parameter("pos_p_gain", _param_pos_p_gain);
    _node.get_parameter("pos_i_gain", _param_pos_i_gain);
    _node.get_parameter("pos_d_gain", _param_pos_d_gain);
    _node.get_parameter("max_velocity", _param_max_velocity);
    _node.get_parameter("max_heading_rate", _param_max_heading_rate);

    // Load waypoints
    std::vector<double> waypoints_x, waypoints_y, waypoints_z;
    _node.get_parameter("waypoints_x", waypoints_x);
    _node.get_parameter("waypoints_y", waypoints_y);
    _node.get_parameter("waypoints_z", waypoints_z);

    // Ensure waypoint lists are the same length
    size_t num_waypoints = std::min({waypoints_x.size(), waypoints_y.size(), waypoints_z.size()});
    _waypoints.clear();
    for (size_t i = 0; i < num_waypoints; ++i)
    {
        _waypoints.emplace_back(static_cast<float>(waypoints_x[i]),
                                static_cast<float>(waypoints_y[i]),
                                static_cast<float>(waypoints_z[i]));
    }

    RCLCPP_INFO(_node.get_logger(), "Loaded %zu waypoints", _waypoints.size());
    RCLCPP_INFO(_node.get_logger(), "PID Gains - P: %f, I: %f, D: %f",
                _param_pos_p_gain, _param_pos_i_gain, _param_pos_d_gain);
    RCLCPP_INFO(_node.get_logger(), "Max velocity: %f, Max heading rate: %f",
                _param_max_velocity, _param_max_heading_rate);
}

void YawAlign::onActivate()
{
    switchToState(State::GotoPosition);
    _current_waypoint_idx = 0;
    _position_error_integral = Eigen::Vector3f::Zero();
    _position_error_prev = Eigen::Vector3f::Zero();
    _heading_initialized = false;
}

void YawAlign::onDeactivate()
{
    _position_error_integral = Eigen::Vector3f::Zero();
    _position_error_prev = Eigen::Vector3f::Zero();
    _heading_initialized = false;
    _current_waypoint_idx = 0;
}

void YawAlign::updateSetpoint(float dt_s)
{
    if (_current_waypoint_idx >= _waypoints.size())
    {
        switchToState(State::Idle);
    }

    switch (_state)
    {
    case State::Idle:
        _trajectory_setpoint->update(
            Eigen::Vector3f(0.0f, 0.0f, 0.0f), // Zero velocity
            std::nullopt,
            std::nullopt,
            0.0f // Zero yaw rate
        );
        break;

    case State::GotoPosition:
    {
        Eigen::Vector3f current_position = _vehicle_local_position->positionNed();
        Eigen::Vector3f current_velocity = _vehicle_local_position->velocityNed();
        Eigen::Vector3f velocity_setpoint = calculatePID(
            _waypoints[_current_waypoint_idx], current_position, current_velocity, dt_s);

        float heading_rate = calculateHeadingRate(velocity_setpoint, dt_s);

        _trajectory_setpoint->update(
            velocity_setpoint,
            std::nullopt,
            std::nullopt,
            heading_rate);

        if (positionReached(_waypoints[_current_waypoint_idx]))
        {
            RCLCPP_INFO(_node.get_logger(), "Waypoint %zu reached!", _current_waypoint_idx);
            _current_waypoint_idx++;
            if (_current_waypoint_idx >= _waypoints.size())
            {
                switchToState(State::Idle);
                completed(px4_ros2::Result::Success);
            }
        }
    }
    break;
    }
}

Eigen::Vector3f YawAlign::calculatePID(const Eigen::Vector3f &target_position,
                                       const Eigen::Vector3f &current_position,
                                       const Eigen::Vector3f &current_velocity,
                                       float dt_s)
{
    // Calculate position error
    Eigen::Vector3f error = target_position - current_position;

    // P term
    Eigen::Vector3f p_term = error * _param_pos_p_gain;

    // I term with anti-windup
    _position_error_integral += error * dt_s;
    _position_error_integral = _position_error_integral.cwiseMax(-_param_max_velocity / _param_pos_i_gain).cwiseMin(_param_max_velocity / _param_pos_i_gain);
    Eigen::Vector3f i_term = _position_error_integral * _param_pos_i_gain;

    // D term using velocity feedback
    Eigen::Vector3f d_term = -current_velocity * _param_pos_d_gain;

    // add
    Eigen::Vector3f velocity_command = p_term + i_term + d_term;

    // Clamp velocity
    velocity_command = velocity_command.cwiseMax(-_param_max_velocity).cwiseMin(_param_max_velocity);

    if (kEnableDebugOutput)
    {
        RCLCPP_DEBUG(_node.get_logger(),
                     "Error: [%.2f, %.2f, %.2f], Velocity: [%.2f, %.2f, %.2f]",
                     error.x(), error.y(), error.z(),
                     velocity_command.x(), velocity_command.y(), velocity_command.z());
    }

    return velocity_command;
}

float YawAlign::calculateHeadingRate(const Eigen::Vector3f &velocity_setpoint, float dt_s)
{
    Eigen::Vector2f horizontal_velocity = velocity_setpoint.head(2);

    if (horizontal_velocity.norm() < 0.1f)
    {
        return 0.0f;
    }

    float desired_heading = atan2f(horizontal_velocity.y(), horizontal_velocity.x());

    if (!_heading_initialized)
    {
        _target_heading = _vehicle_local_position->heading();
        _heading_initialized = true;
    }

    float heading_error = px4_ros2::wrapPi(desired_heading - _target_heading);
    float heading_rate = heading_error * 2.0f; // Proportional gain
    heading_rate = std::clamp(heading_rate, -_param_max_heading_rate, _param_max_heading_rate);

    _target_heading += heading_rate * dt_s;
    _target_heading = px4_ros2::wrapPi(_target_heading);

    if (kEnableDebugOutput)
    {
        RCLCPP_DEBUG(_node.get_logger(),
                     "Desired heading: %.2f, Current heading: %.2f, Heading rate: %.2f",
                     desired_heading, _vehicle_local_position->heading(), heading_rate);
    }

    return heading_rate;
}

bool YawAlign::positionReached(const Eigen::Vector3f &target) const
{
    static constexpr float kPositionErrorThreshold = 0.3f; // Tighter threshold
    static constexpr float kVelocityErrorThreshold = 0.3f; // Tighter threshold

    Eigen::Vector3f current_position = _vehicle_local_position->positionNed();
    Eigen::Vector3f position_error = target - current_position;
    Eigen::Vector3f current_velocity = _vehicle_local_position->velocityNed();

    return position_error.norm() < kPositionErrorThreshold &&
           current_velocity.norm() < kVelocityErrorThreshold;
}

std::string YawAlign::stateName(YawAlign::State state)
{
    switch (state)
    {
    case YawAlign::State::Idle:
        return "Idle";
    case YawAlign::State::GotoPosition:
        return "GotoPosition";
    default:
        return "Unknown";
    }
}

void YawAlign::switchToState(YawAlign::State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<YawAlign>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}