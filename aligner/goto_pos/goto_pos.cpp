#include "goto_pos.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <Eigen/Core>

static const std::string kModeName = "GlobalPosCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

GoToGlobalPos::GoToGlobalPos(rclcpp::Node &node) : ModeBase(node, kModeName), _node(node)
{
    _goto_setpoint = std::make_shared<px4_ros2::GotoGlobalSetpointType>(*this);
    _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
}

void GoToGlobalPos::onActivate()
{
    switchToState(State::startPose);
    start_global_pose = _vehicle_global_position->position();
}

void GoToGlobalPos::onDeactivate() {}

void GoToGlobalPos::updateSetpoint(float /*dt_s*/)
{
    switch (_state)
    {
    case State::Idle:
        break;
    case State::startPose:
    {
        Eigen::Vector3d target_pose = start_global_pose;
        target_pose.z() = start_global_pose.z() + 20.0;
        _goto_setpoint->update(target_pose);
        if (positionReached(target_pose))
        {
            switchToState(State::strikePose);
        }
        break;
    }
    case State::strikePose:
    {
        Eigen::Vector3d strike_pose(37.41286512778175, -121.99825137672016, 38.617105851881206);
        _goto_setpoint->update(strike_pose);
        if (positionReached(strike_pose))
        {
            switchToState(State::hover);
        }
        break;
    }
    case State::hover:
    {
        _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, std::nullopt, 0.0f);
        break;
    }
    }
}

std::string GoToGlobalPos::stateName(State state)
{
    switch (state)
    {
    case State::Idle:
        return "Idle";
    case State::startPose:
        return "startPose";
    case State::strikePose:
        return "strikePose";
    case State::hover:
        return "hover";
    default:
        return "Unknown";
    }
}

bool GoToGlobalPos::positionReached(const Eigen::Vector3d &target) const
{
    auto position = _vehicle_global_position->position();
    return (target - position).norm() < 0.2;
}

void GoToGlobalPos::switchToState(State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<GoToGlobalPos>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}