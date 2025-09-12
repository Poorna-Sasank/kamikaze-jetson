#include "yawAlign.hpp"
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <Eigen/Core>

static const std::string kModeName = "YawAlignCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

yawAlign::yawAlign(rclcpp::Node &node)
    : ModeBase(node, kModeName),
      _node(node),
      _yaw_pid(_param_yaw_p_gain, _param_yaw_i_gain, _param_yaw_d_gain, _param_max_yaw_speed),
      _forward_pid(_param_forward_p_gain, _param_forward_i_gain, _param_forward_d_gain, _param_max_forward_speed)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    rclcpp::SubscriptionOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;

    _bbox_center_sub = _node.create_subscription<std_msgs::msg::Float32MultiArray>(
        "/car_bbox_center", rclcpp::QoS(1).best_effort(),
        std::bind(&yawAlign::bboxCenterCallback, this, std::placeholders::_1), options);
    loadParameters();
}

void yawAlign::loadParameters()
{
    // Yaw control parameters
    _node.declare_parameter<float>("yaw_p_gain", _param_yaw_p_gain);
    _node.declare_parameter<float>("yaw_i_gain", _param_yaw_i_gain);
    _node.declare_parameter<float>("yaw_d_gain", _param_yaw_d_gain);
    _node.declare_parameter<float>("max_yaw_speed", _param_max_yaw_speed);

    // Forward control parameters
    _node.declare_parameter<float>("forward_p_gain", _param_forward_p_gain);
    _node.declare_parameter<float>("forward_i_gain", _param_forward_i_gain);
    _node.declare_parameter<float>("forward_d_gain", _param_forward_d_gain);
    _node.declare_parameter<float>("max_forward_speed", _param_max_forward_speed);

    // Bbox area parameters
    _node.declare_parameter<float>("max_bbox_area", _param_max_bbox_area);
    _node.declare_parameter<float>("min_bbox_area", _param_min_bbox_area);

    _node.declare_parameter<float>("target_timeout", _param_target_timeout);
    _node.declare_parameter<float>("hold_command_duration", _param_hold_command_duration);

    // Get yaw parameters
    _node.get_parameter("yaw_p_gain", _param_yaw_p_gain);
    _node.get_parameter("yaw_i_gain", _param_yaw_i_gain);
    _node.get_parameter("yaw_d_gain", _param_yaw_d_gain);
    _node.get_parameter("max_yaw_speed", _param_max_yaw_speed);

    // Get forward parameters
    _node.get_parameter("forward_p_gain", _param_forward_p_gain);
    _node.get_parameter("forward_i_gain", _param_forward_i_gain);
    _node.get_parameter("forward_d_gain", _param_forward_d_gain);
    _node.get_parameter("max_forward_speed", _param_max_forward_speed);

    // Get bbox area parameters
    _node.get_parameter("max_bbox_area", _param_max_bbox_area);
    _node.get_parameter("min_bbox_area", _param_min_bbox_area);

    _node.get_parameter("target_timeout", _param_target_timeout);
    _node.get_parameter("hold_command_duration", _param_hold_command_duration);

    // Update PID controllers with new parameters
    _yaw_pid = PIDController(_param_yaw_p_gain, _param_yaw_i_gain, _param_yaw_d_gain, _param_max_yaw_speed);
    _forward_pid = PIDController(_param_forward_p_gain, _param_forward_i_gain, _param_forward_d_gain, _param_max_forward_speed);
}

void yawAlign::bboxCenterCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4)
    {
        _bbox_center = Eigen::Vector4f(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        _last_bbox_time = _node.now();
        RCLCPP_INFO(_node.get_logger(), "Received bbox center: x=%f, y=%f, z=%f, w=%f", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    }
    else
    {
        _bbox_center = Eigen::Vector4f::Zero();
        _last_bbox_time = rclcpp::Time(0);
        RCLCPP_INFO(_node.get_logger(), "Invalid bbox data, resetting");
    }
}

void yawAlign::onActivate()
{
    switchToState(State::GotoPos);
    _yaw_pid.reset();
    _forward_pid.reset();
    _has_valid_command = false;
    _last_valid_velocity = Eigen::Vector3f::Zero();
    _last_valid_yaw_speed = 0.0f;
    _target_lost_time = rclcpp::Time(0);
}

void yawAlign::onDeactivate() {}

void yawAlign::updateSetpoint(float dt_s)
{
    bool target_lost = checkTargetTimeout();
    if (target_lost && !_target_lost_prev)
    {
        RCLCPP_INFO(_node.get_logger(), "Target lost: State %s - Will hold last command for %.1f seconds", 
                   stateName(_state).c_str(), _param_hold_command_duration);
        _target_lost_time = _node.now();  // Record when target was lost
    }
    else if (!target_lost && _target_lost_prev)
    {
        RCLCPP_INFO(_node.get_logger(), "Target acquired");
        _yaw_pid.reset();
        _forward_pid.reset();
        _target_lost_time = rclcpp::Time(0);  // Reset the lost time
    }
    _target_lost_prev = target_lost;

    switch (_state)
    {
    case State::Idle:
        break;
    case State::GotoPos:
    {
        Eigen::Vector3f target_position(0.0f, -25.0f, -12.0f);
        _trajectory_setpoint->updatePosition(target_position);
        if (positionReached(target_position))
        {
            switchToState(State::Align);
        }
        break;
    }
    case State::Align:
    {
        if (target_lost)
        {
            // Check if we're still within the hold duration
            double time_since_lost = _node.now().seconds() - _target_lost_time.seconds();
            
            if (_has_valid_command && time_since_lost < _param_hold_command_duration)
            {
                // Continue with last valid command for the specified duration
                RCLCPP_INFO(_node.get_logger(), "Holding last command (%.1f/%.1f sec): vel=[%f,%f,%f], yaw_speed=%f", 
                           time_since_lost, _param_hold_command_duration,
                           _last_valid_velocity.x(), _last_valid_velocity.y(), _last_valid_velocity.z(), _last_valid_yaw_speed);
                
                _trajectory_setpoint->update(_last_valid_velocity, std::nullopt, std::nullopt, _last_valid_yaw_speed);
            }
            else
            {
                // Hold duration expired or no valid command, switch to hover
                if (_has_valid_command)
                {
                    RCLCPP_INFO(_node.get_logger(), "Hold duration expired, switching to hover mode");
                }
                else
                {
                    RCLCPP_INFO(_node.get_logger(), "No valid command to hold, hovering");
                }
                _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, _last_valid_yaw_speed, 0.0f);
            }
        }
        else
        {
            float bbox_area = _bbox_center.z() * _bbox_center.w();
            float x_error = (_image_center_x - _bbox_center.x()) / 1920.0f; // Normalized x-error
            
            if (bbox_area < _param_max_bbox_area)
            {
                // If the bbox area is too small, we need to move closer
                float forward_speed = _forward_pid.update(bbox_area, dt_s);
                // Calculate control outputs
                float yaw_speed = -_yaw_pid.update(x_error, dt_s);
                float yaw = _vehicle_attitude->yaw();

                Eigen::Vector3f body_velocity(forward_speed, 0.0f, 0.0f); // X is forward, Y is right, Z is down
                Eigen::Vector3f ned_velocity;
                ned_velocity.x() = body_velocity.x() * std::cos(yaw) - body_velocity.y() * std::sin(yaw);
                ned_velocity.y() = body_velocity.x() * std::sin(yaw) + body_velocity.y() * std::cos(yaw);
                ned_velocity.z() = body_velocity.z();
                
                // Store the current command as the last valid command
                _last_valid_velocity = ned_velocity;
                _last_valid_yaw_speed = yaw_speed;
                _has_valid_command = true;
                
                _trajectory_setpoint->update(ned_velocity, std::nullopt, std::nullopt, yaw_speed);
            }
            else if (bbox_area > _param_max_bbox_area)
            {
                // Store zero commands when we're too close
                _last_valid_velocity = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
                _last_valid_yaw_speed = 0.0f;
                _has_valid_command = true;
                
                _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, std::nullopt, 0.0f);
                _yaw_pid.reset();
                _forward_pid.reset();
            }
        }
        break;
    }
    }
}

bool yawAlign::checkTargetTimeout()
{
    if (_last_bbox_time.nanoseconds() == 0)
    {
        return true;
    }
    return _node.now().seconds() - _last_bbox_time.seconds() > _param_target_timeout;
}

std::string yawAlign::stateName(State state)
{
    switch (state)
    {
    case State::Idle:
        return "Idle";
    case State::GotoPos:
        return "GotoPos";
    case State::Align:
        return "Align";
    default:
        return "Unknown";
    }
}

bool yawAlign::positionReached(const Eigen::Vector3f &target) const
{
    auto position = _vehicle_local_position->positionNed();
    return (target - position).norm() < 0.2f;
}

void yawAlign::switchToState(State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<yawAlign>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}