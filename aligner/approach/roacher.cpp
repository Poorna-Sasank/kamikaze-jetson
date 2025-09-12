#include "roacher.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <Eigen/Core>
#include <cmath>

static const std::string kModeName = "InterceptMode";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

InterceptMode::InterceptMode(rclcpp::Node &node)
    : ModeBase(node, kModeName),
      _node(node),
      _yaw_pid(_param_yaw_p_gain, _param_max_yaw_speed),
      _lateral_pid(_param_lateral_p_gain, _param_max_lateral_speed),
      _height_pid(_param_height_p_gain, _param_max_height_speed)
{
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

    rclcpp::SubscriptionOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;

    _bbox_center_sub = _node.create_subscription<std_msgs::msg::Float32MultiArray>(
        "/car_bbox_center", rclcpp::QoS(1).best_effort(),
        std::bind(&InterceptMode::bboxCenterCallback, this, std::placeholders::_1), options);
    loadParameters();

    _yaw_pid.setGains(_param_yaw_p_gain, _param_max_yaw_speed);
    _lateral_pid.setGains(_param_lateral_p_gain, _param_max_lateral_speed);
    _height_pid.setGains(_param_height_p_gain, _param_max_height_speed);
}

void InterceptMode::loadParameters()
{
    _node.declare_parameter<float>("yaw_p_gain", _param_yaw_p_gain);
    _node.declare_parameter<float>("max_yaw_speed", _param_max_yaw_speed);
    _node.declare_parameter<float>("lateral_p_gain", _param_lateral_p_gain);
    _node.declare_parameter<float>("max_lateral_speed", _param_max_lateral_speed);
    _node.declare_parameter<float>("height_p_gain", _param_height_p_gain);
    _node.declare_parameter<float>("max_height_speed", _param_max_height_speed);
    _node.declare_parameter<float>("forward_speed", _param_forward_speed);
    _node.declare_parameter<float>("target_timeout", _param_target_timeout);
    _node.declare_parameter<float>("continue_timeout", _param_continue_timeout);
    _node.declare_parameter<float>("search_yaw_speed", _param_search_yaw_speed);
    _node.declare_parameter<float>("search_timeout", _param_search_timeout);
    _node.declare_parameter<float>("search_hover_speed", _param_search_hover_speed);
    _node.declare_parameter<float>("min_height", _param_min_height);
    _node.declare_parameter<float>("max_height", _param_max_height);
    _node.declare_parameter<float>("min_bbox_area", _param_min_bbox_area);
    _node.declare_parameter<float>("max_bbox_area", _param_max_bbox_area);
    _node.declare_parameter<float>("proximity_area_threshold", _param_proximity_area_threshold);
    _node.declare_parameter<float>("dive_vmin", _param_dive_vmin);
    _node.declare_parameter<float>("dive_k", _param_dive_k);
    _node.declare_parameter<float>("dive_sstart", _param_dive_sstart);
    _node.declare_parameter<float>("ref_forward_speed", _param_ref_forward_speed);

    _node.get_parameter("yaw_p_gain", _param_yaw_p_gain);
    _node.get_parameter("max_yaw_speed", _param_max_yaw_speed);
    _node.get_parameter("lateral_p_gain", _param_lateral_p_gain);
    _node.get_parameter("max_lateral_speed", _param_max_lateral_speed);
    _node.get_parameter("height_p_gain", _param_height_p_gain);
    _node.get_parameter("max_height_speed", _param_max_height_speed);
    _node.get_parameter("forward_speed", _param_forward_speed);
    _node.get_parameter("target_timeout", _param_target_timeout);
    _node.get_parameter("continue_timeout", _param_continue_timeout);
    _node.get_parameter("search_yaw_speed", _param_search_yaw_speed);
    _node.get_parameter("search_timeout", _param_search_timeout);
    _node.get_parameter("search_hover_speed", _param_search_hover_speed);
    _node.get_parameter("min_height", _param_min_height);
    _node.get_parameter("max_height", _param_max_height);
    _node.get_parameter("min_bbox_area", _param_min_bbox_area);
    _node.get_parameter("max_bbox_area", _param_max_bbox_area);
    _node.get_parameter("proximity_area_threshold", _param_proximity_area_threshold);
    _node.get_parameter("dive_vmin", _param_dive_vmin);
    _node.get_parameter("dive_k", _param_dive_k);
    _node.get_parameter("dive_sstart", _param_dive_sstart);
    _node.get_parameter("ref_forward_speed", _param_ref_forward_speed);

    _yaw_pid.setGains(_param_yaw_p_gain, _param_max_yaw_speed);
    _lateral_pid.setGains(_param_lateral_p_gain, _param_max_lateral_speed);
    _height_pid.setGains(_param_height_p_gain, _param_max_height_speed);
}

void InterceptMode::bboxCenterCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4)
    {
        _bbox_data = Eigen::Vector4f(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
        _last_bbox_time = _node.now();
        _last_target.bbox_data = _bbox_data;
        _last_target.timestamp = _last_bbox_time;
        _last_target.is_valid = true;
        
        float bbox_area = _bbox_data.z() * _bbox_data.w();
        
        // Enter proximity lock (direct dive mode) when area threshold is reached
        if (bbox_area >= _param_proximity_area_threshold && !_proximity_lock)
        {
            _proximity_lock = true;
            if (kEnableDebugOutput) {
                RCLCPP_INFO(_node.get_logger(), "PROXIMITY LOCK ENGAGED - Area: %.0f >= %.0f", 
                            bbox_area, _param_proximity_area_threshold);
            }
        }
        
        // Optional: Exit proximity lock if area becomes too small (target lost or moved away)
        // You can uncomment this if you want the drone to exit dive mode when target shrinks
        /*
        else if (bbox_area < _param_proximity_area_threshold * 0.8f && _proximity_lock)
        {
            _proximity_lock = false;
            if (kEnableDebugOutput) {
                RCLCPP_INFO(_node.get_logger(), "PROXIMITY LOCK DISENGAGED - Area: %.0f < %.0f", 
                            bbox_area, _param_proximity_area_threshold * 0.8f);
            }
        }
        */
    }
    else
    {
        _bbox_data = Eigen::Vector4f::Zero();
        _last_bbox_time = rclcpp::Time(0);
    }
}

void InterceptMode::onActivate()
{
    _lateral_pid.reset();
    _height_pid.reset();
    _yaw_pid.reset();
    _proximity_lock = false;
    _bbox_data = Eigen::Vector4f::Zero();
    _last_bbox_time = rclcpp::Time(0);
    _last_target.is_valid = false;
    _search_start_yaw = 0.0f;
    _search_direction_cw = true;
    _current_search_yaw = 0.0f;
    _prev_error_y = 0.0f; // Initialize previous Y error
    switchToState(State::Idle);
}

void InterceptMode::onDeactivate()
{
    _last_target.is_valid = false;
    _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, std::nullopt, 0.0f);
}

float InterceptMode::calculateVerticalSpeed(const Eigen::Vector4f &active_bbox)
{
    // Check if we're in proximity lock mode (direct dive)
    if (_proximity_lock) {
        // Direct dive - maximum downward speed without Y error correction
        float dive_speed = -_param_max_height_speed; // Maximum downward speed
        
        if (kEnableDebugOutput) {
            float current_height = _vehicle_local_position->positionNed().z();
            float bbox_area = active_bbox.z() * active_bbox.w();
            RCLCPP_INFO(_node.get_logger(), "DIRECT DIVE MODE - Area: %.0f, Dive Speed: %.2f m/s, Height: %.2f m", 
                        bbox_area, dive_speed, current_height);
        }
        
        return dive_speed;
    }
    
    // Normal descent logic based on target Y position in image (existing code)
    if (active_bbox.y() == 0.0f)
    {
        return 0.0f; // No vertical adjustment if no valid target
    }

    // Calculate error in Y direction (positive error means target is below center)
    float error_y = active_bbox.y() - _image_center_y;

    // Base vertical movement speed (m/s)
    float vertical_speed = 11.0f;

    // Scale speed based on how far off-center the target is
    float vertical_adjustment = vertical_speed * (error_y / (_image_center_y));

    // Limit maximum vertical adjustment to prevent rapid changes
    vertical_adjustment = std::clamp(vertical_adjustment, -8.0f, 8.0f);

    // Apply scaling factor similar to Python code (0.75)
    vertical_adjustment *= 4.f;

    if (kEnableDebugOutput)
    {
        float current_height = _vehicle_local_position->positionNed().z();
        RCLCPP_INFO(_node.get_logger(), "Y Error: %f, Vertical adjustment: %f m/s - Current Height: %f m",
                    error_y, vertical_adjustment, current_height);
    }

    _prev_error_y = error_y;
    return vertical_adjustment;
}

void InterceptMode::updateSetpoint(float dt_s)
{
    bool target_lost = checkTargetTimeout();

    // Handle Search state
    if (_state == State::Search)
    {
        // Check if target is found during search
        if (!target_lost)
        {
            switchToState(State::AlignAndApproach);
            _lateral_pid.reset();
            _height_pid.reset();
            _yaw_pid.reset();
            if (kEnableDebugOutput)
            {
                RCLCPP_INFO(_node.get_logger(), "Target found during search, switching to AlignAndApproach");
            }
        }
        else
        {
            // Check search timeout
            double search_elapsed = _node.now().seconds() - _search_start_time.seconds();
            if (search_elapsed > _param_search_timeout)
            {
                switchToState(State::Idle);
                _last_target.is_valid = false;
                _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, std::nullopt, 0.0f);
                if (kEnableDebugOutput)
                {
                    RCLCPP_INFO(_node.get_logger(), "Search timeout reached, switching to Idle");
                }
                return;
            }

            // Update search yaw
            float yaw_increment = _param_search_yaw_speed * dt_s;
            if (!_search_direction_cw)
                yaw_increment = -yaw_increment;
            _current_search_yaw += yaw_increment;

            // Normalize yaw to [-π, π]
            while (_current_search_yaw > M_PI)
                _current_search_yaw -= 2.0 * M_PI;
            while (_current_search_yaw < -M_PI)
                _current_search_yaw += 2.0 * M_PI;

            float target_yaw = _search_start_yaw + _current_search_yaw;
            float current_yaw = _vehicle_attitude->yaw();
            float yaw_error = target_yaw - current_yaw;

            // Normalize yaw error to [-π, π]
            while (yaw_error > M_PI)
                yaw_error -= 2.0 * M_PI;
            while (yaw_error < -M_PI)
                yaw_error += 2.0 * M_PI;

            float yaw_rate = _yaw_pid.update(yaw_error);

            // Maintain current height and move slowly forward while searching
            float current_height = _vehicle_local_position->positionNed().z();
            float target_height = std::clamp(current_height, _param_min_height, _param_max_height);
            float height_error = target_height - current_height;
            float vz = _height_pid.update(height_error);

            // Use body frame velocities for search
            Eigen::Vector3f body_velocity(_param_search_hover_speed, 0.0f, vz);
            Eigen::Vector3f ned_velocity;
            ned_velocity.x() = body_velocity.x() * std::cos(current_yaw) - body_velocity.y() * std::sin(current_yaw);
            ned_velocity.y() = body_velocity.x() * std::sin(current_yaw) + body_velocity.y() * std::cos(current_yaw);
            ned_velocity.z() = body_velocity.z();

            _trajectory_setpoint->update(ned_velocity, std::nullopt, std::nullopt, yaw_rate);

            if (kEnableDebugOutput)
            {
                RCLCPP_INFO(_node.get_logger(), "State: %s, Search Yaw: %f, Target Yaw: %f, Yaw Rate: %f, Elapsed: %f",
                            stateName(_state).c_str(), _current_search_yaw, target_yaw, yaw_rate, search_elapsed);
            }
            return;
        }
    }

    if (target_lost && (!_last_target.is_valid ||
                        (_node.now().seconds() - _last_target.timestamp.seconds() > _param_continue_timeout)))
    {
        switchToState(State::Idle);
        _last_target.is_valid = false;
        _lateral_pid.reset();
        _height_pid.reset();
        _yaw_pid.reset();
        _trajectory_setpoint->update(Eigen::Vector3f(0.0f, 0.0f, 0.0f), std::nullopt, std::nullopt, 0.0f);
        if (kEnableDebugOutput)
        {
            RCLCPP_INFO(_node.get_logger(), "Target lost, stopping drone.");
        }
        return;
    }

    if (_state == State::Idle && !target_lost)
    {
        switchToState(State::AlignAndApproach);
        _lateral_pid.reset();
        _height_pid.reset();
        _yaw_pid.reset();
    }

    if (target_lost && _last_target.is_valid &&
        (_node.now().seconds() - _last_target.timestamp.seconds() <= _param_continue_timeout))
    {
        switchToState(State::ContinueApproach);
    }
    else if (!target_lost && _state == State::ContinueApproach)
    {
        switchToState(State::AlignAndApproach);
    }
    else if (_state == State::ContinueApproach && target_lost &&
             (_node.now().seconds() - _last_target.timestamp.seconds() > _param_continue_timeout))
    {
        // Transition from ContinueApproach to Search when continue timeout is exceeded
        switchToState(State::Search);
        _search_start_yaw = _vehicle_attitude->yaw();
        _search_start_time = _node.now();
        _current_search_yaw = 0.0f;
        _search_direction_cw = true; // Start with clockwise search
        _yaw_pid.reset();
        if (kEnableDebugOutput)
        {
            RCLCPP_INFO(_node.get_logger(), "Continue timeout exceeded, switching to Search state");
        }
        return;
    }

    float lateral_speed = 0.0f;
    Eigen::Vector4f active_bbox = _bbox_data;
    if (_state == State::ContinueApproach && _last_target.is_valid)
    {
        active_bbox = _last_target.bbox_data;
    }

    if (active_bbox.x() != 0.0f && active_bbox.z() * active_bbox.w() >= _param_min_bbox_area)
    {
        float x_error = (active_bbox.x() - _image_center_x) / _image_center_x; // Normalize error to [-1, 1]
        lateral_speed = _lateral_pid.update(x_error);
    }

    // Use new vertical speed calculation based on target Y position or direct dive
    float vz = calculateVerticalSpeed(active_bbox);

    // Apply height limits to prevent going too high or too low
    float current_height = _vehicle_local_position->positionNed().z();
    if (current_height <= _param_min_height && vz < 0.0f)
    {
        vz = 0.0f; // Stop descending if at minimum height
    }
    if (current_height >= _param_max_height && vz > 0.0f)
    {
        vz = 0.0f; // Stop ascending if at maximum height
    }

    float forward_speed = (_state == State::Idle) ? 0.0f : _param_forward_speed;
    float yaw = _vehicle_attitude->yaw();

    Eigen::Vector3f body_velocity(forward_speed, lateral_speed, vz);
    Eigen::Vector3f ned_velocity;
    ned_velocity.x() = body_velocity.x() * std::cos(yaw) - body_velocity.y() * std::sin(yaw);
    ned_velocity.y() = body_velocity.x() * std::sin(yaw) + body_velocity.y() * std::cos(yaw);
    ned_velocity.z() = body_velocity.z();

    _trajectory_setpoint->update(ned_velocity, std::nullopt, std::nullopt, 0.0f);

    if (kEnableDebugOutput)
    {
        RCLCPP_INFO(_node.get_logger(), "State: %s, Velocity: [%f, %f, %f], Target Y: %f, Proximity Lock: %s",
                    stateName(_state).c_str(), ned_velocity.x(), ned_velocity.y(), ned_velocity.z(), 
                    active_bbox.y(), _proximity_lock ? "TRUE" : "FALSE");
    }
}

bool InterceptMode::checkTargetTimeout()
{
    if (_last_bbox_time.nanoseconds() == 0 ||
        _node.now().seconds() - _last_bbox_time.seconds() > _param_target_timeout)
    {
        _proximity_lock = false;
        return true;
    }
    return false;
}

std::string InterceptMode::stateName(State state)
{
    switch (state)
    {
    case State::Idle:
        return "Idle";
    case State::AlignAndApproach:
        return "AlignAndApproach";
    case State::ContinueApproach:
        return "ContinueApproach";
    case State::Search:
        return "Search";
    default:
        return "Unknown";
    }
}

void InterceptMode::switchToState(State state)
{
    if (_state != state && kEnableDebugOutput)
    {
        RCLCPP_INFO(_node.get_logger(), "Switching to state: %s", stateName(state).c_str());
    }
    _state = state;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<InterceptMode>>(kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}