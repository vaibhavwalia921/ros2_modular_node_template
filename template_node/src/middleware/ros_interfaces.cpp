#include "template_node/middleware/ros_interfaces.hpp"
#include <sstream>
#include <chrono>

namespace template_node::middleware {

using namespace std::chrono_literals;

RosInterfaces::RosInterfaces(rclcpp_lifecycle::LifecycleNode* node) 
    : node_(node), active_(false) {
    RCLCPP_INFO(node_->get_logger(), "Initializing ROS interfaces");
}


void RosInterfaces::setup_publishers() {
    RCLCPP_INFO(node_->get_logger(), "Setting up publishers");
    
    // Create lifecycle publishers
    status_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "~/status", rclcpp::QoS(10));
    
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::QoS(10));
    
    diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10));
    
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "~/pose", rclcpp::QoS(10));
}

void RosInterfaces::setup_subscribers() {
    RCLCPP_INFO(node_->get_logger(), "Setting up subscribers");
    
    // Point cloud subscriber
    point_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input_cloud", 10,
        std::bind(&RosInterfaces::internal_point_cloud_callback, this, std::placeholders::_1));
    
    // Command subscriber
    command_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "~/command", 10,
        std::bind(&RosInterfaces::internal_command_callback, this, std::placeholders::_1));
    
    // Odometry subscriber
    odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "~/odometry", 10,
        std::bind(&RosInterfaces::internal_odometry_callback, this, std::placeholders::_1));
    
    // IMU subscriber
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "~/imu", 10,
        std::bind(&RosInterfaces::internal_imu_callback, this, std::placeholders::_1));
}

void RosInterfaces::setup_services() {
    RCLCPP_INFO(node_->get_logger(), "Setting up services");
    
    // Example compute service
    compute_service_ = node_->create_service<example_interfaces::srv::AddTwoInts>(
        "~/compute",
        [this](const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
               std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
            response->sum = request->a + request->b;
            RCLCPP_INFO(node_->get_logger(), "Compute service: %ld + %ld = %ld",
                       request->a, request->b, response->sum);
        });
}

void RosInterfaces::setup_actions() {
    RCLCPP_INFO(node_->get_logger(), "Setting up action servers");
    
    // Fibonacci action server
    using Fibonacci = example_interfaces::action::Fibonacci;
    
    fibonacci_action_ = rclcpp_action::create_server<Fibonacci>(
        node_,
        "~/fibonacci",
        [](const rclcpp_action::GoalUUID& uuid,
           std::shared_ptr<const Fibonacci::Goal> goal) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> goal_handle) {
            // Execute in a separate thread
            std::thread{[this, goal_handle]() {
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<Fibonacci::Feedback>();
                auto result = std::make_shared<Fibonacci::Result>();
                
                feedback->sequence.push_back(0);
                feedback->sequence.push_back(1);
                
                for (int i = 2; i < goal->order; ++i) {
                    if (goal_handle->is_canceling()) {
                        goal_handle->canceled(result);
                        return;
                    }
                    
                    feedback->sequence.push_back(
                        feedback->sequence[i-1] + feedback->sequence[i-2]);
                    goal_handle->publish_feedback(feedback);
                    std::this_thread::sleep_for(100ms);
                }
                
                result->sequence = feedback->sequence;
                goal_handle->succeed(result);
            }}.detach();
        });
}

void RosInterfaces::setup_timers() {
    RCLCPP_INFO(node_->get_logger(), "Setting up timers");
    
    // Main processing timer (10 Hz default)
    timers_["main_timer"] = node_->create_wall_timer(
        100ms,
        [this]() {
            if (timer_callbacks_.count("main_timer")) {
                timer_callbacks_["main_timer"]();
            }
        });
    
    // Status publishing timer (1 Hz default)
    timers_["status_timer"] = node_->create_wall_timer(
        1s,
        [this]() {
            if (timer_callbacks_.count("status_timer")) {
                timer_callbacks_["status_timer"]();
            }
        });
}

void RosInterfaces::activate_publishers() {
    RCLCPP_INFO(node_->get_logger(), "Activating publishers");
    
    status_pub_->on_activate();
    cmd_vel_pub_->on_activate();
    diagnostics_pub_->on_activate();
    pose_pub_->on_activate();
    
    active_ = true;
}

void RosInterfaces::deactivate_publishers() {
    RCLCPP_INFO(node_->get_logger(), "Deactivating publishers");
    
    status_pub_->on_deactivate();
    cmd_vel_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    pose_pub_->on_deactivate();
    
    active_ = false;
}

void RosInterfaces::cleanup_all() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up all interfaces");
    
    // Stop timers
    stop_timers();
    
    // Reset subscribers
    point_cloud_sub_.reset();
    command_sub_.reset();
    odometry_sub_.reset();
    imu_sub_.reset();
    
    // Reset services
    trigger_services_.clear();
    set_bool_services_.clear();
    compute_service_.reset();
    
    // Reset action servers
    fibonacci_action_.reset();
    
    // Reset publishers
    status_pub_.reset();
    cmd_vel_pub_.reset();
    diagnostics_pub_.reset();
    pose_pub_.reset();
}

void RosInterfaces::publish_status(const core::StatusInfo& status) {
    if (!active_ || !status_pub_) return;
    
    auto msg = create_status_message(status);
    status_pub_->publish(msg);
}

void RosInterfaces::publish_command_velocity(double linear, double angular) {
    if (!active_ || !cmd_vel_pub_) return;
    
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    cmd_vel_pub_->publish(msg);
}

void RosInterfaces::publish_diagnostics(const std::vector<core::ErrorInfo>& errors) {
    if (!active_ || !diagnostics_pub_) return;
    
    auto msg = create_diagnostics_message(errors);
    diagnostics_pub_->publish(msg);
}

void RosInterfaces::register_point_cloud_callback(PointCloudCallback callback) {
    point_cloud_callback_ = callback;
}

void RosInterfaces::register_command_callback(CommandCallback callback) {
    command_callback_ = callback;
}

void RosInterfaces::register_odometry_callback(OdometryCallback callback) {
    odometry_callback_ = callback;
}

void RosInterfaces::register_imu_callback(ImuCallback callback) {
    imu_callback_ = callback;
}

void RosInterfaces::register_trigger_handler(const std::string& service_name, TriggerHandler handler) {
    trigger_handlers_[service_name] = handler;
    
    // Create the service
    trigger_services_[service_name] = node_->create_service<std_srvs::srv::Trigger>(
        "~/" + service_name,
        [this, service_name](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            handle_trigger_service(service_name, request, response);
        });
}

void RosInterfaces::register_set_bool_handler(const std::string& service_name, SetBoolHandler handler) {
    set_bool_handlers_[service_name] = handler;
    
    // Create the service
    set_bool_services_[service_name] = node_->create_service<std_srvs::srv::SetBool>(
        "~/" + service_name,
        [this, service_name](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            handle_set_bool_service(service_name, request, response);
        });
}

void RosInterfaces::start_timers() {
    for (auto& [name, timer] : timers_) {
        if (timer) {
            timer->reset();
        }
    }
}

void RosInterfaces::stop_timers() {
    for (auto& [name, timer] : timers_) {
        if (timer) {
            timer->cancel();
        }
    }
}

void RosInterfaces::set_timer_frequency(const std::string& timer_name, double frequency_hz) {
    if (timers_.count(timer_name) && timers_[timer_name]) {
        // Cancel existing timer
        timers_[timer_name]->cancel();
        
        // Create new timer with updated frequency
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_hz));
        timers_[timer_name] = node_->create_wall_timer(
            period,
            [this, timer_name]() {
                if (timer_callbacks_.count(timer_name)) {
                    timer_callbacks_[timer_name]();
                }
            });
    }
}

size_t RosInterfaces::get_subscriber_count(const std::string& topic) const {
    // This would need to check the specific subscriber
    // For demonstration, returning 0
    return 0;
}

size_t RosInterfaces::get_publisher_count(const std::string& topic) const {
    if (topic == "~/status" && status_pub_) {
        return status_pub_->get_subscription_count();
    } else if (topic == "~/cmd_vel" && cmd_vel_pub_) {
        return cmd_vel_pub_->get_subscription_count();
    }
    return 0;
}

// Internal callback implementations
void RosInterfaces::internal_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (point_cloud_callback_) {
        point_cloud_callback_(msg);
    }
}

void RosInterfaces::internal_command_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (command_callback_) {
        auto command = parse_command(msg->data);
        command_callback_(command);
    }
}

void RosInterfaces::internal_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (odometry_callback_) {
        odometry_callback_(msg);
    }
}

void RosInterfaces::internal_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (imu_callback_) {
        imu_callback_(msg);
    }
}

void RosInterfaces::handle_trigger_service(
    const std::string& service_name,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    if (trigger_handlers_.count(service_name)) {
        std::string message;
        response->success = trigger_handlers_[service_name](message);
        response->message = message;
    } else {
        response->success = false;
        response->message = "No handler registered for service: " + service_name;
    }
}

void RosInterfaces::handle_set_bool_service(
    const std::string& service_name,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (set_bool_handlers_.count(service_name)) {
        std::string message;
        response->success = set_bool_handlers_[service_name](request->data, message);
        response->message = message;
    } else {
        response->success = false;
        response->message = "No handler registered for service: " + service_name;
    }
}

// Utility method implementations
core::Command RosInterfaces::parse_command(const std::string& command_str) {
    core::Command cmd;
    cmd.timestamp = core::now();
    
    if (command_str == "start") {
        cmd.type = core::Command::Type::START;
    } else if (command_str == "stop") {
        cmd.type = core::Command::Type::STOP;
    } else if (command_str == "pause") {
        cmd.type = core::Command::Type::PAUSE;
    } else if (command_str == "resume") {
        cmd.type = core::Command::Type::RESUME;
    } else if (command_str == "reset") {
        cmd.type = core::Command::Type::RESET;
    } else if (command_str == "emergency_stop") {
        cmd.type = core::Command::Type::EMERGENCY_STOP;
    } else {
        cmd.type = core::Command::Type::CUSTOM;
        cmd.parameters["custom_command"] = 1.0;
    }
    
    cmd.id = command_str;
    return cmd;
}

std_msgs::msg::String RosInterfaces::create_status_message(const core::StatusInfo& status) {
    std_msgs::msg::String msg;
    std::stringstream ss;
    
    ss << "Node: " << status.node_name << ", ";
    ss << "State: " << status.state << ", ";
    ss << "Lifecycle: " << status.lifecycle_state << ", ";
    ss << "CPU: " << status.cpu_usage << "%, ";
    ss << "Memory: " << status.memory_usage << "MB";
    
    msg.data = ss.str();
    return msg;
}

diagnostic_msgs::msg::DiagnosticArray RosInterfaces::create_diagnostics_message(
    const std::vector<core::ErrorInfo>& errors) {
    
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = node_->now();
    
    for (const auto& error : errors) {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = node_->get_name();
        status.hardware_id = "template_node";
        
        // Map severity to diagnostic level
        switch (error.severity) {
            case core::ErrorInfo::Severity::INFO:
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                break;
            case core::ErrorInfo::Severity::WARNING:
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                break;
            case core::ErrorInfo::Severity::ERROR:
            case core::ErrorInfo::Severity::CRITICAL:
                status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                break;
        }
        
        status.message = error.message;
        
        // Add key-value pairs
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = "error_code";
        kv.value = error.error_code;
        status.values.push_back(kv);
        
        if (error.suggested_action) {
            kv.key = "suggested_action";
            kv.value = *error.suggested_action;
            status.values.push_back(kv);
        }
        
        msg.status.push_back(status);
    }
    
    return msg;
}

} // namespace template_node::middleware