#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Message types
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

// Service types
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// Action types
#include "example_interfaces/action/fibonacci.hpp"

// Custom type conversions
#include "template_node/core/data_types.hpp"

#include <memory>
#include <map>
#include <functional>

namespace template_node::middleware {

// Forward declaration
class LifecycleNodeWrapper;

/**
 * @brief Manages all ROS 2 communication interfaces
 * 
 * This class encapsulates all ROS 2 communication patterns including
 * publishers, subscribers, services, and actions. It provides a clean
 * interface for setting up and managing these communications.
 */
class RosInterfaces {
public:
    explicit RosInterfaces(rclcpp_lifecycle::LifecycleNode* node);
    ~RosInterfaces() = default;
    
    // Setup and cleanup
    void setup_publishers();
    void setup_subscribers();
    void setup_services();
    void setup_actions();
    void setup_timers();
    
    void activate_publishers();
    void deactivate_publishers();
    void cleanup_all();
    
    // Publisher methods
    void publish_status(const core::StatusInfo& status);
    void publish_command_velocity(double linear, double angular);
    void publish_diagnostics(const std::vector<core::ErrorInfo>& errors);
    
    // Subscriber callbacks
    using PointCloudCallback = std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)>;
    using CommandCallback = std::function<void(const core::Command&)>;
    using OdometryCallback = std::function<void(const nav_msgs::msg::Odometry::SharedPtr)>;
    using ImuCallback = std::function<void(const sensor_msgs::msg::Imu::SharedPtr)>;
    
    void register_point_cloud_callback(PointCloudCallback callback);
    void register_command_callback(CommandCallback callback);
    void register_odometry_callback(OdometryCallback callback);
    void register_imu_callback(ImuCallback callback);
    
    // Service handlers
    using TriggerHandler = std::function<bool(std::string&)>;
    using SetBoolHandler = std::function<bool(bool, std::string&)>;
    
    void register_trigger_handler(const std::string& service_name, TriggerHandler handler);
    void register_set_bool_handler(const std::string& service_name, SetBoolHandler handler);
    
    // Timer management
    void start_timers();
    void stop_timers();
    void set_timer_frequency(const std::string& timer_name, double frequency_hz);
    
    // Utility methods
    bool is_active() const { return active_; }
    size_t get_subscriber_count(const std::string& topic) const;
    size_t get_publisher_count(const std::string& topic) const;
    
private:
    rclcpp_lifecycle::LifecycleNode* node_;
    bool active_;
    
    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // Services
    std::map<std::string, rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> trigger_services_;
    std::map<std::string, rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> set_bool_services_;
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr compute_service_;
    
    // Action servers
    rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr fibonacci_action_;
    
    // Timers
    std::map<std::string, rclcpp::TimerBase::SharedPtr> timers_;
    std::map<std::string, std::function<void()>> timer_callbacks_;
    
    // Callbacks storage
    PointCloudCallback point_cloud_callback_;
    CommandCallback command_callback_;
    OdometryCallback odometry_callback_;
    ImuCallback imu_callback_;
    
    std::map<std::string, TriggerHandler> trigger_handlers_;
    std::map<std::string, SetBoolHandler> set_bool_handlers_;
    
    // Internal callback methods
    void internal_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void internal_command_callback(const std_msgs::msg::String::SharedPtr msg);
    void internal_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void internal_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Service callback methods
    void handle_trigger_service(
        const std::string& service_name,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void handle_set_bool_service(
        const std::string& service_name,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    
    // Utility methods
    core::Command parse_command(const std::string& command_str);
    std_msgs::msg::String create_status_message(const core::StatusInfo& status);
    diagnostic_msgs::msg::DiagnosticArray create_diagnostics_message(const std::vector<core::ErrorInfo>& errors);
};

} // namespace template_node::middleware