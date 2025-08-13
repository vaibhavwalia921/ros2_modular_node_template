#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "template_node/core/algorithm_interface.hpp"
#include "template_node/core/state_machine.hpp"
#include "template_node/middleware/ros_interfaces.hpp"
#include "template_node/middleware/parameter_handler.hpp"

// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "example_interfaces/action/fibonacci.hpp"

namespace template_node::middleware {

class LifecycleNodeWrapper : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit LifecycleNodeWrapper(const rclcpp::NodeOptions& options);
    ~LifecycleNodeWrapper() override = default;
    
    // Lifecycle callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& state) override;
    
private:
    // Core components
    core::AlgorithmPtr algorithm_;
    std::unique_ptr<core::StateMachine> state_machine_;
    std::unique_ptr<ParameterHandler> parameter_handler_;
    std::unique_ptr<RosInterfaces> ros_interfaces_;
    
    // Publishers (for direct access in callbacks)
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    
    // Services
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr compute_service_;
    
    // Action server
    rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr action_server_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Callbacks
    void main_timer_callback();
    void status_timer_callback();
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
    void compute_service_callback(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);
    
    // Action callbacks
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    void execute_action(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    
    // State machine callback
    void on_state_change(core::OperationalState old_state, core::OperationalState new_state);
    
    // Helper methods
    void setup_ros_interfaces();
    void cleanup_ros_interfaces();
    bool validate_configuration();
};

} // namespace template_node::middleware