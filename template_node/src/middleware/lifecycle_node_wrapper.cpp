#include "template_node/middleware/lifecycle_node_wrapper.hpp"
#include "template_node/algorithms/example_algorithm.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace template_node::middleware {

LifecycleNodeWrapper::LifecycleNodeWrapper(const rclcpp::NodeOptions& options)
    : LifecycleNode("template_node", options) {
    RCLCPP_INFO(get_logger(), "Constructing LifecycleNodeWrapper");
    
    // Initialize core components
    algorithm_ = std::make_shared<algorithms::ExampleAlgorithm>();
    state_machine_ = std::make_unique<core::StateMachine>();
    parameter_handler_ = std::make_unique<ParameterHandler>(this);
    ros_interfaces_ = std::make_unique<RosInterfaces>(this);
    
    // Register state machine callback
    state_machine_->register_state_change_callback(
        std::bind(&LifecycleNodeWrapper::on_state_change, this, 
                  std::placeholders::_1, std::placeholders::_2));
}

CallbackReturn LifecycleNodeWrapper::on_configure(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Configuring node...");
    
    try {
        // Declare and load parameters
        parameter_handler_->declare_parameters();
        
        // Configure algorithm with parameters
        auto algo_params = parameter_handler_->get_algorithm_params();
        if (!algorithm_->configure(algo_params)) {
            RCLCPP_ERROR(get_logger(), "Failed to configure algorithm");
            return CallbackReturn::FAILURE;
        }
        
        // Setup ROS interfaces (publishers, subscribers, services, actions)
        setup_ros_interfaces();
        
        // Initialize operational state machine
        if (!state_machine_->transition(core::StateTransition::INITIALIZE)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize state machine");
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(get_logger(), "Configuration successful");
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }
}

CallbackReturn LifecycleNodeWrapper::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Activating node...");
    
    // Activate publishers
    status_pub_->on_activate();
    cmd_pub_->on_activate();
    
    // Start timers
    main_timer_ = create_wall_timer(
        100ms, std::bind(&LifecycleNodeWrapper::main_timer_callback, this));
    status_timer_ = create_wall_timer(
        1s, std::bind(&LifecycleNodeWrapper::status_timer_callback, this));
    
    RCLCPP_INFO(get_logger(), "Node activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNodeWrapper::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Deactivating node...");
    
    // Stop processing
    if (state_machine_->get_current_state() == core::OperationalState::PROCESSING) {
        state_machine_->transition(core::StateTransition::STOP_PROCESSING);
    }
    
    // Cancel timers
    if (main_timer_) {
        main_timer_->cancel();
        main_timer_.reset();
    }
    if (status_timer_) {
        status_timer_->cancel();
        status_timer_.reset();
    }
    
    // Deactivate publishers
    status_pub_->on_deactivate();
    cmd_pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "Node deactivated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNodeWrapper::on_cleanup(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Cleaning up node...");
    
    // Reset algorithm
    algorithm_->reset();
    
    // Clean up ROS interfaces
    cleanup_ros_interfaces();
    
    // Reset state machine
    state_machine_->transition(core::StateTransition::RESET);
    
    RCLCPP_INFO(get_logger(), "Cleanup complete");
    return CallbackReturn::SUCCESS;
}

CallbackReturn LifecycleNodeWrapper::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Shutting down node...");
    
    // Emergency stop if processing
    if (state_machine_->get_current_state() == core::OperationalState::PROCESSING) {
        state_machine_->transition(core::StateTransition::EMERGENCY_STOP_TRIGGERED);
    }
    
    // Clean up all resources
    cleanup_ros_interfaces();
    
    RCLCPP_INFO(get_logger(), "Shutdown complete");
    return CallbackReturn::SUCCESS;
}

void LifecycleNodeWrapper::setup_ros_interfaces() {
    // Publishers
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", 10);
    
    // Subscribers
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input_cloud", 10,
        std::bind(&LifecycleNodeWrapper::cloud_callback, this, std::placeholders::_1));
    
    command_sub_ = create_subscription<std_msgs::msg::String>(
        "~/command", 10,
        std::bind(&LifecycleNodeWrapper::command_callback, this, std::placeholders::_1));
    
    // Service
    compute_service_ = create_service<example_interfaces::srv::AddTwoInts>(
        "~/compute",
        std::bind(&LifecycleNodeWrapper::compute_service_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // Action server
    action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
        this,
        "~/fibonacci",
        std::bind(&LifecycleNodeWrapper::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LifecycleNodeWrapper::handle_cancel, this, std::placeholders::_1),
        std::bind(&LifecycleNodeWrapper::handle_accepted, this, std::placeholders::_1));
}

void LifecycleNodeWrapper::cleanup_ros_interfaces() {
    cloud_sub_.reset();
    command_sub_.reset();
    compute_service_.reset();
    action_server_.reset();
}

void LifecycleNodeWrapper::main_timer_callback() {
    if (state_machine_->get_current_state() != core::OperationalState::PROCESSING) {
        return;
    }
    
    // Create processing request
    core::ProcessingRequest request;
    request.timestamp = now().seconds();
    request.request_id = std::to_string(request.timestamp);
    request.input_data = {1.0, 2.0, 3.0, 4.0, 5.0};  // Example data
    
    // Process with algorithm
    auto result = algorithm_->process(request);
    
    // Publish results
    if (result.status == "success") {
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = result.output_data.empty() ? 0.0 : result.output_data[0];
        cmd_pub_->publish(cmd_msg);
    } else {
        RCLCPP_WARN(get_logger(), "Processing failed: %s", result.status.c_str());
        state_machine_->transition(core::StateTransition::ERROR_DETECTED);
    }
}

void LifecycleNodeWrapper::status_timer_callback() {
    auto msg = std_msgs::msg::String();
    msg.data = "State: " + state_machine_->get_state_name(state_machine_->get_current_state()) +
               ", Lifecycle: " + get_current_state().label();
    status_pub_->publish(msg);
}

void LifecycleNodeWrapper::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received point cloud with %d points", 
                 msg->width * msg->height);
    
    if (state_machine_->get_current_state() == core::OperationalState::IDLE) {
        state_machine_->transition(core::StateTransition::START_PROCESSING);
    }
}

void LifecycleNodeWrapper::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received command: %s", msg->data.c_str());
    
    if (msg->data == "start") {
        state_machine_->transition(core::StateTransition::START_PROCESSING);
    } else if (msg->data == "stop") {
        state_machine_->transition(core::StateTransition::STOP_PROCESSING);
    } else if (msg->data == "emergency_stop") {
        state_machine_->transition(core::StateTransition::EMERGENCY_STOP_TRIGGERED);
    }
}

void LifecycleNodeWrapper::compute_service_callback(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Service request: %ld + %ld = %ld",
                request->a, request->b, response->sum);
}

rclcpp_action::GoalResponse LifecycleNodeWrapper::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal) {
    
    RCLCPP_INFO(get_logger(), "Received goal request with order %d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LifecycleNodeWrapper::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
    
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LifecycleNodeWrapper::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
    
    // Execute action in a separate thread
    std::thread{std::bind(&LifecycleNodeWrapper::execute_action, this, goal_handle)}.detach();
}

void LifecycleNodeWrapper::execute_action(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle) {
    
    RCLCPP_INFO(get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<example_interfaces::action::Fibonacci::Feedback>();
    auto result = std::make_shared<example_interfaces::action::Fibonacci::Result>();
    
    // Fibonacci sequence
    feedback->sequence.push_back(0);
    feedback->sequence.push_back(1);
    
    for (int i = 2; i < goal->order; ++i) {
        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            return;
        }
        
        feedback->sequence.push_back(
            feedback->sequence[i-1] + feedback->sequence[i-2]);
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(100ms);
    }
    
    result->sequence = feedback->sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
}

void LifecycleNodeWrapper::on_state_change(core::OperationalState old_state, 
                                           core::OperationalState new_state) {
    RCLCPP_INFO(get_logger(), "State transition: %s -> %s",
                state_machine_->get_state_name(old_state).c_str(),
                state_machine_->get_state_name(new_state).c_str());
}

} // namespace template_node::middleware