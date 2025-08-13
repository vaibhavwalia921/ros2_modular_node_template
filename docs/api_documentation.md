# API Documentation

## Core Layer API

### State Machine (`template_node::core::StateMachine`)

#### Overview
The `StateMachine` class provides thread-safe operational state management independent of ROS 2 lifecycle states.

#### Header
```cpp
#include "template_node/core/state_machine.hpp"
```

#### Enums

##### OperationalState
```cpp
enum class OperationalState {
    UNINITIALIZED,  // Initial state before configuration
    IDLE,          // Configured but not processing
    PROCESSING,    // Actively processing data
    ERROR,         // Error state requiring intervention
    MAINTENANCE,   // Maintenance mode
    EMERGENCY_STOP // Emergency stop state
};
```

##### StateTransition
```cpp
enum class StateTransition {
    INITIALIZE,              // Transition from UNINITIALIZED to IDLE
    START_PROCESSING,       // Transition from IDLE to PROCESSING
    STOP_PROCESSING,        // Transition from PROCESSING to IDLE
    ERROR_DETECTED,         // Transition to ERROR state
    ERROR_CLEARED,          // Transition from ERROR to IDLE
    ENTER_MAINTENANCE,      // Transition from IDLE to MAINTENANCE
    EXIT_MAINTENANCE,       // Transition from MAINTENANCE to IDLE
    EMERGENCY_STOP_TRIGGERED, // Emergency transition
    RESET                   // Reset to UNINITIALIZED
};
```

#### Public Methods

##### Constructor
```cpp
StateMachine();
```
Creates a new state machine instance in UNINITIALIZED state.

##### transition()
```cpp
bool transition(StateTransition trigger);
```
Attempts to transition to a new state.

**Parameters:**
- `trigger`: The transition to attempt

**Returns:**
- `true` if transition was successful, `false` if invalid

##### get_current_state()
```cpp
OperationalState get_current_state() const;
```
Gets the current operational state.

**Returns:**
- Current `OperationalState`

##### get_state_name()
```cpp
std::string get_state_name(OperationalState state) const;
```
Gets the human-readable name of a state.

**Parameters:**
- `state`: The state to get the name for

**Returns:**
- String representation of the state

##### register_state_change_callback()
```cpp
using StateChangeCallback = std::function<void(OperationalState, OperationalState)>;
void register_state_change_callback(StateChangeCallback callback);
```
Registers a callback to be called on state changes.

**Parameters:**
- `callback`: Function to call when state changes

##### can_transition()
```cpp
bool can_transition(StateTransition trigger) const;
```
Checks if a transition is valid from the current state.

**Parameters:**
- `trigger`: The transition to check

**Returns:**
- `true` if transition is valid, `false` otherwise

##### get_available_transitions()
```cpp
std::vector<StateTransition> get_available_transitions() const;
```
Gets all valid transitions from the current state.

**Returns:**
- Vector of valid `StateTransition` values

#### Example Usage
```cpp
#include "template_node/core/state_machine.hpp"

template_node::core::StateMachine sm;

// Register callback
sm.register_state_change_callback(
    [](template_node::core::OperationalState old_state, 
       template_node::core::OperationalState new_state) {
        std::cout << "State changed from " 
                  << static_cast<int>(old_state) << " to "
                  << static_cast<int>(new_state) << std::endl;
    });

// Transition to IDLE
if (sm.transition(template_node::core::StateTransition::INITIALIZE)) {
    std::cout << "Successfully initialized" << std::endl;
}

// Check current state
auto current_state = sm.get_current_state();
std::cout << "Current state: " << sm.get_state_name(current_state) << std::endl;
```

### Algorithm Interface (`template_node::core::IAlgorithm`)

#### Overview
The `IAlgorithm` class defines the interface for all algorithm implementations, ensuring they can be used interchangeably.

#### Header
```cpp
#include "template_node/core/algorithm_interface.hpp"
```

#### Data Structures

##### ProcessingRequest
```cpp
struct ProcessingRequest {
    std::vector<double> input_data;  // Input data for processing
    double timestamp;               // Timestamp of the request
    std::string request_id;         // Unique identifier for the request
};
```

##### ProcessingResult
```cpp
struct ProcessingResult {
    std::vector<double> output_data;  // Processed output data
    double processing_time;          // Time taken for processing
    std::string status;              // Status of processing ("success", "error", etc.)
    std::string request_id;          // Corresponding request ID
};
```

#### Public Methods

##### process()
```cpp
virtual ProcessingResult process(const ProcessingRequest& request) = 0;
```
Processes the input data and returns results.

**Parameters:**
- `request`: Input data and metadata

**Returns:**
- `ProcessingResult` containing output data and status

##### configure()
```cpp
virtual bool configure(const std::map<std::string, double>& params) = 0;
```
Configures the algorithm with parameters.

**Parameters:**
- `params`: Map of parameter names to values

**Returns:**
- `true` if configuration was successful, `false` otherwise

##### validate_input()
```cpp
virtual bool validate_input(const ProcessingRequest& request) const = 0;
```
Validates input data before processing.

**Parameters:**
- `request`: Input data to validate

**Returns:**
- `true` if input is valid, `false` otherwise

##### reset()
```cpp
virtual void reset() = 0;
```
Resets the algorithm to its initial state.

##### get_status()
```cpp
virtual std::string get_status() const = 0;
```
Gets the current status of the algorithm.

**Returns:**
- String describing the algorithm status

#### Example Implementation
```cpp
#include "template_node/core/algorithm_interface.hpp"

class MyAlgorithm : public template_node::core::IAlgorithm {
public:
    template_node::core::ProcessingResult process(
        const template_node::core::ProcessingRequest& request) override {
        // Process input data
        template_node::core::ProcessingResult result;
        result.request_id = request.request_id;
        
        // Example: simple gain application
        for (const auto& value : request.input_data) {
            result.output_data.push_back(value * gain_);
        }
        
        result.status = "success";
        return result;
    }
    
    bool configure(const std::map<std::string, double>& params) override {
        auto it = params.find("gain");
        if (it != params.end()) {
            gain_ = it->second;
        }
        return true;
    }
    
    bool validate_input(const template_node::core::ProcessingRequest& request) const override {
        return !request.input_data.empty();
    }
    
    void reset() override {
        gain_ = 1.0;
    }
    
    std::string get_status() const override {
        return "configured";
    }

private:
    double gain_ = 1.0;
};
```

### Data Types (`template_node::core`)

#### Overview
The core data types provide common structures used throughout the system.

#### Header
```cpp
#include "template_node/core/data_types.hpp"
```

#### Key Data Structures

##### Command
```cpp
struct Command {
    enum class Type {
        START,          // Start processing
        STOP,           // Stop processing
        PAUSE,          // Pause processing
        RESUME,         // Resume processing
        RESET,          // Reset to initial state
        EMERGENCY_STOP, // Emergency stop
        CUSTOM          // Custom command
    };
    
    Type type;                                    // Command type
    std::string id;                               // Command identifier
    std::map<std::string, double> parameters;    // Command parameters
    Timestamp timestamp;                          // When command was issued
};
```

##### StatusInfo
```cpp
struct StatusInfo {
    std::string node_name;                        // Name of the node
    std::string state;                            // Current operational state
    std::string lifecycle_state;                  // Current lifecycle state
    double cpu_usage;                             // CPU usage percentage
    double memory_usage;                          // Memory usage in MB
    Timestamp timestamp;                          // Status timestamp
    std::map<std::string, std::string> details;  // Additional details
};
```

##### ErrorInfo
```cpp
struct ErrorInfo {
    enum class Severity {
        INFO,       // Informational message
        WARNING,    // Warning condition
        ERROR,      // Error condition
        CRITICAL    // Critical error
    };
    
    std::string error_code;                       // Error code
    std::string message;                          // Error message
    Severity severity;                            // Error severity
    Timestamp timestamp;                          // When error occurred
    std::optional<std::string> suggested_action;  // Suggested recovery action
};
```

## Middleware Layer API

### Lifecycle Node Wrapper (`template_node::middleware::LifecycleNodeWrapper`)

#### Overview
The `LifecycleNodeWrapper` class implements ROS 2 lifecycle management and integrates all system components.

#### Header
```cpp
#include "template_node/middleware/lifecycle_node_wrapper.hpp"
```

#### Constructor
```cpp
explicit LifecycleNodeWrapper(const rclcpp::NodeOptions& options);
```
Creates a new lifecycle node wrapper.

**Parameters:**
- `options`: ROS 2 node options

#### Lifecycle Callbacks

These methods are called automatically by the ROS 2 lifecycle system:

##### on_configure()
```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_configure(const rclcpp_lifecycle::State& state) override;
```
Called when transitioning to CONFIGURE state.

##### on_activate()
```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_activate(const rclcpp_lifecycle::State& state) override;
```
Called when transitioning to ACTIVE state.

##### on_deactivate()
```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_deactivate(const rclcpp_lifecycle::State& state) override;
```
Called when transitioning to INACTIVE state.

##### on_cleanup()
```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_cleanup(const rclcpp_lifecycle::State& state) override;
```
Called when transitioning to UNCONFIGURED state.

##### on_shutdown()
```cpp
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
on_shutdown(const rclcpp_lifecycle::State& state) override;
```
Called when transitioning to SHUTDOWN state.

#### Example Usage
```cpp
#include "template_node/middleware/lifecycle_node_wrapper.hpp"

// Create node with default options
rclcpp::NodeOptions options;
auto node = std::make_shared<template_node::middleware::LifecycleNodeWrapper>(options);

// Add to executor
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node->get_node_base_interface());

// Run executor (lifecycle transitions happen automatically or via ROS 2 tools)
executor.spin();
```

### ROS Interfaces (`template_node::middleware::RosInterfaces`)

#### Overview
The `RosInterfaces` class manages all ROS 2 communication patterns.

#### Header
```cpp
#include "template_node/middleware/ros_interfaces.hpp"
```

#### Constructor
```cpp
explicit RosInterfaces(rclcpp_lifecycle::LifecycleNode* node);
```
Creates a new ROS interfaces manager.

**Parameters:**
- `node`: The lifecycle node to manage interfaces for

#### Setup Methods

##### setup_publishers()
```cpp
void setup_publishers();
```
Sets up all ROS publishers.

##### setup_subscribers()
```cpp
void setup_subscribers();
```
Sets up all ROS subscribers.

##### setup_services()
```cpp
void setup_services();
```
Sets up all ROS services.

##### setup_actions()
```cpp
void setup_actions();
```
Sets up all ROS action servers.

##### setup_timers()
```cpp
void setup_timers();
```
Sets up all ROS timers.

#### Publisher Methods

##### publish_status()
```cpp
void publish_status(const core::StatusInfo& status);
```
Publishes status information.

**Parameters:**
- `status`: Status information to publish

##### publish_command_velocity()
```cpp
void publish_command_velocity(double linear, double angular);
```
Publishes velocity commands.

**Parameters:**
- `linear`: Linear velocity
- `angular`: Angular velocity

##### publish_diagnostics()
```cpp
void publish_diagnostics(const std::vector<core::ErrorInfo>& errors);
```
Publishes diagnostic information.

**Parameters:**
- `errors`: List of errors to publish

#### Callback Registration

##### register_point_cloud_callback()
```cpp
using PointCloudCallback = std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)>;
void register_point_cloud_callback(PointCloudCallback callback);
```
Registers a callback for point cloud messages.

##### register_command_callback()
```cpp
using CommandCallback = std::function<void(const core::Command&)>;
void register_command_callback(CommandCallback callback);
```
Registers a callback for command messages.

##### register_odometry_callback()
```cpp
using OdometryCallback = std::function<void(const nav_msgs::msg::Odometry::SharedPtr)>;
void register_odometry_callback(OdometryCallback callback);
```
Registers a callback for odometry messages.

##### register_imu_callback()
```cpp
using ImuCallback = std::function<void(const sensor_msgs::msg::Imu::SharedPtr)>;
void register_imu_callback(ImuCallback callback);
```
Registers a callback for IMU messages.

#### Service Handler Registration

##### register_trigger_handler()
```cpp
using TriggerHandler = std::function<bool(std::string&)>;
void register_trigger_handler(const std::string& service_name, TriggerHandler handler);
```
Registers a handler for trigger services.

**Parameters:**
- `service_name`: Name of the service
- `handler`: Handler function

##### register_set_bool_handler()
```cpp
using SetBoolHandler = std::function<bool(bool, std::string&)>;
void register_set_bool_handler(const std::string& service_name, SetBoolHandler handler);
```
Registers a handler for SetBool services.

**Parameters:**
- `service_name`: Name of the service
- `handler`: Handler function

#### Timer Management

##### start_timers()
```cpp
void start_timers();
```
Starts all timers.

##### stop_timers()
```cpp
void stop_timers();
```
Stops all timers.

##### set_timer_frequency()
```cpp
void set_timer_frequency(const std::string& timer_name, double frequency_hz);
```
Sets the frequency of a specific timer.

**Parameters:**
- `timer_name`: Name of the timer
- `frequency_hz`: Frequency in Hz

#### Example Usage
```cpp
#include "template_node/middleware/ros_interfaces.hpp"

// Assuming 'node' is a LifecycleNode pointer
auto ros_interfaces = std::make_unique<template_node::middleware::RosInterfaces>(node);

// Set up all interfaces
ros_interfaces->setup_publishers();
ros_interfaces->setup_subscribers();
ros_interfaces->setup_services();
ros_interfaces->setup_actions();
ros_interfaces->setup_timers();

// Register callbacks
ros_interfaces->register_command_callback(
    [](const template_node::core::Command& cmd) {
        std::cout << "Received command: " << static_cast<int>(cmd.type) << std::endl;
    });

// Activate publishers
ros_interfaces->activate_publishers();

// Publish status
template_node::core::StatusInfo status;
status.node_name = "template_node";
status.state = "PROCESSING";
ros_interfaces->publish_status(status);
```

### Parameter Handler (`template_node::middleware::ParameterHandler`)

#### Overview
The `ParameterHandler` class manages dynamic parameters with validation and callbacks.

#### Header
```cpp
#include "template_node/middleware/parameter_handler.hpp"
```

#### Constructor
```cpp
explicit ParameterHandler(rclcpp_lifecycle::LifecycleNode* node);
```
Creates a new parameter handler.

**Parameters:**
- `node`: The lifecycle node to manage parameters for

#### Public Methods

##### declare_parameters()
```cpp
void declare_parameters();
```
Declares all parameters with their default values and descriptors.

##### get_parameter()
```cpp
template<typename T>
T get_parameter(const std::string& name) const;
```
Gets a parameter value.

**Parameters:**
- `name`: Parameter name

**Returns:**
- Parameter value of type T

##### get_algorithm_params()
```cpp
std::map<std::string, double> get_algorithm_params() const;
```
Gets all algorithm-related parameters.

**Returns:**
- Map of parameter names to values

##### register_parameter_callback()
```cpp
using ParameterCallback = std::function<void(const rclcpp::Parameter&)>;
void register_parameter_callback(const std::string& param_name, ParameterCallback callback);
```
Registers a callback for parameter changes.

**Parameters:**
- `param_name`: Name of the parameter
- `callback`: Function to call when parameter changes

#### Example Usage
```cpp
#include "template_node/middleware/parameter_handler.hpp"

// Assuming 'node' is a LifecycleNode pointer
auto param_handler = std::make_unique<template_node::middleware::ParameterHandler>(node);

// Declare parameters
param_handler->declare_parameters();

// Register callback for parameter changes
param_handler->register_parameter_callback(
    "algorithm.gain",
    [](const rclcpp::Parameter& param) {
        std::cout << "Gain changed to: " << param.as_double() << std::endl;
    });

// Get parameter values
double gain = param_handler->get_parameter<double>("algorithm.gain");
auto algo_params = param_handler->get_algorithm_params();
```

## Algorithm Layer API

### Example Algorithm (`template_node::algorithms::ExampleAlgorithm`)

#### Overview
The `ExampleAlgorithm` class provides a reference implementation of the `IAlgorithm` interface.

#### Header
```cpp
#include "template_node/algorithms/example_algorithm.hpp"
```

#### Constructor
```cpp
ExampleAlgorithm();
```
Creates a new example algorithm with default parameters.

#### Public Methods

All methods implement the `IAlgorithm` interface (see Core Layer API).

#### Example Usage
```cpp
#include "template_node/algorithms/example_algorithm.hpp"

auto algorithm = std::make_shared<template_node::algorithms::ExampleAlgorithm>();

// Configure algorithm
std::map<std::string, double> params = {
    {"gain", 2.0},
    {"offset", 1.0},
    {"filter_coefficient", 0.7}
};
algorithm->configure(params);

// Process data
template_node::core::ProcessingRequest request;
request.input_data = {1.0, 2.0, 3.0, 4.0, 5.0};
request.request_id = "test_001";

auto result = algorithm->process(request);

if (result.status == "success") {
    std::cout << "Processing successful. Output: ";
    for (const auto& value : result.output_data) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
}
```

## Error Handling

### Common Error Patterns

#### Invalid State Transitions
```cpp
template_node::core::StateMachine sm;

if (!sm.transition(template_node::core::StateTransition::START_PROCESSING)) {
    std::cerr << "Invalid state transition" << std::endl;
}
```

#### Algorithm Configuration Errors
```cpp
std::map<std::string, double> params = {
    {"filter_coefficient", 1.5}  // Invalid: > 1.0
};

if (!algorithm->configure(params)) {
    std::cerr << "Algorithm configuration failed" << std::endl;
}
```

#### Parameter Validation Errors
```cpp
rclcpp::Parameter invalid_param("algorithm.gain", -1.0);
if (!param_handler->validate_parameter(invalid_param)) {
    std::cerr << "Invalid parameter value" << std::endl;
}
```

## Best Practices

### State Machine Usage
- Always check if transitions are valid before attempting them
- Register callbacks to respond to state changes
- Use `can_transition()` to validate user-initiated transitions

### Algorithm Implementation
- Always validate input data in `validate_input()`
- Return meaningful status messages from `process()`
- Implement proper cleanup in `reset()`

### ROS Interface Usage
- Register callbacks before setting up interfaces
- Activate publishers only when needed
- Use appropriate QoS settings for your use case

### Parameter Management
- Declare all parameters with proper descriptors
- Validate parameter values before applying them
- Register callbacks for parameters that affect runtime behavior

### Threading Considerations
- The state machine is thread-safe for transitions
- Algorithm processing should be thread-safe if used from multiple threads
- ROS callbacks are called from the main executor thread