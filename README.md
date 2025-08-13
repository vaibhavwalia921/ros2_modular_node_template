# ROS 2 Modular Node Template

A production-ready, modular ROS 2 node template implementing lifecycle management, state machines, and clean architecture principles. This template serves as a foundation for building robust, maintainable, and testable ROS 2 nodes following software engineering best practices.

## Features

- ✅ **Full ROS 2 LifecycleNode Implementation**: Complete lifecycle state management with proper resource handling
- ✅ **Internal Operational State Machine**: Six-state operational machine with thread-safe transitions
- ✅ **Clean Architecture**: Complete separation of algorithmic logic from ROS middleware
- ✅ **Dynamic Parameter System**: Runtime parameter updates with validation and callbacks
- ✅ **All ROS 2 Communication Patterns**: Publishers, subscribers, services, actions, and timers
- ✅ **Comprehensive Testing**: Unit tests for core logic and integration tests for ROS interfaces
- ✅ **Docker Support**: Multi-stage container builds for development and deployment
- ✅ **CI/CD Ready**: GitLab CI configuration foundation (to be completed)

## Architecture Overview

The template follows a clean architecture pattern with three main layers:

### 1. Core Layer (`core/`)
- **Algorithm Interface**: Pure virtual `IAlgorithm` interface for ROS-independent logic
- **State Machine**: Thread-safe operational state management
- **Data Types**: Generic data structures used across the application

### 2. Algorithm Layer (`algorithms/`)
- **Example Algorithm**: Reference implementation demonstrating the interface pattern
- **Extensible Design**: Easy to add new algorithms without touching middleware

### 3. Middleware Layer (`middleware/`)
- **Lifecycle Node Wrapper**: ROS 2 lifecycle management
- **ROS Interfaces**: All communication patterns (pub/sub, services, actions)
- **Parameter Handler**: Dynamic parameter management with validation

## Quick Start

### Prerequisites

- ROS 2 Humble (or later)
- Ubuntu 22.04 (recommended)
- C++17 compatible compiler
- Docker (optional, for containerized deployment)

### Native Build

```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone <repository-url>
cd ..

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run the node
ros2 run template_node template_node_main
```

### Building with Docker

```bash
cd docker
docker-compose up --build
```

### Running Tests

```bash
# Build tests
colcon test --packages-select template_node

# View test results
colcon test-result --verbose
```

### Launch Files

The template includes two launch files for different deployment scenarios:

#### Single Node Launch
```bash
# Launch a single template node with automatic lifecycle transitions
ros2 launch template_node template_node.launch.py

# Launch with custom parameters
ros2 launch template_node template_node.launch.py config_file:=/path/to/params.yaml
```

#### Composed Launch
```bash
# Launch with composable nodes for better performance
ros2 launch template_node template_node_composed.launch.py

# Launch with custom namespace
ros2 launch template_node template_node_composed.launch.py namespace:=my_robot
```

## Usage Examples

### Lifecycle Management

The node follows ROS 2 lifecycle patterns:

```bash
# Configure the node
ros2 lifecycle set /template_node configure

# Activate the node
ros2 lifecycle set /template_node activate

# Check node state
ros2 lifecycle get /template_node

# Deactivate when done
ros2 lifecycle set /template_node deactivate
```

### Dynamic Parameters

Parameters can be updated at runtime:

```bash
# List available parameters
ros2 param list /template_node

# Get current parameter values
ros2 param get /template_node algorithm.gain

# Set new parameter value
ros2 param set /template_node algorithm.gain 2.5
```

### ROS 2 Interfaces

The template provides multiple communication interfaces:

```bash
# Subscribe to status messages
ros2 topic echo /template_node/status

# Send commands
ros2 topic pub /template_node/command std_msgs/msg/String "{data: 'start'}"

# Call compute service
ros2 service call /template_node/compute example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

# Send action goal
ros2 action send_goal /template_node/fibonacci example_interfaces/action/Fibonacci "{order: 10}"
```

## Project Structure

```
ros2_modular_node_template/
├── template_node/               # Main ROS 2 package
│   ├── include/template_node/   # Header files
│   │   ├── core/               # Core logic (ROS-independent)
│   │   ├── algorithms/         # Algorithm implementations
│   │   └── middleware/         # ROS-specific code
│   ├── src/                    # Source files
│   ├── test/                   # Unit and integration tests
│   ├── launch/                 # Launch files (to be added)
│   ├── config/                 # Configuration files
│   ├── CMakeLists.txt          # Build configuration
│   └── package.xml             # Package metadata
├── docker/                     # Docker configuration
├── docs/                       # Documentation
│   ├── architecture.md         # Architecture details (to be completed)
│   ├── api_documentation.md    # API documentation (to be completed)
│   └── diagrams/               # Architecture diagrams
└── .gitlab-ci.yml             # CI/CD pipeline (to be completed)
```

## Extending the Template

### Adding New Algorithms

1. Create a new algorithm class inheriting from `template_node::core::IAlgorithm`
2. Implement the pure virtual methods: `process()`, `configure()`, `validate_input()`, `reset()`, `get_status()`
3. Update the `LifecycleNodeWrapper` to use your new algorithm
4. Add unit tests for your algorithm

### Adding New ROS Interfaces

1. Extend the `RosInterfaces` class with new communication methods
2. Update the setup and cleanup methods
3. Add corresponding tests in the integration test suite
4. Update parameter definitions if needed

## Testing

The template includes comprehensive tests:

- **Unit Tests**: Test core logic and algorithms independently
- **Integration Tests**: Test ROS interfaces and lifecycle transitions
- **Test Coverage**: Aim for >90% coverage of critical paths

Run specific test suites:
```bash
# Unit tests only
colcon test --packages-select template_node --ctest-args -R unit

# Integration tests only
colcon test --packages-select template_node --ctest-args -R integration
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

Apache License 2.0 - see [LICENSE](LICENSE) file for details.

## Documentation

- [Architecture Documentation](docs/architecture.md) (to be completed)
- [API Documentation](docs/api_documentation.md) (to be completed)
- [State Machine Diagram](docs/diagrams/state_machine.puml)