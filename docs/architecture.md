# Architecture Documentation

## Overview

The ROS 2 Modular Node Template follows a clean architecture pattern that strictly separates concerns between core business logic, algorithm implementations, and ROS 2 middleware. This design enables independent testing, reuse of components, and maintainability.

## Architectural Layers

### 1. Core Layer (`template_node/core/`)

The core layer contains fundamental, ROS-independent logic that forms the foundation of the system.

#### Components:

- **`algorithm_interface.hpp`**: Pure virtual interface defining the contract for all algorithms
- **`state_machine.hpp`**: Thread-safe operational state machine implementation
- **`data_types.hpp`**: Common data structures and type definitions used across the system

#### Design Principles:

- **ROS Independence**: No ROS dependencies, enabling pure unit testing
- **Interface Segregation**: Clear interfaces between components
- **Type Safety**: Strong typing throughout the core layer

### 2. Algorithm Layer (`template_node/algorithms/`)

The algorithm layer contains implementations of the core interfaces, providing specific processing capabilities.

#### Components:

- **`example_algorithm.hpp`**: Reference implementation demonstrating the interface pattern

#### Design Principles:

- **Interface Compliance**: All algorithms implement the `IAlgorithm` interface
- **Pluggability**: Algorithms can be swapped without changing middleware code
- **Testability**: Pure business logic that can be tested independently

### 3. Middleware Layer (`template_node/middleware/`)

The middleware layer contains all ROS 2-specific code, acting as a bridge between the core/algorithm layers and the ROS 2 ecosystem.

#### Components:

- **`lifecycle_node_wrapper.hpp`**: ROS 2 LifecycleNode implementation
- **`ros_interfaces.hpp`**: All ROS 2 communication patterns
- **`parameter_handler.hpp`**: Dynamic parameter management

#### Design Principles:

- **ROS Integration**: Leverages ROS 2 features while maintaining clean boundaries
- **Lifecycle Management**: Proper resource management through ROS 2 lifecycle states
- **Communication Abstraction**: Unified interface for all ROS 2 communication patterns

## Data Flow

### High-Level Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   ROS 2 Topics  │◄──►│   Middleware    │◄──►│   Algorithm     │
│   Services      │    │   Layer         │    │   Layer         │
│   Actions       │    │                 │    │                 │
│   Parameters    │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                               ▲
                               │
                               ▼
                        ┌─────────────────┐
                        │    Core Layer   │
                        │                 │
                        │  State Machine  │
                        │  Data Types     │
                        └─────────────────┘
```

### Detailed Component Interaction

1. **ROS 2 Interface**:
   - Messages arrive via subscribers
   - Service requests are received
   - Action goals are accepted
   - Parameter updates are processed

2. **Middleware Processing**:
   - ROS messages are converted to core data types
   - Requests are validated and routed
   - State machine transitions are triggered
   - Algorithm processing is initiated

3. **Algorithm Execution**:
   - Pure business logic processing
   - No ROS dependencies
   - Results returned to middleware

4. **Response Generation**:
   - Algorithm results converted back to ROS messages
   - Responses published/sent via appropriate ROS interfaces
   - State machine updated based on results

## State Machine Architecture

### Operational States

The system maintains an internal operational state machine independent of the ROS 2 lifecycle:

- **UNINITIALIZED**: Initial state before configuration
- **IDLE**: Configured but not processing
- **PROCESSING**: Actively processing data
- **ERROR**: Error state requiring intervention
- **MAINTENANCE**: Maintenance mode
- **EMERGENCY_STOP**: Emergency stop state

### State Transitions

```
┌─────────────────┐
│  UNINITIALIZED  │
└───────┬─────────┘
        │ INITIALIZE
        ▼
┌─────────────────┐     ┌─────────────────┐
│      IDLE       │────►│  PROCESSING     │
└───────┬─────────┘     └────────┬────────┘
        │                      │
        │ ENTER_MAINTENANCE     │ STOP_PROCESSING
        ▼                      │
┌─────────────────┐             │
│  MAINTENANCE    │             │
└───────┬─────────┘             │
        │ EXIT_MAINTENANCE      │
        ▼                      │
┌─────────────────┐             │
│      ERROR      │◄────────────┘
└───────┬─────────┘
        │ ERROR_CLEARED
        ▼
┌─────────────────┐
│ EMERGENCY_STOP  │
└───────┬─────────┘
        │ RESET
        ▼
┌─────────────────┐
│  UNINITIALIZED  │
└─────────────────┘
```

## Communication Patterns

### Publishers

- **Status Publisher**: Lifecycle-aware status updates
- **Command Publisher**: Output commands (e.g., velocity commands)
- **Diagnostics Publisher**: System diagnostics and error information
- **Pose Publisher**: Position and orientation data

### Subscribers

- **Point Cloud Subscriber**: Sensor data input
- **Command Subscriber**: External command input
- **Odometry Subscriber**: Position and velocity feedback
- **IMU Subscriber**: Inertial measurement unit data

### Services

- **Trigger Services**: Simple trigger-based operations
- **SetBool Services**: Boolean parameter setting
- **Compute Service**: Mathematical computation service

### Actions

- **Fibonacci Action**: Long-running computation with feedback

### Parameters

- **Algorithm Parameters**: Gain, offset, filter coefficients
- **Processing Parameters**: Rates, timeouts
- **Debug Parameters**: Logging and debugging flags

## Threading Model

### Main Thread

- Handles ROS 2 callbacks
- Manages lifecycle transitions
- Coordinates state machine updates

### Processing Threads

- Algorithm execution in separate threads when needed
- Action server execution in dedicated threads
- Timer callbacks in main thread

### Thread Safety

- State machine uses mutex for thread-safe transitions
- Atomic operations for simple state variables
- Careful design to minimize locking overhead

## Error Handling

### Error Detection

- Input validation in algorithms
- ROS 2 communication error handling
- State machine validation

### Error Recovery

- Automatic state transitions to error states
- Callback mechanisms for error notification
- Recovery procedures through state machine

### Error Reporting

- Diagnostic messages
- Status updates
- Parameter validation feedback

## Extensibility

### Adding New Algorithms

1. Create new class inheriting from `IAlgorithm`
2. Implement required virtual methods
3. Register with middleware layer
4. Add unit tests

### Adding New Communication Patterns

1. Extend `RosInterfaces` class
2. Add setup/cleanup methods
3. Implement conversion functions
4. Add integration tests

### Adding New States

1. Update `OperationalState` enum
2. Add transitions to state machine
3. Update state change callbacks
4. Add tests for new states

## Performance Considerations

### Memory Management

- Smart pointers for automatic memory management
- Object pooling for frequently created/destroyed objects
- Careful design to minimize memory allocations

### CPU Usage

- Efficient algorithms with minimal overhead
- Timer-based processing with configurable rates
- Asynchronous processing for long-running operations

### Communication

- QoS settings appropriate for use case
- Intra-process communication where beneficial
- Message filtering and throttling when needed

## Testing Strategy

### Unit Tests

- Core logic testing without ROS dependencies
- Algorithm testing with mock data
- State machine transition testing

### Integration Tests

- ROS 2 interface testing
- Lifecycle transition testing
- End-to-end message flow testing

### Performance Tests

- Processing rate validation
- Memory usage monitoring
- Communication latency testing

## Deployment

### Containerization

- Docker support with multi-stage builds
- Optimized images for production
- Development images with debugging tools

### Configuration

- YAML-based parameter configuration
- Environment variable support
- Runtime parameter updates

### Monitoring

- Diagnostic aggregation
- Status reporting
- Performance metrics collection