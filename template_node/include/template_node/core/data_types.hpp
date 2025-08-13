#pragma once

#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <optional>
#include <variant>

namespace template_node::core {

// Data structures used by algorithms
struct ProcessingRequest {
    std::vector<double> input_data;
    double timestamp;
    std::string request_id;
};

struct ProcessingResult {
    std::vector<double> output_data;
    double processing_time;
    std::string status;
    std::string request_id;
};

// Common data types used across the core layer
using Timestamp = std::chrono::time_point<std::chrono::system_clock>;
using Duration = std::chrono::duration<double>;

// Sensor data structures
struct SensorData {
    std::string sensor_id;
    Timestamp timestamp;
    std::vector<double> values;
    std::map<std::string, std::string> metadata;
};

// Point cloud data structure (simplified)
struct PointCloud {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
    std::vector<float> intensity;
    Timestamp timestamp;
    std::string frame_id;
};

// Command structure
struct Command {
    enum class Type {
        START,
        STOP,
        PAUSE,
        RESUME,
        RESET,
        EMERGENCY_STOP,
        CUSTOM
    };
    
    Type type;
    std::string id;
    std::map<std::string, double> parameters;
    Timestamp timestamp;
};

// Status information
struct StatusInfo {
    std::string node_name;
    std::string state;
    std::string lifecycle_state;
    double cpu_usage;
    double memory_usage;
    Timestamp timestamp;
    std::map<std::string, std::string> details;
};

// Configuration structure
struct Configuration {
    std::map<std::string, double> algorithm_params;
    std::map<std::string, bool> feature_flags;
    std::map<std::string, std::string> string_params;
};

// Error information
struct ErrorInfo {
    enum class Severity {
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };
    
    std::string error_code;
    std::string message;
    Severity severity;
    Timestamp timestamp;
    std::optional<std::string> suggested_action;
};

// Metrics for performance monitoring
struct PerformanceMetrics {
    double processing_time_ms;
    double latency_ms;
    size_t processed_items;
    size_t failed_items;
    double success_rate;
    Timestamp start_time;
    Timestamp end_time;
};

// Result status enum
enum class ResultStatus {
    SUCCESS,
    PARTIAL_SUCCESS,
    FAILURE,
    TIMEOUT,
    CANCELLED,
    INVALID_INPUT,
    PROCESSING_ERROR
};

// Extended processing request
struct ExtendedProcessingRequest : ProcessingRequest {
    std::optional<Command> command;
    std::optional<SensorData> sensor_data;
    std::optional<PointCloud> point_cloud;
    int priority;
};

// Extended processing result
struct ExtendedProcessingResult : ProcessingResult {
    ResultStatus status_enum;
    std::optional<ErrorInfo> error;
    PerformanceMetrics metrics;
};

// Type aliases for common containers
using ParameterMap = std::map<std::string, std::variant<int, double, bool, std::string>>;
using DataBuffer = std::vector<std::vector<double>>;
using TimeSeries = std::vector<std::pair<Timestamp, double>>;

// Utility functions
inline Timestamp now() {
    return std::chrono::system_clock::now();
}

inline double timestamp_to_seconds(const Timestamp& ts) {
    auto duration = ts.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
}

inline Duration time_diff(const Timestamp& end, const Timestamp& start) {
    return end - start;
}

} // namespace template_node::core