#include "template_node/middleware/parameter_handler.hpp"

namespace template_node::middleware {

ParameterHandler::ParameterHandler(rclcpp_lifecycle::LifecycleNode* node) : node_(node) {
    initialize_parameter_definitions();
    
    // Set parameter change callback
    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ParameterHandler::on_parameter_change, this, std::placeholders::_1));
}

void ParameterHandler::initialize_parameter_definitions() {
    parameter_definitions_ = {
        {
            "algorithm.gain",
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            "Processing gain factor",
            rclcpp::ParameterValue(1.0),
            [](const rclcpp::Parameter& p) { return p.as_double() > 0.0; }
        },
        {
            "algorithm.offset",
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            "Processing offset value",
            rclcpp::ParameterValue(0.0),
            [](const rclcpp::Parameter&) { return true; }
        },
        {
            "algorithm.filter_coefficient",
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            "Filter coefficient (0.0 to 1.0)",
            rclcpp::ParameterValue(0.5),
            [](const rclcpp::Parameter& p) { 
                double val = p.as_double();
                return val >= 0.0 && val <= 1.0;
            }
        },
        {
            "processing.rate_hz",
            rclcpp::ParameterType::PARAMETER_DOUBLE,
            "Main processing loop rate",
            rclcpp::ParameterValue(10.0),
            [](const rclcpp::Parameter& p) { return p.as_double() > 0.0 && p.as_double() <= 100.0; }
        },
        {
            "debug.enable_logging",
            rclcpp::ParameterType::PARAMETER_BOOL,
            "Enable debug logging",
            rclcpp::ParameterValue(false),
            [](const rclcpp::Parameter&) { return true; }
        }
    };
}

void ParameterHandler::declare_parameters() {
    for (const auto& def : parameter_definitions_) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = def.description;
        descriptor.type = static_cast<uint8_t>(def.type);
        
        node_->declare_parameter(def.name, def.default_value, descriptor);
    }
}

std::map<std::string, double> ParameterHandler::get_algorithm_params() const {
    std::map<std::string, double> params;
    
    params["gain"] = node_->get_parameter("algorithm.gain").as_double();
    params["offset"] = node_->get_parameter("algorithm.offset").as_double();
    params["filter_coefficient"] = node_->get_parameter("algorithm.filter_coefficient").as_double();
    
    return params;
}

template<typename T>
T ParameterHandler::get_parameter(const std::string& name) const {
    return node_->get_parameter(name).get_value<T>();
}

void ParameterHandler::register_parameter_callback(const std::string& param_name, 
                                                   ParameterCallback callback) {
    parameter_callbacks_[param_name] = callback;
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::on_parameter_change(
    const std::vector<rclcpp::Parameter>& parameters) {
    
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        // Validate parameter
        if (!validate_parameter(param)) {
            result.successful = false;
            result.reason = "Invalid value for parameter: " + param.get_name();
            return result;
        }
        
        // Call registered callback if exists
        auto it = parameter_callbacks_.find(param.get_name());
        if (it != parameter_callbacks_.end()) {
            it->second(param);
        }
    }
    
    return result;
}

bool ParameterHandler::validate_parameter(const rclcpp::Parameter& param) {
    for (const auto& def : parameter_definitions_) {
        if (def.name == param.get_name()) {
            if (def.validator) {
                return def.validator(param);
            }
            return true;
        }
    }
    return false;  // Unknown parameter
}

// Explicit template instantiations
template double ParameterHandler::get_parameter<double>(const std::string& name) const;
template int ParameterHandler::get_parameter<int>(const std::string& name) const;
template bool ParameterHandler::get_parameter<bool>(const std::string& name) const;
template std::string ParameterHandler::get_parameter<std::string>(const std::string& name) const;

} // namespace template_node::middleware