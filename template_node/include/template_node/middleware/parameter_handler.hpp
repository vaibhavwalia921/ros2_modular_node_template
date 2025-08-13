#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp" 
#include <map>
#include <string>
#include <functional>
#include <vector>

namespace template_node::middleware {

class ParameterHandler {
public:
    using ParameterCallback = std::function<void(const rclcpp::Parameter&)>;
    
    explicit ParameterHandler(rclcpp_lifecycle::LifecycleNode* node);
    ~ParameterHandler() = default;
    
    // Parameter declaration and initialization
    void declare_parameters();
    void load_from_yaml(const std::string& yaml_file);
    
    // Parameter access
    template<typename T>
    T get_parameter(const std::string& name) const;
    
    std::map<std::string, double> get_algorithm_params() const;
    
    // Dynamic parameter handling
    void register_parameter_callback(const std::string& param_name, ParameterCallback callback);
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // Validation
    bool validate_parameter(const rclcpp::Parameter& param);
    
private:
    rclcpp_lifecycle::LifecycleNode* node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::map<std::string, ParameterCallback> parameter_callbacks_;
    
    // Parameter definitions
    struct ParameterDefinition {
        std::string name;
        rclcpp::ParameterType type;
        std::string description;
        rclcpp::ParameterValue default_value;
        std::function<bool(const rclcpp::Parameter&)> validator;
    };
    
    std::vector<ParameterDefinition> parameter_definitions_;
    void initialize_parameter_definitions();
};

} // namespace template_node::middleware