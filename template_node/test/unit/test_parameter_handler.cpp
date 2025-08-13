#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "template_node/middleware/parameter_handler.hpp"

using namespace template_node::middleware;

class ParameterHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_param_node");
        param_handler_ = std::make_unique<ParameterHandler>(node_.get());
    }
    
    void TearDown() override {
        param_handler_.reset();
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<ParameterHandler> param_handler_;
};

TEST_F(ParameterHandlerTest, DeclareParameters) {
    EXPECT_NO_THROW(param_handler_->declare_parameters());
}

TEST_F(ParameterHandlerTest, GetAlgorithmParams) {
    param_handler_->declare_parameters();
    
    auto params = param_handler_->get_algorithm_params();
    
    EXPECT_TRUE(params.count("gain") > 0);
    EXPECT_TRUE(params.count("offset") > 0);
    EXPECT_TRUE(params.count("filter_coefficient") > 0);
}

TEST_F(ParameterHandlerTest, ValidateParameter) {
    param_handler_->declare_parameters();
    
    // Valid parameter
    rclcpp::Parameter valid_gain("algorithm.gain", 2.0);
    EXPECT_TRUE(param_handler_->validate_parameter(valid_gain));
    
    // Invalid parameter (negative gain)
    rclcpp::Parameter invalid_gain("algorithm.gain", -1.0);
    EXPECT_FALSE(param_handler_->validate_parameter(invalid_gain));
    
    // Invalid parameter (filter coefficient > 1.0)
    rclcpp::Parameter invalid_filter("algorithm.filter_coefficient", 1.5);
    EXPECT_FALSE(param_handler_->validate_parameter(invalid_filter));
}

TEST_F(ParameterHandlerTest, ParameterCallback) {
    param_handler_->declare_parameters();
    
    bool callback_called = false;
    double received_value = 0.0;
    
    param_handler_->register_parameter_callback(
        "algorithm.gain",
        [&callback_called, &received_value](const rclcpp::Parameter& param) {
            callback_called = true;
            received_value = param.as_double();
        });
    
    // Simulate parameter change
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("algorithm.gain", 3.0)
    };
    
    auto result = param_handler_->on_parameter_change(params);
    
    EXPECT_TRUE(result.successful);
    EXPECT_TRUE(callback_called);
    EXPECT_DOUBLE_EQ(received_value, 3.0);
}

TEST_F(ParameterHandlerTest, InvalidParameterChange) {
    param_handler_->declare_parameters();
    
    // Try to set invalid parameter value
    std::vector<rclcpp::Parameter> params = {
        rclcpp::Parameter("algorithm.filter_coefficient", 2.0)  // Invalid: > 1.0
    };
    
    auto result = param_handler_->on_parameter_change(params);
    
    EXPECT_FALSE(result.successful);
    EXPECT_FALSE(result.reason.empty());
}