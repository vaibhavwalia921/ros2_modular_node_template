#include <gtest/gtest.h>
#include <cmath>  
#include <limits>  
#include "template_node/algorithms/example_algorithm.hpp"

using namespace template_node;

class AlgorithmTest : public ::testing::Test {
protected:
    void SetUp() override {
        algorithm = std::make_unique<algorithms::ExampleAlgorithm>();
    }
    
    std::unique_ptr<algorithms::ExampleAlgorithm> algorithm;
};

TEST_F(AlgorithmTest, ConfigureWithValidParams) {
    std::map<std::string, double> params = {
        {"gain", 2.0},
        {"offset", 1.0},
        {"filter_coefficient", 0.7}
    };
    
    EXPECT_TRUE(algorithm->configure(params));
}

TEST_F(AlgorithmTest, ConfigureWithInvalidFilterCoefficient) {
    std::map<std::string, double> params = {
        {"filter_coefficient", 1.5}  // Invalid: > 1.0
    };
    
    EXPECT_FALSE(algorithm->configure(params));
}

TEST_F(AlgorithmTest, ProcessValidInput) {
    core::ProcessingRequest request;
    request.input_data = {1.0, 2.0, 3.0, 4.0, 5.0};
    request.timestamp = 123.456;
    request.request_id = "test_001";
    
    auto result = algorithm->process(request);
    
    EXPECT_EQ(result.status, "success");
    EXPECT_EQ(result.request_id, request.request_id);
    EXPECT_EQ(result.output_data.size(), request.input_data.size());
    EXPECT_GT(result.processing_time, 0.0);
}

TEST_F(AlgorithmTest, ProcessEmptyInput) {
    core::ProcessingRequest request;
    request.request_id = "test_002";
    
    auto result = algorithm->process(request);
    
    EXPECT_EQ(result.status, "invalid_input");
    EXPECT_EQ(result.request_id, request.request_id);
}

TEST_F(AlgorithmTest, ProcessWithNaN) {
    core::ProcessingRequest request;
    request.input_data = {1.0, std::numeric_limits<double>::quiet_NaN(), 3.0};
    request.request_id = "test_003";
    
    auto result = algorithm->process(request);
    
    EXPECT_EQ(result.status, "invalid_input");
}

TEST_F(AlgorithmTest, ResetAlgorithm) {
    std::map<std::string, double> params = {
        {"gain", 5.0},
        {"offset", 10.0}
    };
    
    algorithm->configure(params);
    algorithm->reset();
    
    EXPECT_EQ(algorithm->get_status(), "reset");
    
    // Process with default parameters after reset
    core::ProcessingRequest request;
    request.input_data = {1.0};
    auto result = algorithm->process(request);
    
    // With default gain=1.0 and offset=0.0
    EXPECT_NEAR(result.output_data[0], 1.0, 0.001);
}