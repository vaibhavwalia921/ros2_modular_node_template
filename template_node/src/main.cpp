#include "rclcpp/rclcpp.hpp"
#include "template_node/middleware/lifecycle_node_wrapper.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    rclcpp::NodeOptions options;
    auto node = std::make_shared<template_node::middleware::LifecycleNodeWrapper>(options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}