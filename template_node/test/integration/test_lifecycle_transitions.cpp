#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "template_node/middleware/lifecycle_node_wrapper.hpp"

using namespace template_node::middleware;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LifecycleTransitionTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        rclcpp::NodeOptions options;
        node_ = std::make_shared<LifecycleNodeWrapper>(options);
        
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_->get_node_base_interface());
        
        // Run executor in a separate thread
        executor_thread_ = std::thread([this]() {
            executor_->spin();
        });
    }
    
    void TearDown() override {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
        
        node_.reset();
        executor_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<LifecycleNodeWrapper> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
};

TEST_F(LifecycleTransitionTest, InitialState) {
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LifecycleTransitionTest, ConfigureTransition) {
    auto result = node_->on_configure(node_->get_current_state());
    EXPECT_EQ(result, CallbackReturn::SUCCESS);
    
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(LifecycleTransitionTest, ActivateTransition) {
    // First configure
    auto configure_result = node_->on_configure(node_->get_current_state());
    EXPECT_EQ(configure_result, CallbackReturn::SUCCESS);
    
    // Then activate
    auto activate_result = node_->on_activate(node_->get_current_state());
    EXPECT_EQ(activate_result, CallbackReturn::SUCCESS);
    
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(LifecycleTransitionTest, DeactivateTransition) {
    // Configure and activate first
    node_->on_configure(node_->get_current_state());
    node_->on_activate(node_->get_current_state());
    
    // Then deactivate
    auto deactivate_result = node_->on_deactivate(node_->get_current_state());
    EXPECT_EQ(deactivate_result, CallbackReturn::SUCCESS);
    
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(LifecycleTransitionTest, CleanupTransition) {
    // Configure first
    node_->on_configure(node_->get_current_state());
    
    // Then cleanup
    auto cleanup_result = node_->on_cleanup(node_->get_current_state());
    EXPECT_EQ(cleanup_result, CallbackReturn::SUCCESS);
    
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(LifecycleTransitionTest, ShutdownFromActive) {
    // Configure and activate
    node_->on_configure(node_->get_current_state());
    node_->on_activate(node_->get_current_state());
    
    // Shutdown from active state
    auto shutdown_result = node_->on_shutdown(node_->get_current_state());
    EXPECT_EQ(shutdown_result, CallbackReturn::SUCCESS);
    
    auto state = node_->get_current_state();
    EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(LifecycleTransitionTest, FullLifecycle) {
    // Complete lifecycle sequence
    EXPECT_EQ(node_->on_configure(node_->get_current_state()), CallbackReturn::SUCCESS);
    EXPECT_EQ(node_->on_activate(node_->get_current_state()), CallbackReturn::SUCCESS);
    EXPECT_EQ(node_->on_deactivate(node_->get_current_state()), CallbackReturn::SUCCESS);
    EXPECT_EQ(node_->on_cleanup(node_->get_current_state()), CallbackReturn::SUCCESS);
    
    // Can configure again after cleanup
    EXPECT_EQ(node_->on_configure(node_->get_current_state()), CallbackReturn::SUCCESS);
}