#include <gtest/gtest.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "template_node/middleware/ros_interfaces.hpp"
#include "template_node/core/data_types.hpp"

using namespace template_node;

class RosInterfacesTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create a lifecycle node for testing
        node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
        ros_interfaces_ = std::make_unique<middleware::RosInterfaces>(node_.get());
    }
    
    void TearDown() override {
        ros_interfaces_.reset();
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    std::unique_ptr<middleware::RosInterfaces> ros_interfaces_;
};

TEST_F(RosInterfacesTest, SetupPublishers) {
    EXPECT_NO_THROW(ros_interfaces_->setup_publishers());
}

TEST_F(RosInterfacesTest, SetupSubscribers) {
    EXPECT_NO_THROW(ros_interfaces_->setup_subscribers());
}

TEST_F(RosInterfacesTest, SetupServices) {
    EXPECT_NO_THROW(ros_interfaces_->setup_services());
}

TEST_F(RosInterfacesTest, SetupTimers) {
    EXPECT_NO_THROW(ros_interfaces_->setup_timers());
}

TEST_F(RosInterfacesTest, ActivateDeactivatePublishers) {
    ros_interfaces_->setup_publishers();
    
    EXPECT_FALSE(ros_interfaces_->is_active());
    
    ros_interfaces_->activate_publishers();
    EXPECT_TRUE(ros_interfaces_->is_active());
    
    ros_interfaces_->deactivate_publishers();
    EXPECT_FALSE(ros_interfaces_->is_active());
}

TEST_F(RosInterfacesTest, RegisterCallbacks) {
    bool callback_called = false;
    
    ros_interfaces_->register_command_callback(
        [&callback_called](const core::Command& cmd) {
            callback_called = true;
        });
    
    // Verify callback was registered (no exception thrown)
    EXPECT_NO_THROW(ros_interfaces_->setup_subscribers());
}

TEST_F(RosInterfacesTest, TimerManagement) {
    ros_interfaces_->setup_timers();
    
    EXPECT_NO_THROW(ros_interfaces_->start_timers());
    EXPECT_NO_THROW(ros_interfaces_->stop_timers());
    EXPECT_NO_THROW(ros_interfaces_->set_timer_frequency("main_timer", 20.0));
}

TEST_F(RosInterfacesTest, CleanupAll) {
    ros_interfaces_->setup_publishers();
    ros_interfaces_->setup_subscribers();
    ros_interfaces_->setup_services();
    ros_interfaces_->setup_timers();
    
    EXPECT_NO_THROW(ros_interfaces_->cleanup_all());
}