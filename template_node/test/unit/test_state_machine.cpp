#include <gtest/gtest.h>
#include "template_node/core/state_machine.hpp"

using namespace template_node::core;

class StateMachineTest : public ::testing::Test {
protected:
    void SetUp() override {
        state_machine = std::make_unique<StateMachine>();
    }
    
    std::unique_ptr<StateMachine> state_machine;
};

TEST_F(StateMachineTest, InitialState) {
    EXPECT_EQ(state_machine->get_current_state(), OperationalState::UNINITIALIZED);
}

TEST_F(StateMachineTest, ValidTransition) {
    EXPECT_TRUE(state_machine->transition(StateTransition::INITIALIZE));
    EXPECT_EQ(state_machine->get_current_state(), OperationalState::IDLE);
}

TEST_F(StateMachineTest, InvalidTransition) {
    // Cannot go directly from UNINITIALIZED to PROCESSING
    EXPECT_FALSE(state_machine->transition(StateTransition::START_PROCESSING));
    EXPECT_EQ(state_machine->get_current_state(), OperationalState::UNINITIALIZED);
}

TEST_F(StateMachineTest, StateChangeCallback) {
    bool callback_called = false;
    OperationalState old_state_received;
    OperationalState new_state_received;
    
    state_machine->register_state_change_callback(
        [&](OperationalState old_state, OperationalState new_state) {
            callback_called = true;
            old_state_received = old_state;
            new_state_received = new_state;
        });
    
    state_machine->transition(StateTransition::INITIALIZE);
    
    EXPECT_TRUE(callback_called);
    EXPECT_EQ(old_state_received, OperationalState::UNINITIALIZED);
    EXPECT_EQ(new_state_received, OperationalState::IDLE);
}

TEST_F(StateMachineTest, AvailableTransitions) {
    state_machine->transition(StateTransition::INITIALIZE);
    
    auto transitions = state_machine->get_available_transitions();
    
    EXPECT_TRUE(std::find(transitions.begin(), transitions.end(), 
                         StateTransition::START_PROCESSING) != transitions.end());
    EXPECT_TRUE(std::find(transitions.begin(), transitions.end(), 
                         StateTransition::ENTER_MAINTENANCE) != transitions.end());
}