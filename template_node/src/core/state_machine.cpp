#include "template_node/core/state_machine.hpp"
#include <algorithm>

namespace template_node::core {

StateMachine::StateMachine() : current_state_(OperationalState::UNINITIALIZED) {
    initialize_transition_table();
}

void StateMachine::initialize_transition_table() {
    // Define valid state transitions
    transition_table_[{OperationalState::UNINITIALIZED, StateTransition::INITIALIZE}] = 
        OperationalState::IDLE;
    
    transition_table_[{OperationalState::IDLE, StateTransition::START_PROCESSING}] = 
        OperationalState::PROCESSING;
    transition_table_[{OperationalState::IDLE, StateTransition::ENTER_MAINTENANCE}] = 
        OperationalState::MAINTENANCE;
    
    transition_table_[{OperationalState::PROCESSING, StateTransition::STOP_PROCESSING}] = 
        OperationalState::IDLE;
    transition_table_[{OperationalState::PROCESSING, StateTransition::ERROR_DETECTED}] = 
        OperationalState::ERROR;
    transition_table_[{OperationalState::PROCESSING, StateTransition::EMERGENCY_STOP_TRIGGERED}] = 
        OperationalState::EMERGENCY_STOP;
    
    transition_table_[{OperationalState::ERROR, StateTransition::ERROR_CLEARED}] = 
        OperationalState::IDLE;
    transition_table_[{OperationalState::ERROR, StateTransition::EMERGENCY_STOP_TRIGGERED}] = 
        OperationalState::EMERGENCY_STOP;
    
    transition_table_[{OperationalState::MAINTENANCE, StateTransition::EXIT_MAINTENANCE}] = 
        OperationalState::IDLE;
    
    transition_table_[{OperationalState::EMERGENCY_STOP, StateTransition::RESET}] = 
        OperationalState::UNINITIALIZED;
}

bool StateMachine::transition(StateTransition trigger) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    auto key = std::make_pair(current_state_.load(), trigger);
    auto it = transition_table_.find(key);
    
    if (it == transition_table_.end()) {
        return false;  // Invalid transition
    }
    
    OperationalState old_state = current_state_;
    current_state_ = it->second;
    
    notify_state_change(old_state, current_state_);
    return true;
}

OperationalState StateMachine::get_current_state() const {
    return current_state_.load();
}

std::string StateMachine::get_state_name(OperationalState state) const {
    switch (state) {
        case OperationalState::UNINITIALIZED: return "UNINITIALIZED";
        case OperationalState::IDLE: return "IDLE";
        case OperationalState::PROCESSING: return "PROCESSING";
        case OperationalState::ERROR: return "ERROR";
        case OperationalState::MAINTENANCE: return "MAINTENANCE";
        case OperationalState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

void StateMachine::register_state_change_callback(StateChangeCallback callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    callbacks_.push_back(callback);
}

bool StateMachine::can_transition(StateTransition trigger) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto key = std::make_pair(current_state_.load(), trigger);
    return transition_table_.find(key) != transition_table_.end();
}

std::vector<StateTransition> StateMachine::get_available_transitions() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<StateTransition> available;
    
    for (const auto& [key, _] : transition_table_) {
        if (key.first == current_state_) {
            available.push_back(key.second);
        }
    }
    
    return available;
}

void StateMachine::notify_state_change(OperationalState old_state, OperationalState new_state) {
    for (const auto& callback : callbacks_) {
        callback(old_state, new_state);
    }
}

} // namespace template_node::core