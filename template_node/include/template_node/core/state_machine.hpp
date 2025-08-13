#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace template_node::core {

enum class OperationalState {
    UNINITIALIZED,
    IDLE,
    PROCESSING,
    ERROR,
    MAINTENANCE,
    EMERGENCY_STOP
};

enum class StateTransition {
    INITIALIZE,
    START_PROCESSING,
    STOP_PROCESSING,
    ERROR_DETECTED,
    ERROR_CLEARED,
    ENTER_MAINTENANCE,
    EXIT_MAINTENANCE,
    EMERGENCY_STOP_TRIGGERED,
    RESET
};

class StateMachine {
public:
    using StateChangeCallback = std::function<void(OperationalState, OperationalState)>;
    
    StateMachine();
    ~StateMachine() = default;
    
    // State transitions
    bool transition(StateTransition trigger);
    OperationalState get_current_state() const;
    std::string get_state_name(OperationalState state) const;
    
    // Callbacks
    void register_state_change_callback(StateChangeCallback callback);
    
    // Validation
    bool can_transition(StateTransition trigger) const;
    std::vector<StateTransition> get_available_transitions() const;
    
private:
    mutable std::mutex state_mutex_;
    std::atomic<OperationalState> current_state_;
    std::vector<StateChangeCallback> callbacks_;
    
    // Transition table
    std::map<std::pair<OperationalState, StateTransition>, OperationalState> transition_table_;
    
    void initialize_transition_table();
    void notify_state_change(OperationalState old_state, OperationalState new_state);
};

} // namespace template_node::core