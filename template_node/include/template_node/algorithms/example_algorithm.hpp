#pragma once

#include "template_node/core/algorithm_interface.hpp"
#include <atomic>

namespace template_node::algorithms {

class ExampleAlgorithm : public core::IAlgorithm {
public:
    ExampleAlgorithm();
    ~ExampleAlgorithm() override = default;
    
    // IAlgorithm interface implementation
    core::ProcessingResult process(const core::ProcessingRequest& request) override;
    bool configure(const std::map<std::string, double>& params) override;
    bool validate_input(const core::ProcessingRequest& request) const override;
    void reset() override;
    std::string get_status() const override;
    
private:
    // Algorithm parameters
    double gain_;
    double offset_;
    double filter_coefficient_;
    
    // Internal state
    std::atomic<bool> configured_;
    std::string status_;
};

} // namespace template_node::algorithms