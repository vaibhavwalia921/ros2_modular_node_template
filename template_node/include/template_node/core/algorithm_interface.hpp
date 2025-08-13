#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include "template_node/core/data_types.hpp"

namespace template_node::core {

// Pure virtual interface for algorithms
class IAlgorithm {
public:
    virtual ~IAlgorithm() = default;
    
    // Core processing function
    virtual ProcessingResult process(const ProcessingRequest& request) = 0;
    
    // Configuration and validation
    virtual bool configure(const std::map<std::string, double>& params) = 0;
    virtual bool validate_input(const ProcessingRequest& request) const = 0;
    
    // State management
    virtual void reset() = 0;
    virtual std::string get_status() const = 0;
};

using AlgorithmPtr = std::shared_ptr<IAlgorithm>;

} // namespace template_node::core