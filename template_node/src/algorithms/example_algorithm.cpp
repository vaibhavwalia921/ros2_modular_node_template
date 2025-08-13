#include "template_node/algorithms/example_algorithm.hpp"
#include <chrono>
#include <numeric>
#include <algorithm>
#include <cmath>

namespace template_node::algorithms {

ExampleAlgorithm::ExampleAlgorithm() 
    : gain_(1.0), offset_(0.0), filter_coefficient_(0.5), configured_(false) {
    status_ = "initialized";
}

core::ProcessingResult ExampleAlgorithm::process(const core::ProcessingRequest& request) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    core::ProcessingResult result;
    result.request_id = request.request_id;
    
    if (!validate_input(request)) {
        result.status = "invalid_input";
        return result;
    }
    
    // Example processing: apply gain, offset, and simple filter
    result.output_data.reserve(request.input_data.size());
    
    for (size_t i = 0; i < request.input_data.size(); ++i) {
        double processed = request.input_data[i] * gain_ + offset_;
        
        // Simple exponential filter
        if (i > 0 && !result.output_data.empty()) {
            processed = filter_coefficient_ * processed + 
                       (1.0 - filter_coefficient_) * result.output_data.back();
        }
        
        result.output_data.push_back(processed);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    result.processing_time = diff.count();
    result.status = "success";
    
    return result;
}

bool ExampleAlgorithm::configure(const std::map<std::string, double>& params) {
    auto it = params.find("gain");
    if (it != params.end()) {
        gain_ = it->second;
    }
    
    it = params.find("offset");
    if (it != params.end()) {
        offset_ = it->second;
    }
    
    it = params.find("filter_coefficient");
    if (it != params.end()) {
        if (it->second >= 0.0 && it->second <= 1.0) {
            filter_coefficient_ = it->second;
        } else {
            return false;
        }
    }
    
    configured_ = true;
    status_ = "configured";
    return true;
}

bool ExampleAlgorithm::validate_input(const core::ProcessingRequest& request) const {
    if (request.input_data.empty()) {
        return false;
    }
    
    // Check for NaN or Inf values
    return std::none_of(request.input_data.begin(), request.input_data.end(),
                       [](double val) { return std::isnan(val) || std::isinf(val); });
}

void ExampleAlgorithm::reset() {
    gain_ = 1.0;
    offset_ = 0.0;
    filter_coefficient_ = 0.5;
    configured_ = false;
    status_ = "reset";
}

std::string ExampleAlgorithm::get_status() const {
    return status_;
}

} // namespace template_node::algorithms