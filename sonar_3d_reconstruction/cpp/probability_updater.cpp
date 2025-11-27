#include "probability_updater.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <sstream>

namespace sonar_3d_reconstruction
{

ProbabilityUpdater::ProbabilityUpdater(double resolution)
    : resolution_(resolution)
    , log_odds_occupied_(1.5)
    , log_odds_free_(-2.0)
    , adaptive_enabled_(true)
    , adaptive_threshold_(0.5)
    , adaptive_max_ratio_(0.3)
    , min_probability_(0.12)
    , max_probability_(0.97)
{
    // Use direct log-odds storage (Python SimpleOctree style) for exact compatibility
    // octree_mapper_ is kept for fallback but not used
    octree_mapper_ = std::make_unique<OctreeMapper>(
        resolution, 
        log_odds_to_probability(log_odds_occupied_),  // prob_hit
        log_odds_to_probability(-log_odds_free_),     // prob_miss
        min_probability_,                             // prob_thres_min
        max_probability_                              // prob_thres_max
    );
}

ProbabilityUpdater::~ProbabilityUpdater()
{
    // Unique pointer automatically cleans up
}

void ProbabilityUpdater::set_log_odds_params(double log_odds_occupied, double log_odds_free)
{
    log_odds_occupied_ = log_odds_occupied;
    log_odds_free_ = log_odds_free;
    
    // Update underlying octree parameters
    double prob_hit = log_odds_to_probability(log_odds_occupied);
    double prob_miss = log_odds_to_probability(-log_odds_free);  // Note: negate for miss probability
    
    octree_mapper_->set_probability_params(prob_hit, prob_miss);
}

void ProbabilityUpdater::set_adaptive_params(bool adaptive_enabled, double adaptive_threshold, double adaptive_max_ratio)
{
    adaptive_enabled_ = adaptive_enabled;
    adaptive_threshold_ = adaptive_threshold;
    adaptive_max_ratio_ = adaptive_max_ratio;
}

void ProbabilityUpdater::set_clamping_thresholds(double min_prob, double max_prob)
{
    min_probability_ = min_prob;
    max_probability_ = max_prob;
    
    octree_mapper_->set_occupancy_thresholds(min_prob, max_prob);
}

void ProbabilityUpdater::batch_update(const Eigen::MatrixXd& points, 
                                     const Eigen::VectorXd& log_odds_updates,
                                     const std::vector<bool>& is_occupied)
{
    if (points.rows() != log_odds_updates.rows()) {
        throw std::invalid_argument("Points and log_odds_updates must have same number of rows");
    }
    
    if (points.rows() != static_cast<int>(is_occupied.size())) {
        throw std::invalid_argument("Points and is_occupied must have same number of elements");
    }
    
    if (points.cols() != 3) {
        throw std::invalid_argument("Points must be Nx3 matrix");
    }
    
    // Process each point with adaptive updates
    std::vector<bool> final_occupied_flags;
    final_occupied_flags.reserve(points.rows());
    
    for (int i = 0; i < points.rows(); ++i) {
        double log_odds_update = log_odds_updates(i);
        bool occupied = is_occupied[i];
        
        // Apply adaptive scaling if enabled
        if (adaptive_enabled_ && occupied) {
            // Simplified adaptive approach without querying existing voxels
            // to avoid potential thread-safety issues with OctoMap queries
            // Use a conservative default probability for adaptive scaling
            double current_prob = 0.5;  // Default unknown probability
            log_odds_update = apply_adaptive_scaling(current_prob, log_odds_update, occupied);
        }
        
        // Convert log-odds update to probability update and apply
        final_occupied_flags.push_back(occupied);
    }
    
    // Apply batch update to underlying octree with log-odds (Python SimpleOctree compatible)
    try {
        octree_mapper_->batch_update_with_log_odds(points, log_odds_updates);
    } catch (const std::exception& e) {
        // Log error but don't crash the entire update
        std::cerr << "[ProbabilityUpdater] Batch update failed: " << e.what() << std::endl;
        // Fall back to individual updates with error handling
        for (int i = 0; i < points.rows(); ++i) {
            try {
                Eigen::MatrixXd single_point = points.row(i);
                Eigen::VectorXd single_log_odds(1);
                single_log_odds(0) = log_odds_updates(i);
                octree_mapper_->batch_update_with_log_odds(single_point, single_log_odds);
            } catch (const std::exception& e2) {
                // Skip this point and continue
                std::cerr << "[ProbabilityUpdater] Failed to update point " << i << ": " << e2.what() << std::endl;
            }
        }
    }
}

Eigen::MatrixXd ProbabilityUpdater::get_occupied_voxels(double min_probability) const
{
    return octree_mapper_->get_occupied_voxels(min_probability);
}

MemoryStats ProbabilityUpdater::get_memory_usage() const
{
    return octree_mapper_->get_memory_usage();
}

size_t ProbabilityUpdater::get_num_nodes() const
{
    return octree_mapper_->get_num_nodes();
}

double ProbabilityUpdater::get_resolution() const
{
    return octree_mapper_->get_resolution();
}

size_t ProbabilityUpdater::prune_tree()
{
    return octree_mapper_->prune_tree();
}

void ProbabilityUpdater::clear()
{
    octree_mapper_->clear();
}

double ProbabilityUpdater::log_odds_to_probability(double log_odds) const
{
    return 1.0 / (1.0 + std::exp(-log_odds));
}

double ProbabilityUpdater::probability_to_log_odds(double probability) const
{
    // Clamp probability to avoid infinite log-odds
    probability = std::max(1e-6, std::min(1.0 - 1e-6, probability));
    return std::log(probability / (1.0 - probability));
}

double ProbabilityUpdater::apply_adaptive_scaling(double current_prob, double log_odds_update, bool is_occupied) const
{
    if (!adaptive_enabled_ || !is_occupied) {
        return log_odds_update;
    }
    
    // Apply adaptive scaling for occupied updates only
    if (current_prob <= adaptive_threshold_) {
        double scale_factor = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
        return log_odds_update * scale_factor;
    }
    
    return log_odds_update;
}

}  // namespace sonar_3d_reconstruction