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
    , occupied_threshold_(0.7)
    , intensity_max_(255.0)
    , intensity_threshold_(35.0)
    , enable_incremental_sync_(true)
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
    observation_counts_.clear();
    voxels_log_odds_.clear();
    modified_keys_.clear();
}

void ProbabilityUpdater::set_occupied_threshold(double occupied_thresh)
{
    occupied_threshold_ = occupied_thresh;
}

void ProbabilityUpdater::set_intensity_params(double intensity_threshold, double intensity_max)
{
    intensity_threshold_ = intensity_threshold;
    intensity_max_ = intensity_max;
}

void ProbabilityUpdater::batch_update_weighted_average(
    const Eigen::MatrixXd& points,
    const Eigen::VectorXd& intensities,
    const std::vector<bool>& is_occupied)
{
    if (points.rows() != intensities.rows()) {
        throw std::invalid_argument("Points and intensities must have same number of rows");
    }

    if (points.cols() != 3) {
        throw std::invalid_argument("Points must be Nx3 matrix");
    }

    // Process each point with weighted average update
    for (int i = 0; i < points.rows(); ++i) {
        double x = points(i, 0);
        double y = points(i, 1);
        double z = points(i, 2);
        double intensity = intensities(i);

        // Generate voxel key
        std::string key = world_to_key(x, y, z);

        // Increment observation count
        observation_counts_[key]++;
        int count = observation_counts_[key];

        // Skip low intensity points
        if (intensity <= intensity_threshold_) {
            continue;
        }

        // Normalize intensity: [threshold, max] -> [0.7, 0.95]
        double normalized_intensity = std::max(intensity_threshold_, std::min(intensity_max_, intensity));
        double observed_prob = occupied_threshold_ + 0.25 *
            (normalized_intensity - intensity_threshold_) / (intensity_max_ - intensity_threshold_);

        // Clamp observed probability
        observed_prob = std::max(occupied_threshold_, std::min(0.95, observed_prob));

        // Get current log-odds (default 0.0 for unknown voxels)
        double current_log_odds = 0.0;
        auto it = voxels_log_odds_.find(key);
        if (it != voxels_log_odds_.end()) {
            current_log_odds = it->second;
        }

        // Convert current log-odds to probability
        double current_prob = log_odds_to_probability(current_log_odds);

        // Apply weighted average: new_prob = (n * old_prob + obs_prob) / (n + 1)
        // where n = count - 1 (previous observations)
        double new_prob = ((count - 1) * current_prob + observed_prob) / static_cast<double>(count);

        // Clamp to prevent extreme values
        new_prob = std::max(min_probability_, std::min(max_probability_, new_prob));

        // Convert back to log-odds and store
        double new_log_odds = probability_to_log_odds(new_prob);
        voxels_log_odds_[key] = new_log_odds;

        // Track modified voxel for incremental sync
        modified_keys_.insert(key);
    }

    // Sync with octree_mapper_ for visualization
    if (enable_incremental_sync_) {
        sync_modified_voxels_to_octree();
    } else {
        sync_all_voxels_to_octree();
    }

    // Clear modified keys after sync
    modified_keys_.clear();
}

int ProbabilityUpdater::get_observation_count(const std::string& key) const
{
    auto it = observation_counts_.find(key);
    if (it != observation_counts_.end()) {
        return it->second;
    }
    return 0;
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

std::string ProbabilityUpdater::world_to_key(double x, double y, double z) const
{
    // Discretize coordinates to voxel grid indices
    int ix = static_cast<int>(std::floor(x / resolution_));
    int iy = static_cast<int>(std::floor(y / resolution_));
    int iz = static_cast<int>(std::floor(z / resolution_));

    // Create string key: "ix_iy_iz"
    std::ostringstream oss;
    oss << ix << "_" << iy << "_" << iz;
    return oss.str();
}

void ProbabilityUpdater::set_incremental_sync(bool enable)
{
    enable_incremental_sync_ = enable;
}

void ProbabilityUpdater::force_full_sync()
{
    sync_all_voxels_to_octree();
    modified_keys_.clear();
}

void ProbabilityUpdater::sync_modified_voxels_to_octree()
{
    // Sync only modified voxels - O(N) complexity where N = modified voxels
    if (modified_keys_.empty()) {
        return;
    }

    int num_modified = modified_keys_.size();
    Eigen::MatrixXd voxel_points(num_modified, 3);
    Eigen::VectorXd voxel_log_odds(num_modified);

    int idx = 0;
    for (const std::string& key : modified_keys_) {
        // Get log-odds value
        auto it = voxels_log_odds_.find(key);
        if (it == voxels_log_odds_.end()) {
            continue;  // Skip if not found (should not happen)
        }
        double log_odds_val = it->second;

        // Parse key back to coordinates
        std::istringstream ss(key);
        std::string token;
        std::vector<double> coords;

        while (std::getline(ss, token, '_')) {
            coords.push_back(std::stod(token));
        }

        if (coords.size() == 3) {
            voxel_points(idx, 0) = coords[0] * resolution_;  // Reconstruct world coordinates
            voxel_points(idx, 1) = coords[1] * resolution_;
            voxel_points(idx, 2) = coords[2] * resolution_;
            voxel_log_odds(idx) = log_odds_val;
            idx++;
        }
    }

    // Update octree mapper for visualization
    if (idx > 0) {
        voxel_points.conservativeResize(idx, 3);
        voxel_log_odds.conservativeResize(idx);

        try {
            octree_mapper_->batch_update_with_log_odds(voxel_points, voxel_log_odds);
        } catch (const std::exception& e) {
            std::cerr << "[ProbabilityUpdater] Failed to sync modified voxels: " << e.what() << std::endl;
        }
    }
}

void ProbabilityUpdater::sync_all_voxels_to_octree()
{
    // Sync all voxels - O(M) complexity where M = total map size
    int num_voxels = voxels_log_odds_.size();
    if (num_voxels == 0) {
        return;
    }

    Eigen::MatrixXd voxel_points(num_voxels, 3);
    Eigen::VectorXd voxel_log_odds(num_voxels);

    int idx = 0;
    for (const auto& pair : voxels_log_odds_) {
        // Parse key back to coordinates
        std::string key = pair.first;
        double log_odds_val = pair.second;

        // Key format: "x_y_z"
        std::istringstream ss(key);
        std::string token;
        std::vector<double> coords;

        while (std::getline(ss, token, '_')) {
            coords.push_back(std::stod(token));
        }

        if (coords.size() == 3) {
            voxel_points(idx, 0) = coords[0] * resolution_;  // Reconstruct world coordinates
            voxel_points(idx, 1) = coords[1] * resolution_;
            voxel_points(idx, 2) = coords[2] * resolution_;
            voxel_log_odds(idx) = log_odds_val;
            idx++;
        }
    }

    // Update octree mapper for visualization
    if (idx > 0) {
        voxel_points.conservativeResize(idx, 3);
        voxel_log_odds.conservativeResize(idx);

        try {
            octree_mapper_->batch_update_with_log_odds(voxel_points, voxel_log_odds);
        } catch (const std::exception& e) {
            std::cerr << "[ProbabilityUpdater] Failed to sync all voxels: " << e.what() << std::endl;
        }
    }
}

}  // namespace sonar_3d_reconstruction