#include "probability_updater.h"
#include "iwlo_updater.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <unordered_map>
#include <sstream>

namespace sonar_3d_reconstruction
{

ProbabilityUpdater::ProbabilityUpdater(double resolution)
    : resolution_(resolution)
    , log_odds_occupied_(3.5)      // 1.5 -> 3.5 (occupied dominant)
    , log_odds_free_(-3.0)         // -2.0 -> -3.0 (balanced)
    , adaptive_enabled_(true)
    , adaptive_threshold_(0.5)
    , adaptive_max_ratio_(0.3)
    , min_probability_(0.00005)    // 0.12 -> 0.00005 (for L_min=-10.0)
    , max_probability_(0.99995)    // 0.97 -> 0.99995 (for L_max=10.0)
    , occupied_threshold_(0.7)
    , intensity_max_(255.0)
    , intensity_threshold_(35.0)
    , sharpness_(0.1)              // 3.0 -> 0.1 (stonefish recommended)
    , decay_rate_(0.1)
    , min_alpha_(0.3)              // 0.1 -> 0.3 (for change detection)
    , L_min_(-10.0)                // -2.0 -> -10.0 (extended bounds)
    , L_max_(10.0)                 // 3.5 -> 10.0 (extended bounds)
    , enable_incremental_sync_(true)
{
    // Use direct log-odds storage (Python SimpleOctree style) for exact compatibility
    // octree_mapper_ is kept for fallback but not used
    // Note: log_odds_free_ is already negative (-2.0), so prob_miss = sigmoid(-2.0) = 0.12
    octree_mapper_ = std::make_unique<OctreeMapper>(
        resolution,
        log_odds_to_probability(log_odds_occupied_),  // prob_hit = sigmoid(1.5) = 0.82
        log_odds_to_probability(log_odds_free_),      // prob_miss = sigmoid(-2.0) = 0.12
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
    // log_odds_free is already negative, so prob_miss = sigmoid(log_odds_free) < 0.5
    double prob_hit = log_odds_to_probability(log_odds_occupied);
    double prob_miss = log_odds_to_probability(log_odds_free);

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

// batch_update() method removed - using IWLO only

Eigen::MatrixXd ProbabilityUpdater::get_occupied_voxels(double min_probability)
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

// batch_update_weighted_average() method removed - using IWLO only

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
    return IWLOUpdater::log_odds_to_probability(log_odds);
}

double ProbabilityUpdater::probability_to_log_odds(double probability) const
{
    return IWLOUpdater::probability_to_log_odds(probability);
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

void ProbabilityUpdater::set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                                          double L_min, double L_max)
{
    sharpness_ = sharpness;
    decay_rate_ = decay_rate;
    min_alpha_ = min_alpha;
    L_min_ = L_min;
    L_max_ = L_max;
}

double ProbabilityUpdater::intensity_to_weight(double intensity) const
{
    // Delegate to IWLOUpdater (shared implementation)
    return IWLOUpdater::intensity_to_weight(intensity,
                                            intensity_threshold_,
                                            intensity_max_,
                                            sharpness_);
}

double ProbabilityUpdater::compute_alpha(int observation_count) const
{
    // Delegate to IWLOUpdater (shared implementation)
    return IWLOUpdater::compute_alpha(observation_count, decay_rate_, min_alpha_);
}

void ProbabilityUpdater::batch_update_iwlo(
    const Eigen::MatrixXd& points,
    const Eigen::VectorXd& intensities,
    const std::vector<bool>& is_occupied)
{
    if (points.rows() != intensities.rows()) {
        throw std::invalid_argument("Points and intensities must have same number of rows");
    }

    if (points.rows() != static_cast<int>(is_occupied.size())) {
        throw std::invalid_argument("Points and is_occupied must have same number of elements");
    }

    if (points.cols() != 3) {
        throw std::invalid_argument("Points must have 3 columns (x, y, z)");
    }

    for (int i = 0; i < points.rows(); ++i) {
        // Get voxel key
        std::string key = world_to_key(points(i, 0), points(i, 1), points(i, 2));

        // Increment observation count
        observation_counts_[key]++;
        int n = observation_counts_[key];

        double intensity = intensities(i);

        // Get current log-odds (default 0.0 for unknown voxels)
        double current_log_odds = 0.0;
        auto it = voxels_log_odds_.find(key);
        if (it != voxels_log_odds_.end()) {
            current_log_odds = it->second;
        }

        // Convert to probability for adaptive scaling
        double current_prob = log_odds_to_probability(current_log_odds);

        // 1. Compute intensity-based weight using sigmoid
        double w_intensity = intensity_to_weight(intensity);

        // 2. Compute learning rate based on observation count
        double alpha_n = compute_alpha(n - 1);  // n-1 because we already incremented

        // 3. Compute adaptive scaling (bidirectional protection)
        double adapt_scale = 1.0;
        if (adaptive_enabled_) {
            if (intensity > intensity_threshold_) {
                // Occupied update: protect free voxels (existing logic)
                if (current_prob < adaptive_threshold_) {
                    adapt_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                }
            } else {
                // Free update: protect occupied voxels (new - bidirectional)
                if (current_prob > (1.0 - adaptive_threshold_)) {
                    double protection_factor = (current_prob - (1.0 - adaptive_threshold_)) / adaptive_threshold_;
                    adapt_scale = adaptive_max_ratio_ + (1.0 - adaptive_max_ratio_) * (1.0 - protection_factor);
                }
            }
        }

        // 4. Compute log-odds update
        double delta_L;
        if (intensity > intensity_threshold_) {
            // Occupied update: ΔL = L_occ × w(I) × α(n) × scale
            delta_L = log_odds_occupied_ * w_intensity * alpha_n * adapt_scale;
        } else {
            // Free space update: ΔL = L_free × α(n) × scale (now with bidirectional protection)
            delta_L = log_odds_free_ * alpha_n * adapt_scale;
        }

        // 5. Apply update with saturation limits
        double new_log_odds = current_log_odds + delta_L;
        new_log_odds = std::max(L_min_, std::min(L_max_, new_log_odds));

        // Store updated value
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

}  // namespace sonar_3d_reconstruction