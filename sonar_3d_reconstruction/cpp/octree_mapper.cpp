#include "octree_mapper.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// OpenMP removed for thread-safety with OctoMap

namespace sonar_3d_reconstruction
{

OctreeMapper::OctreeMapper(double resolution, double prob_hit, double prob_miss,
                           double prob_thres_min, double prob_thres_max)
    : resolution_(resolution)
    , prob_hit_(prob_hit)
    , prob_miss_(prob_miss)
    , prob_thres_min_(prob_thres_min)
    , prob_thres_max_(prob_thres_max)
    , log_odds_min_(-10.0)
    , log_odds_max_(10.0)
    , cached_num_nodes_(0)
    , cache_valid_(false)
{
    octree_ = std::make_unique<octomap::OcTree>(resolution_);
    
    // Set probability parameters
    octree_->setProbHit(prob_hit_);
    octree_->setProbMiss(prob_miss_);
    // Set occupancy threshold to match Python positive log-odds (>0.5 probability)  
    octree_->setOccupancyThres(0.5);  // Equivalent to log-odds > 0  
    octree_->setClampingThresMin(prob_thres_min_);
    octree_->setClampingThresMax(prob_thres_max_);
}

OctreeMapper::~OctreeMapper()
{
    // Unique pointer automatically cleans up
}

void OctreeMapper::update_point(const Eigen::Vector3d& point, bool occupied)
{
    octomap::point3d octo_point(point.x(), point.y(), point.z());
    
    if (occupied) {
        octree_->updateNode(octo_point, true);  // Occupied
    } else {
        octree_->updateNode(octo_point, false); // Free
    }
    
    invalidate_cache();
}

void OctreeMapper::batch_update(const Eigen::MatrixXd& points, const std::vector<bool>& occupied_flags)
{
    if (points.rows() != static_cast<int>(occupied_flags.size())) {
        throw std::invalid_argument("Number of points must match number of occupied flags");
    }
    
    if (points.cols() != 3) {
        throw std::invalid_argument("Points must be Nx3 matrix");
    }
    
    // Batch update strategy (inspired by stonefish_slam)
    // 1. Collect all valid updates in batch
    // 2. Apply in single pass to minimize OctoMap API calls and node expansion conflicts
    
    std::vector<std::pair<octomap::point3d, bool>> valid_updates;
    valid_updates.reserve(points.rows());
    
    // Phase 1: Validate coordinates and collect updates
    for (int i = 0; i < points.rows(); ++i) {
        double x = points(i, 0);
        double y = points(i, 1); 
        double z = points(i, 2);
        
        // Check for valid coordinates
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            octomap::point3d octo_point(x, y, z);
            valid_updates.emplace_back(octo_point, occupied_flags[i]);
        }
    }
    
    // Phase 2: Apply all updates sequentially
    // This reduces node expansion conflicts compared to interleaved updateNode calls
    for (const auto& update : valid_updates) {
        try {
            octree_->updateNode(update.first, update.second);
        } catch (const std::exception& e) {
            // Continue processing other points if one fails
            std::cerr << "[OctreeMapper] Update failed for point (" 
                      << update.first.x() << ", " << update.first.y() << ", " << update.first.z() 
                      << "): " << e.what() << std::endl;
        }
    }
    
    invalidate_cache();
}

void OctreeMapper::batch_update_with_log_odds(const Eigen::MatrixXd& points, 
                                             const Eigen::VectorXd& log_odds_updates)
{
    if (points.rows() != log_odds_updates.rows()) {
        throw std::invalid_argument("Number of points must match number of log-odds updates");
    }
    
    if (points.cols() != 3) {
        throw std::invalid_argument("Points must be Nx3 matrix");
    }
    
    // Store log-odds updates in our map for later probability calculation
    for (int i = 0; i < points.rows(); ++i) {
        double x = points(i, 0);
        double y = points(i, 1); 
        double z = points(i, 2);
        
        // Check for valid coordinates
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            octomap::point3d octo_point(x, y, z);
            
            // Get current log-odds or initialize to 0
            double current_log_odds = 0.0;
            auto it = log_odds_map_.find(octo_point);
            if (it != log_odds_map_.end()) {
                current_log_odds = it->second;
            }
            
            // Apply update
            double new_log_odds = current_log_odds + log_odds_updates(i);
            
            // Clamp to prevent overflow
            new_log_odds = std::max(log_odds_min_, std::min(log_odds_max_, new_log_odds));
            
            // Store in our map
            log_odds_map_[octo_point] = new_log_odds;
            
            // Also update OctoMap for spatial queries (using probability from log-odds)
            double probability = 1.0 / (1.0 + std::exp(-new_log_odds));
            bool is_occupied = probability > 0.5;
            
            try {
                octree_->updateNode(octo_point, is_occupied);
            } catch (const std::exception& e) {
                std::cerr << "[OctreeMapper] Update failed for point (" 
                          << octo_point.x() << ", " << octo_point.y() << ", " << octo_point.z() 
                          << "): " << e.what() << std::endl;
            }
        }
    }
    
    invalidate_cache();
}

void OctreeMapper::insert_ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& endpoint)
{
    octomap::point3d octo_origin(origin.x(), origin.y(), origin.z());
    octomap::point3d octo_endpoint(endpoint.x(), endpoint.y(), endpoint.z());
    
    // Insert ray: marks free space along ray and occupied at endpoint
    octree_->insertRay(octo_origin, octo_endpoint);
    
    invalidate_cache();
}

Eigen::MatrixXd OctreeMapper::get_occupied_voxels(double min_probability) const
{
    if (min_probability < 0.0) {
        min_probability = 0.5;  // Use default threshold
    }
    
    std::vector<Eigen::Vector4d> occupied_points;
    
    // Convert probability to log-odds threshold (same as Python backend)
    double min_log_odds;
    if (min_probability >= 1.0) {
        min_log_odds = log_odds_max_ - 0.01;  // Use max log-odds
    } else if (min_probability <= 0.0) {
        min_log_odds = log_odds_min_;
    } else {
        min_log_odds = std::log(min_probability / (1.0 - min_probability));
    }
    
    // Iterate through stored log-odds map (Python SimpleOctree style)
    for (const auto& pair : log_odds_map_) {
        const octomap::point3d& coord = pair.first;
        double log_odds = pair.second;
        
        // Apply same threshold logic as Python backend
        if (log_odds > min_log_odds) {
            // Convert log-odds to probability (exact Python logic)
            double probability = 1.0 / (1.0 + std::exp(-log_odds));
            occupied_points.emplace_back(coord.x(), coord.y(), coord.z(), probability);
        }
    }
    
    // Convert to Eigen matrix
    if (occupied_points.empty()) {
        return Eigen::MatrixXd(0, 4);  // Return empty matrix
    }
    
    Eigen::MatrixXd result(occupied_points.size(), 4);
    for (size_t i = 0; i < occupied_points.size(); ++i) {
        result.row(i) = occupied_points[i];
    }
    
    return result;
}

MemoryStats OctreeMapper::get_memory_usage() const
{
    update_cache();
    
    size_t num_leaf_nodes = 0;
    
    // Count leaf nodes
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
        ++num_leaf_nodes;
    }
    
    // Estimate memory usage (rough calculation)
    // Each node typically uses ~32 bytes in OctoMap
    double memory_mb = (cached_num_nodes_ * 32.0) / (1024.0 * 1024.0);
    
    // Memory efficiency: ratio of leaf nodes to total nodes
    double efficiency = (cached_num_nodes_ > 0) ? 
                       static_cast<double>(num_leaf_nodes) / cached_num_nodes_ : 0.0;
    
    return MemoryStats(cached_num_nodes_, num_leaf_nodes, memory_mb, efficiency);
}

size_t OctreeMapper::prune_tree()
{
    size_t nodes_before = get_num_nodes();
    octree_->prune();
    size_t nodes_after = get_num_nodes();
    
    invalidate_cache();
    
    return nodes_before - nodes_after;
}

void OctreeMapper::clear()
{
    octree_->clear();
    log_odds_map_.clear();
    invalidate_cache();
}

double OctreeMapper::get_resolution() const
{
    return resolution_;
}

size_t OctreeMapper::get_num_nodes() const
{
    update_cache();
    return cached_num_nodes_;
}

void OctreeMapper::set_probability_params(double prob_hit, double prob_miss)
{
    prob_hit_ = prob_hit;
    prob_miss_ = prob_miss;
    
    octree_->setProbHit(prob_hit_);
    octree_->setProbMiss(prob_miss_);
}

void OctreeMapper::set_occupancy_thresholds(double min_thresh, double max_thresh)
{
    prob_thres_min_ = min_thresh;
    prob_thres_max_ = max_thresh;
    
    octree_->setOccupancyThres(max_thresh);
    octree_->setClampingThresMin(min_thresh);
    octree_->setClampingThresMax(max_thresh);
}

bool OctreeMapper::save_to_file(const std::string& filename) const
{
    return octree_->writeBinary(filename);
}

bool OctreeMapper::load_from_file(const std::string& filename)
{
    bool success = octree_->readBinary(filename);
    if (success) {
        invalidate_cache();
    }
    return success;
}

void OctreeMapper::invalidate_cache()
{
    cache_valid_ = false;
}

void OctreeMapper::update_cache() const
{
    if (!cache_valid_) {
        cached_num_nodes_ = octree_->calcNumNodes();
        cache_valid_ = true;
    }
}

}  // namespace sonar_3d_reconstruction