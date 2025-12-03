#ifndef SONAR_3D_RECONSTRUCTION__PROBABILITY_UPDATER_H_
#define SONAR_3D_RECONSTRUCTION__PROBABILITY_UPDATER_H_

#include "octree_mapper.h"
#include <Eigen/Dense>
#include <vector>

namespace sonar_3d_reconstruction
{

/**
 * High-level probabilistic updater that wraps OctreeMapper
 * Provides Python-friendly interface for sonar data processing
 */
class ProbabilityUpdater
{
public:
    /**
     * Constructor
     * @param resolution Voxel resolution in meters
     */
    explicit ProbabilityUpdater(double resolution = 0.05);
    
    ~ProbabilityUpdater();
    
    /**
     * Set log-odds parameters for probability updates
     * @param log_odds_occupied Log-odds increment for occupied voxels
     * @param log_odds_free Log-odds decrement for free voxels
     */
    void set_log_odds_params(double log_odds_occupied, double log_odds_free);
    
    /**
     * Set adaptive update parameters
     * @param adaptive_enabled Enable adaptive updating
     * @param adaptive_threshold Threshold for adaptive behavior
     * @param adaptive_max_ratio Maximum update ratio for adaptive behavior
     */
    void set_adaptive_params(bool adaptive_enabled, double adaptive_threshold, double adaptive_max_ratio);
    
    /**
     * Set probability clamping thresholds
     * @param min_prob Minimum probability (maps to log-odds minimum)
     * @param max_prob Maximum probability (maps to log-odds maximum)
     */
    void set_clamping_thresholds(double min_prob, double max_prob);
    
    /**
     * Batch update multiple points with adaptive probability updates
     * @param points Nx3 matrix of point coordinates
     * @param log_odds_updates N-vector of log-odds updates for each point
     * @param is_occupied N-vector of boolean flags for occupied vs free
     */
    void batch_update(const Eigen::MatrixXd& points, 
                     const Eigen::VectorXd& log_odds_updates,
                     const std::vector<bool>& is_occupied);
    
    /**
     * Get all occupied voxels above probability threshold
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5) const;
    
    /**
     * Get memory usage statistics
     * @return MemoryStats structure
     */
    MemoryStats get_memory_usage() const;
    
    /**
     * Get number of nodes in octree
     * @return Total number of nodes
     */
    size_t get_num_nodes() const;
    
    /**
     * Get octree resolution
     * @return Voxel resolution in meters
     */
    double get_resolution() const;
    
    /**
     * Prune unnecessary nodes to reduce memory usage
     * @return Number of nodes removed
     */
    size_t prune_tree();
    
    /**
     * Clear all data from octree
     */
    void clear();

    /**
     * Set thresholds for weighted average probability update
     * @param occupied_thresh Occupied threshold (default: 0.7)
     * @param free_thresh Free threshold (default: 0.3)
     */
    void set_thresholds(double occupied_thresh, double free_thresh);

    /**
     * Set intensity normalization parameters
     * @param intensity_threshold Minimum intensity to consider (default: 35.0)
     * @param intensity_max Maximum intensity value (default: 255.0)
     */
    void set_intensity_params(double intensity_threshold, double intensity_max);

    /**
     * Batch update using weighted average method (voxelmap_fusion style)
     * @param points Nx3 matrix of point coordinates
     * @param intensities N-vector of intensity values
     * @param is_occupied N-vector of boolean flags (currently unused but kept for interface compatibility)
     */
    void batch_update_weighted_average(
        const Eigen::MatrixXd& points,
        const Eigen::VectorXd& intensities,
        const std::vector<bool>& is_occupied
    );

    /**
     * Get observation count for a voxel
     * @param key Voxel key
     * @return Observation count
     */
    int get_observation_count(const std::string& key) const;

private:
    std::unique_ptr<OctreeMapper> octree_mapper_;

    // Alternative: Direct log-odds storage (like Python SimpleOctree)
    std::unordered_map<std::string, double> voxels_log_odds_;  // key -> log_odds
    double resolution_;

    // Log-odds parameters
    double log_odds_occupied_;
    double log_odds_free_;

    // Adaptive update parameters
    bool adaptive_enabled_;
    double adaptive_threshold_;
    double adaptive_max_ratio_;

    // Clamping thresholds
    double min_probability_;
    double max_probability_;

    // Weighted average parameters
    std::unordered_map<std::string, int> observation_counts_;  // key -> observation count
    double occupied_threshold_;  // Occupied threshold (0.7)
    double free_threshold_;  // Free threshold (0.3)
    double intensity_max_;  // Maximum intensity value (255.0)
    double intensity_threshold_;  // Minimum intensity threshold (35.0)
    
    /**
     * Convert log-odds to probability
     * @param log_odds Log-odds value
     * @return Probability [0, 1]
     */
    double log_odds_to_probability(double log_odds) const;
    
    /**
     * Convert probability to log-odds
     * @param probability Probability [0, 1]
     * @return Log-odds value
     */
    double probability_to_log_odds(double probability) const;
    
    /**
     * Apply adaptive scaling to log-odds update
     * @param current_prob Current probability of voxel
     * @param log_odds_update Original log-odds update
     * @param is_occupied Whether this is an occupied update
     * @return Scaled log-odds update
     */
    double apply_adaptive_scaling(double current_prob, double log_odds_update, bool is_occupied) const;
    
    /**
     * Convert world coordinates to voxel key (same as Python SimpleOctree)
     * @param x, y, z World coordinates
     * @return String key for hash map
     */
    std::string world_to_key(double x, double y, double z) const;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__PROBABILITY_UPDATER_H_