#ifndef SONAR_3D_RECONSTRUCTION__OCTREE_MAPPER_H_
#define SONAR_3D_RECONSTRUCTION__OCTREE_MAPPER_H_

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>

namespace sonar_3d_reconstruction
{

/**
 * Memory usage statistics for octree
 */
struct MemoryStats
{
    size_t num_nodes;
    size_t num_leaf_nodes;
    double memory_mb;
    double memory_efficiency;
    
    MemoryStats() : num_nodes(0), num_leaf_nodes(0), memory_mb(0.0), memory_efficiency(0.0) {}
    MemoryStats(size_t nodes, size_t leaf_nodes, double mem_mb, double efficiency)
        : num_nodes(nodes), num_leaf_nodes(leaf_nodes), memory_mb(mem_mb), memory_efficiency(efficiency) {}
};

/**
 * High-performance OctoMap-based octree mapper for sonar data
 * Supports probabilistic updates, batch processing, and memory management
 */
class OctreeMapper
{
public:
    /**
     * Constructor
     * @param resolution Voxel resolution in meters
     * @param prob_hit Probability value for occupied updates (0.5-1.0)
     * @param prob_miss Probability value for free updates (0.0-0.5)
     * @param prob_thres_min Minimum probability threshold for occupancy
     * @param prob_thres_max Maximum probability threshold for occupancy
     */
    OctreeMapper(double resolution = 0.05,
                 double prob_hit = 0.7,
                 double prob_miss = 0.3,
                 double prob_thres_min = 0.12,
                 double prob_thres_max = 0.97);
    
    ~OctreeMapper();
    
    /**
     * Update single point in octree
     * @param point 3D point coordinates [x, y, z]
     * @param occupied True for occupied, false for free
     */
    void update_point(const Eigen::Vector3d& point, bool occupied);
    
    /**
     * Batch update multiple points (more efficient than individual updates)
     * @param points Nx3 matrix of point coordinates
     * @param occupied_flags Vector of bool flags for each point
     */
    void batch_update(const Eigen::MatrixXd& points, const std::vector<bool>& occupied_flags);
    
    /**
     * Batch update with log-odds values (Python SimpleOctree compatible)
     * @param points Nx3 matrix of point coordinates
     * @param log_odds_updates Vector of log-odds update values
     */
    void batch_update_with_log_odds(const Eigen::MatrixXd& points, const Eigen::VectorXd& log_odds_updates);
    
    /**
     * Insert ray from origin to endpoint (handles free space along ray)
     * @param origin Ray origin point
     * @param endpoint Ray endpoint
     */
    void insert_ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& endpoint);
    
    /**
     * Get all occupied voxels above probability threshold
     * @param min_probability Minimum probability threshold (default: current threshold)
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_occupied_voxels(double min_probability = -1.0) const;
    
    /**
     * Get memory usage statistics
     * @return MemoryStats structure with detailed memory information
     */
    MemoryStats get_memory_usage() const;
    
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
     * Get octree resolution
     * @return Voxel resolution in meters
     */
    double get_resolution() const;
    
    /**
     * Get number of nodes in octree
     * @return Total number of nodes
     */
    size_t get_num_nodes() const;
    
    /**
     * Set probability parameters
     * @param prob_hit Hit probability (0.5-1.0)
     * @param prob_miss Miss probability (0.0-0.5)
     */
    void set_probability_params(double prob_hit, double prob_miss);
    
    /**
     * Set occupancy thresholds
     * @param min_thresh Minimum threshold for free space
     * @param max_thresh Maximum threshold for occupied space
     */
    void set_occupancy_thresholds(double min_thresh, double max_thresh);
    
    /**
     * Save octree to binary file
     * @param filename Output filename
     * @return True if successful
     */
    bool save_to_file(const std::string& filename) const;
    
    /**
     * Load octree from binary file
     * @param filename Input filename
     * @return True if successful
     */
    bool load_from_file(const std::string& filename);

private:
    // Hash function for octomap::point3d
    struct Point3dHash {
        std::size_t operator()(const octomap::point3d& p) const {
            std::size_t h1 = std::hash<float>{}(p.x());
            std::size_t h2 = std::hash<float>{}(p.y());
            std::size_t h3 = std::hash<float>{}(p.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
    
    // Equality function for octomap::point3d
    struct Point3dEqual {
        bool operator()(const octomap::point3d& lhs, const octomap::point3d& rhs) const {
            return (lhs.x() == rhs.x()) && (lhs.y() == rhs.y()) && (lhs.z() == rhs.z());
        }
    };
    
    std::unique_ptr<octomap::OcTree> octree_;
    double resolution_;
    double prob_hit_;
    double prob_miss_;
    double prob_thres_min_;
    double prob_thres_max_;
    
    // Log-odds storage (Python SimpleOctree compatible)
    std::unordered_map<octomap::point3d, double, Point3dHash, Point3dEqual> log_odds_map_;
    double log_odds_min_;
    double log_odds_max_;
    
    mutable size_t cached_num_nodes_;
    mutable bool cache_valid_;
    
    void invalidate_cache();
    void update_cache() const;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__OCTREE_MAPPER_H_