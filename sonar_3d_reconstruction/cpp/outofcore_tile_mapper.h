#ifndef SONAR_3D_RECONSTRUCTION__OUTOFCORE_TILE_MAPPER_H_
#define SONAR_3D_RECONSTRUCTION__OUTOFCORE_TILE_MAPPER_H_

#include "tile.h"
#include "tile_manager.h"
#include "lru_cache.h"
#include "octree_mapper.h"  // For MemoryStats
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <mutex>

namespace sonar_3d_reconstruction
{

/**
 * OutofcoreTileMapper: Out-of-core 3D mapping with tile-based storage
 *
 * Features:
 * - Disk-based tile storage (only active tiles in memory)
 * - LRU cache for tile management
 * - IWLO (Intensity-Weighted Log-Odds) algorithm support
 * - API compatible with existing ProbabilityUpdater
 */
class OutofcoreTileMapper
{
public:
    /**
     * Constructor
     * @param map_path Base directory for map tiles
     * @param resolution Voxel resolution in meters
     * @param tile_size Tile size in meters
     * @param cache_size Maximum number of tiles in memory
     */
    OutofcoreTileMapper(const std::string& map_path,
                        double resolution = 0.05,
                        double tile_size = 10.0,
                        size_t cache_size = 16);

    ~OutofcoreTileMapper();

    // Non-copyable
    OutofcoreTileMapper(const OutofcoreTileMapper&) = delete;
    OutofcoreTileMapper& operator=(const OutofcoreTileMapper&) = delete;

    // ============== ProbabilityUpdater API (Compatibility) ==============

    /**
     * Batch update using IWLO algorithm
     * @param points Nx3 matrix of point coordinates
     * @param intensities N-vector of intensity values
     * @param is_occupied N-vector of boolean flags
     */
    void batch_update_iwlo(const Eigen::MatrixXd& points,
                           const Eigen::VectorXd& intensities,
                           const std::vector<bool>& is_occupied);

    /**
     * Set IWLO parameters
     */
    void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                         double L_min, double L_max);

    /**
     * Set log-odds parameters
     */
    void set_log_odds_params(double log_odds_occupied, double log_odds_free);

    /**
     * Set intensity parameters
     */
    void set_intensity_params(double intensity_threshold, double intensity_max);

    /**
     * Set occupied threshold for tile save filtering
     */
    void set_occupied_threshold(double threshold);

    /**
     * Set adaptive parameters
     */
    void set_adaptive_params(bool enabled, double threshold, double max_ratio);

    /**
     * Get all occupied voxels from cached tiles only
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5);

    /**
     * Get ALL occupied voxels from ALL tiles (loads from disk)
     * Use this for visualization to ensure no tiles are missing
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_all_occupied_voxels(double min_probability = 0.5);

    /**
     * Get memory usage statistics
     */
    MemoryStats get_memory_usage() const;

    /**
     * Clear all data
     */
    void clear();

    /**
     * Get resolution
     */
    double get_resolution() const { return resolution_; }

    /**
     * Get total number of nodes across all cached tiles
     */
    size_t get_num_nodes() const;

    // ============== Extended API ==============

    /**
     * Get occupied voxels in a specific region
     * @param min_bound Minimum corner
     * @param max_bound Maximum corner
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_occupied_voxels_in_region(
        const Eigen::Vector3d& min_bound,
        const Eigen::Vector3d& max_bound,
        double min_probability = 0.5);

    /**
     * Flush all dirty tiles to disk
     */
    void flush_all();

    /**
     * Flush all dirty tiles and return their indices
     * @return Vector of tile indices that were flushed
     */
    std::vector<TileIndex> flush_and_get_dirty_tiles();

    /**
     * Flush specific tile to disk
     * @param idx Tile index
     * @return True if successful
     */
    bool flush_tile(const TileIndex& idx);

    /**
     * Get merged OcTree (for visualization)
     * Combines all loaded tiles into a single OcTree
     * @return Unique pointer to merged OcTree
     */
    std::unique_ptr<octomap::OcTree> get_merged_octree();

    /**
     * Get merged OcTree for all tiles (loads from disk)
     * WARNING: May be slow for large maps
     * @return Unique pointer to merged OcTree
     */
    std::unique_ptr<octomap::OcTree> get_full_merged_octree();

    /**
     * Save merged octree to file
     * @param filepath Output file path (.bt)
     * @return True if successful
     */
    bool save_merged_octree(const std::string& filepath);

    /**
     * Get merged octree as binary data (for ROS octomap_msgs)
     * @return Pair of (binary_data, tree_id) - empty if no data
     */
    std::pair<std::vector<int8_t>, std::string> get_octree_binary();

    /**
     * Get number of tiles currently in cache
     */
    size_t get_cached_tile_count() const { return tile_cache_.size(); }

    /**
     * Get total number of tiles on disk
     */
    size_t get_total_tile_count() const;

    /**
     * Get list of all tile indices
     */
    std::vector<TileIndex> get_all_tile_indices() const;

    /**
     * Get disk usage in bytes
     */
    size_t get_disk_usage() const;

    /**
     * Preload tiles in a region
     * @param min_bound Minimum corner
     * @param max_bound Maximum corner
     */
    void preload_region(const Eigen::Vector3d& min_bound,
                        const Eigen::Vector3d& max_bound);

    /**
     * Reload specific tiles from disk (for visualization sync)
     * @param indices Tile indices to reload
     */
    void reload_tiles(const std::vector<TileIndex>& indices);

    /**
     * Prune all cached tiles (merge homogeneous octree nodes)
     * @return Total number of nodes pruned across all tiles
     */
    size_t prune_all();

    /**
     * Get IWLO parameters
     */
    const IWLOParams& get_iwlo_params() const { return iwlo_params_; }

    /**
     * Get and clear recently saved tiles (from eviction)
     * Used to notify visualizer which tiles have been updated
     * @return Vector of tile indices that were saved since last call
     */
    std::vector<TileIndex> get_and_clear_saved_tiles();

private:
    std::string map_path_;
    double resolution_;
    double tile_size_;
    size_t cache_size_;

    TileManager tile_manager_;
    mutable LRUCache<TileIndex, std::unique_ptr<Tile>, TileIndexHash> tile_cache_;
    IWLOParams iwlo_params_;
    double occupied_threshold_;  // Threshold for tile save filtering

    mutable std::mutex cache_mutex_;
    mutable std::mutex saved_tiles_mutex_;
    std::vector<TileIndex> recently_saved_tiles_;  // Eviction 시 저장된 타일 인덱스

    /**
     * Get or load a tile (thread-safe)
     * @param idx Tile index
     * @return Pointer to tile (never null)
     */
    Tile* get_or_load_tile(const TileIndex& idx);

    /**
     * Handle tile eviction (save to disk)
     */
    void on_tile_eviction(const TileIndex& idx, std::unique_ptr<Tile>& tile);

    /**
     * Save all dirty tiles when cache is not full (early visualization)
     * Enables visualization before cache reaches capacity
     */
    void save_dirty_tiles_early();

    /**
     * Group points by tile
     * @return Map from tile index to vector of point indices
     */
    std::unordered_map<TileIndex, std::vector<int>, TileIndexHash>
    group_points_by_tile(const Eigen::MatrixXd& points);
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__OUTOFCORE_TILE_MAPPER_H_
