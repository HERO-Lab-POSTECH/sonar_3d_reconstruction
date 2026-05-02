#ifndef SONAR_3D_RECONSTRUCTION__OUTOFCORE_TILE_MAPPER_H_
#define SONAR_3D_RECONSTRUCTION__OUTOFCORE_TILE_MAPPER_H_

#include "mapper_backend.h"
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
 *
 * Implements IMapperBackend interface (Out-of-Core/Disk backend)
 */
class OutofcoreTileMapper : public IMapperBackend
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
                           const std::vector<bool>& is_occupied) override;

    /**
     * Set IWLO parameters
     */
    void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                         double L_min, double L_max) override;

    /**
     * Set log-odds parameters
     */
    void set_log_odds_params(double log_odds_occupied, double log_odds_free) override;

    /**
     * Set intensity parameters
     */
    void set_intensity_params(double intensity_threshold, double intensity_max) override;

    /**
     * Set occupied threshold for tile save filtering
     */
    void set_occupied_threshold(double threshold) override;

    /**
     * Set adaptive parameters
     */
    void set_adaptive_params(bool enabled, double threshold, double max_ratio) override;

    /**
     * Get all occupied voxels from cached tiles only
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix [x, y, z, probability]
     */
    Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5) override;

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
    MemoryStats get_memory_usage() const override;

    /**
     * Clear all data
     */
    void clear() override;

    /**
     * Get resolution
     */
    double get_resolution() const override { return resolution_; }

    /**
     * Get total number of nodes across all cached tiles
     */
    size_t get_num_nodes() const override;

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
     * @param min_probability Minimum probability threshold (default: include all)
     * @return Unique pointer to merged OcTree
     */
    std::unique_ptr<octomap::OcTree> get_full_merged_octree(double min_probability = -1.0);

    /**
     * Save merged octree to file
     * @param filepath Output file path (.bt)
     * @return True if successful
     */
    bool save_merged_octree(const std::string& filepath);

    /**
     * Get merged octree as binary data (for ROS octomap_msgs)
     * @param min_probability Minimum probability threshold for occupied voxels
     * @return Pair of (binary_data, tree_id) - empty if no data
     */
    std::pair<std::vector<int8_t>, std::string> get_octree_binary(double min_probability = 0.5);

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
     * Get disk usage in bytes (IMapperBackend interface)
     */
    size_t get_disk_usage() const override;

    /**
     * Preload tiles in a region (IMapperBackend interface)
     * @param min_bound Minimum corner
     * @param max_bound Maximum corner
     */
    void preload_region(const Eigen::Vector3d& min_bound,
                        const Eigen::Vector3d& max_bound) override;

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
     * Prune (IMapperBackend interface)
     */
    size_t prune() override { return prune_all(); }

    /**
     * Get IWLO parameters
     */
    const IWLOParams& get_iwlo_params() const { return iwlo_params_; }

    // ============== Ray-casting API ==============

    /**
     * Ray-cast to find depth of first occupied voxel along a direction.
     * Steps along ray from origin, checking each voxel's occupancy.
     * Handles tile boundary crossings automatically.
     *
     * @param origin     3D starting point (world coordinates)
     * @param direction  3D direction vector (will be normalized)
     * @param max_range  Maximum search distance (meters)
     * @param step_size  Step size along ray (meters)
     * @param min_probability Minimum occupancy probability to count as "hit"
     * @return Distance to first occupied voxel, or -1.0 if no hit
     */
    double ray_cast_depth(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& direction,
        double max_range,
        double step_size,
        double min_probability = 0.7);

    /**
     * Batch ray-cast for multiple directions from a single origin.
     * More cache-efficient than calling ray_cast_depth N times.
     *
     * @param origin      3D starting point (world coordinates)
     * @param directions  Nx3 matrix of direction vectors
     * @param max_range   Maximum search distance (meters)
     * @param step_size   Step size along ray (meters)
     * @param min_probability Minimum occupancy probability threshold
     * @return N-element vector of depths (-1.0 for misses)
     */
    Eigen::VectorXd batch_ray_cast_depth(
        const Eigen::Vector3d& origin,
        const Eigen::MatrixXd& directions,
        double max_range,
        double step_size,
        double min_probability = 0.7);

    /**
     * Batch check if points are occupied in the map.
     * Useful for filtering detection voxels that overlap with a reference map.
     *
     * @param points      Nx3 matrix of world coordinates to check
     * @param min_probability Minimum occupancy probability threshold
     * @param tolerance   Number of neighboring voxels to check in each direction.
     *                    0 = exact voxel only, 1 = ±1 voxel (3x3x3), 2 = ±2 (5x5x5)
     * @return N-element vector: 1 = occupied, 0 = free/unknown
     */
    Eigen::VectorXi batch_check_occupied(
        const Eigen::MatrixXd& points,
        double min_probability = 0.7,
        int tolerance = 0);

    // ============== IMapperBackend Optional API ==============

    /**
     * Flush data to persistent storage
     */
    void flush() override { flush_all(); }

    /**
     * Check if backend supports persistent storage
     */
    bool supports_persistence() const override { return true; }

    /**
     * Get backend type
     */
    std::string get_backend_type() const override { return "Disk"; }

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
    std::vector<TileIndex> recently_saved_tiles_;  // Tile indices saved during eviction

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
