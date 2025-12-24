/**
 * @file voxel_storage.h
 * @brief Abstract interface for voxel data storage
 *
 * Provides a common interface for storing and retrieving voxel data
 * (log-odds, observation counts) with persistence support.
 */

#ifndef SONAR_3D_RECONSTRUCTION__VOXEL_STORAGE_H_
#define SONAR_3D_RECONSTRUCTION__VOXEL_STORAGE_H_

#include <octomap/octomap.h>
#include <vector>
#include <string>

namespace sonar_3d_reconstruction
{

// Forward declarations
struct OccupiedVoxel;
struct IWLOMeta;

/**
 * @class VoxelStorage
 * @brief Abstract interface for voxel storage backends
 *
 * This interface defines the contract for storing voxel data including:
 * - Log-odds values for occupancy probability
 * - Observation counts for learning rate decay
 * - Persistence (save/load)
 *
 * Implementations include OctreeStorage (OctoMap-based).
 */
class VoxelStorage
{
public:
    virtual ~VoxelStorage() = default;

    // =========================================================================
    // Core Operations
    // =========================================================================

    /**
     * @brief Get log-odds value for a voxel
     * @param key OcTree key identifying the voxel
     * @return Log-odds value (0.0 if not found)
     */
    virtual double get_log_odds(const octomap::OcTreeKey& key) const = 0;

    /**
     * @brief Set log-odds value for a voxel
     * @param key OcTree key identifying the voxel
     * @param value Log-odds value to set
     */
    virtual void set_log_odds(const octomap::OcTreeKey& key, double value) = 0;

    /**
     * @brief Get observation count for a voxel
     * @param key OcTree key identifying the voxel
     * @return Observation count (0 if not found)
     */
    virtual int get_observation_count(const octomap::OcTreeKey& key) const = 0;

    /**
     * @brief Increment observation count for a voxel
     * @param key OcTree key identifying the voxel
     * @return New observation count after increment
     */
    virtual int increment_observation_count(const octomap::OcTreeKey& key) = 0;

    /**
     * @brief Get or create IWLO metadata for a voxel
     * @param key OcTree key identifying the voxel
     * @return Reference to IWLOMeta (created with defaults if not exists)
     */
    virtual IWLOMeta& get_or_create_meta(const octomap::OcTreeKey& key) = 0;

    // =========================================================================
    // Coordinate Operations
    // =========================================================================

    /**
     * @brief Convert world coordinates to OcTree key
     * @param point World coordinates
     * @return OcTree key
     */
    virtual octomap::OcTreeKey coord_to_key(const octomap::point3d& point) const = 0;

    /**
     * @brief Convert OcTree key to world coordinates (center of voxel)
     * @param key OcTree key
     * @return World coordinates
     */
    virtual octomap::point3d key_to_coord(const octomap::OcTreeKey& key) const = 0;

    // =========================================================================
    // Query Operations
    // =========================================================================

    /**
     * @brief Get all occupied voxels above probability threshold
     * @param min_probability Minimum probability threshold
     * @return Vector of occupied voxels with coordinates and probabilities
     */
    virtual std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const = 0;

    /**
     * @brief Get number of stored voxels
     * @return Number of voxels with metadata
     */
    virtual size_t get_num_voxels() const = 0;

    /**
     * @brief Check if storage has any occupied voxels
     * @param min_probability Probability threshold
     * @return True if at least one voxel exceeds threshold
     */
    virtual bool has_occupied_voxels(double min_probability = 0.5) const = 0;

    // =========================================================================
    // Persistence Operations
    // =========================================================================

    /**
     * @brief Save storage to directory
     * @param path Directory path
     * @return True if successful
     */
    virtual bool save(const std::string& path) = 0;

    /**
     * @brief Load storage from directory
     * @param path Directory path
     * @return True if successful
     */
    virtual bool load(const std::string& path) = 0;

    // =========================================================================
    // Maintenance Operations
    // =========================================================================

    /**
     * @brief Clear all stored data
     */
    virtual void clear() = 0;

    /**
     * @brief Sync metadata to underlying OcTree (for visualization)
     */
    virtual void sync_to_octree() = 0;

    /**
     * @brief Prune octree (merge homogeneous nodes)
     * @return Number of nodes pruned
     */
    virtual size_t prune() = 0;

    /**
     * @brief Get memory usage estimate in bytes
     */
    virtual size_t get_memory_usage() const = 0;

    // =========================================================================
    // OcTree Access (for visualization/merging)
    // =========================================================================

    /**
     * @brief Get underlying OcTree pointer
     * @return Pointer to OcTree (may be null)
     */
    virtual octomap::OcTree* get_octree() = 0;
    virtual const octomap::OcTree* get_octree() const = 0;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__VOXEL_STORAGE_H_
