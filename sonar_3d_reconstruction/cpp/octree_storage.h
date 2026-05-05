/**
 * @file octree_storage.h
 * @brief OctoMap-based implementation of VoxelStorage
 *
 * Provides voxel storage using OctoMap's OcTree with additional
 * IWLO metadata tracking (observation counts, log-odds).
 */

#ifndef SONAR_3D_RECONSTRUCTION__OCTREE_STORAGE_H_
#define SONAR_3D_RECONSTRUCTION__OCTREE_STORAGE_H_

#include "voxel_storage.h"
#include "tile.h"  // For IWLOMeta, OccupiedVoxel, OcTreeKeyHash
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace sonar_3d_reconstruction
{

/**
 * @class OctreeStorage
 * @brief OctoMap-based voxel storage implementation
 *
 * Stores voxel data using:
 * - OctoMap OcTree for spatial indexing and visualization
 * - Separate metadata map for IWLO (log-odds, observation counts)
 *
 * The metadata map is the source of truth for probability values;
 * the OcTree is synchronized before visualization/saving.
 */
class OctreeStorage : public VoxelStorage
{
public:
    /**
     * @brief Constructor
     * @param resolution Voxel resolution in meters
     */
    explicit OctreeStorage(double resolution);

    ~OctreeStorage() override = default;

    // Prevent copy
    OctreeStorage(const OctreeStorage&) = delete;
    OctreeStorage& operator=(const OctreeStorage&) = delete;

    // Allow move
    OctreeStorage(OctreeStorage&&) = default;
    OctreeStorage& operator=(OctreeStorage&&) = default;

    // =========================================================================
    // Core Operations (VoxelStorage interface)
    // =========================================================================

    double get_log_odds(const octomap::OcTreeKey& key) const override;
    void set_log_odds(const octomap::OcTreeKey& key, double value) override;
    int get_observation_count(const octomap::OcTreeKey& key) const override;
    int increment_observation_count(const octomap::OcTreeKey& key) override;
    IWLOMeta& get_or_create_meta(const octomap::OcTreeKey& key) override;

    // =========================================================================
    // Coordinate Operations
    // =========================================================================

    octomap::OcTreeKey coord_to_key(const octomap::point3d& point) const override;
    octomap::point3d key_to_coord(const octomap::OcTreeKey& key) const override;

    // =========================================================================
    // Query Operations
    // =========================================================================

    std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const override;
    size_t get_num_voxels() const override;
    bool has_occupied_voxels(double min_probability = 0.5) const override;

    // =========================================================================
    // Persistence Operations
    // =========================================================================

    bool save(const std::string& path) override;
    bool load(const std::string& path) override;

    // =========================================================================
    // Maintenance Operations
    // =========================================================================

    void clear() override;
    void sync_to_octree() override;
    size_t prune() override;
    size_t get_memory_usage() const override;

    // =========================================================================
    // OcTree Access
    // =========================================================================

    octomap::OcTree* get_octree() override { return octree_.get(); }
    const octomap::OcTree* get_octree() const override { return octree_.get(); }

    // =========================================================================
    // Additional Methods
    // =========================================================================

    /**
     * @brief Get resolution
     */
    double get_resolution() const { return resolution_; }

    /**
     * @brief Check if dirty (modified since last save)
     */
    bool is_dirty() const { return dirty_; }

    /**
     * @brief Mark as clean
     */
    void mark_clean() { dirty_ = false; }

    /**
     * @brief Mark as dirty
     */
    void mark_dirty() { dirty_ = true; }

private:
    double resolution_;
    std::unique_ptr<octomap::OcTree> octree_;
    std::unordered_map<octomap::OcTreeKey, IWLOMeta, OcTreeKeyHash> iwlo_meta_;
    /// Keys with metadata changed since last sync_to_octree (P2-5).
    /// `sync_to_octree` re-applies only these to the octree, then clears.
    std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash> dirty_keys_;
    bool dirty_ = false;

    /**
     * @brief Save IWLO metadata to binary file
     */
    bool save_iwlo_meta(const std::string& filepath);

    /**
     * @brief Load IWLO metadata from binary file
     */
    bool load_iwlo_meta(const std::string& filepath);

    /**
     * @brief Convert log-odds to probability
     */
    static double log_odds_to_probability(double log_odds);
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__OCTREE_STORAGE_H_
