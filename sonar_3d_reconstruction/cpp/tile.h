#ifndef SONAR_3D_RECONSTRUCTION__TILE_H_
#define SONAR_3D_RECONSTRUCTION__TILE_H_

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

namespace sonar_3d_reconstruction
{

/**
 * IWLO metadata for each voxel
 */
struct IWLOMeta
{
    double log_odds;
    int observation_count;

    IWLOMeta() : log_odds(0.0), observation_count(0) {}
    IWLOMeta(double lo, int count) : log_odds(lo), observation_count(count) {}
};

/**
 * IWLO algorithm parameters
 */
struct IWLOParams
{
    double log_odds_occupied;
    double log_odds_free;
    double sharpness;
    double decay_rate;
    double min_alpha;
    double L_min;
    double L_max;
    double intensity_threshold;
    double intensity_max;
    double adaptive_threshold;
    double adaptive_max_ratio;
    bool adaptive_enabled;

    IWLOParams()
        : log_odds_occupied(3.5)
        , log_odds_free(-3.0)
        , sharpness(0.1)
        , decay_rate(0.1)
        , min_alpha(0.3)
        , L_min(-10.0)
        , L_max(10.0)
        , intensity_threshold(35.0)
        , intensity_max(255.0)
        , adaptive_threshold(0.5)
        , adaptive_max_ratio(0.3)
        , adaptive_enabled(true)
    {}
};

/**
 * Tile index (spatial coordinate in tile grid)
 */
struct TileIndex
{
    int x, y, z;

    TileIndex() : x(0), y(0), z(0) {}
    TileIndex(int ix, int iy, int iz) : x(ix), y(iy), z(iz) {}

    bool operator==(const TileIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const TileIndex& other) const {
        return !(*this == other);
    }

    std::string to_string() const {
        return std::to_string(x) + "_" + std::to_string(y) + "_" + std::to_string(z);
    }
};

/**
 * Hash function for TileIndex
 */
struct TileIndexHash
{
    std::size_t operator()(const TileIndex& idx) const {
        std::size_t h1 = std::hash<int>{}(idx.x);
        std::size_t h2 = std::hash<int>{}(idx.y);
        std::size_t h3 = std::hash<int>{}(idx.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * Occupied voxel data for return
 */
struct OccupiedVoxel
{
    double x, y, z;
    double probability;

    OccupiedVoxel(double px, double py, double pz, double prob)
        : x(px), y(py), z(pz), probability(prob) {}
};

/**
 * Hash function for OcTreeKey
 */
struct OcTreeKeyHash
{
    std::size_t operator()(const octomap::OcTreeKey& key) const {
        std::size_t h1 = std::hash<unsigned short>{}(key[0]);
        std::size_t h2 = std::hash<unsigned short>{}(key[1]);
        std::size_t h3 = std::hash<unsigned short>{}(key[2]);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * Tile: A spatial region containing an Octree and IWLO metadata
 * Each tile represents a 10m x 10m x 10m region (configurable)
 */
class Tile
{
public:
    /**
     * Constructor
     * @param index Tile index in grid
     * @param resolution Voxel resolution in meters
     * @param tile_size Tile size in meters
     */
    Tile(const TileIndex& index, double resolution, double tile_size);

    ~Tile();

    // Prevent copy (octree is not copyable)
    Tile(const Tile&) = delete;
    Tile& operator=(const Tile&) = delete;

    // Allow move
    Tile(Tile&&) = default;
    Tile& operator=(Tile&&) = default;

    /**
     * Update single voxel with IWLO algorithm
     * @param point World coordinates
     * @param intensity Sonar intensity value
     * @param is_occupied Whether this is an occupied observation
     * @param params IWLO parameters
     */
    void update_voxel(const octomap::point3d& point,
                      double intensity,
                      bool is_occupied,
                      const IWLOParams& params);

    /**
     * Batch update multiple voxels
     * @param points Nx3 matrix of world coordinates
     * @param intensities N-vector of intensity values
     * @param is_occupied N-vector of occupation flags
     * @param params IWLO parameters
     */
    void batch_update(const Eigen::MatrixXd& points,
                      const Eigen::VectorXd& intensities,
                      const std::vector<bool>& is_occupied,
                      const IWLOParams& params);

    /**
     * Save tile to directory
     * @param tile_dir Directory path for this tile
     * @return True if successful
     */
    bool save(const std::string& tile_dir);

    /**
     * Load tile from directory
     * @param tile_dir Directory path for this tile
     * @return True if successful
     */
    bool load(const std::string& tile_dir);

    /**
     * Get all occupied voxels above threshold
     * @param min_probability Minimum probability threshold
     * @return Vector of occupied voxels
     */
    std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const;

    /**
     * Get Eigen matrix of occupied voxels [x, y, z, probability]
     * @param min_probability Minimum probability threshold
     * @return Nx4 matrix
     */
    Eigen::MatrixXd get_occupied_voxels_matrix(double min_probability = 0.5) const;

    /**
     * Get internal OcTree (for visualization/merging)
     * @return Pointer to OcTree (may be null if empty)
     */
    octomap::OcTree* get_octree() { return octree_.get(); }
    const octomap::OcTree* get_octree() const { return octree_.get(); }

    /**
     * Check if tile has been modified since last save
     */
    bool is_dirty() const { return dirty_; }

    /**
     * Mark tile as clean (after saving)
     */
    void mark_clean() { dirty_ = false; }

    /**
     * Mark tile as dirty (after modification)
     */
    void mark_dirty() { dirty_ = true; }

    /**
     * Get tile index
     */
    const TileIndex& get_index() const { return index_; }

    /**
     * Get number of voxels in tile
     */
    size_t get_num_voxels() const { return iwlo_meta_.size(); }

    /**
     * Check if tile has any occupied voxels (for save filtering)
     * @param occupied_threshold Probability threshold (default 0.5)
     * @return True if at least one voxel is occupied
     */
    bool has_occupied_voxels(double occupied_threshold = 0.5) const;

    /**
     * Get tile bounds (min corner)
     */
    Eigen::Vector3d get_min_bound() const;

    /**
     * Get tile bounds (max corner)
     */
    Eigen::Vector3d get_max_bound() const;

    /**
     * Check if point is within this tile
     */
    bool contains(const octomap::point3d& point) const;

    /**
     * Clear all data
     */
    void clear();

    /**
     * Prune octree (merge homogeneous nodes)
     * @return Number of nodes pruned
     */
    size_t prune();

    /**
     * Get memory usage estimate in bytes
     */
    size_t get_memory_usage() const;

private:
    TileIndex index_;
    double resolution_;
    double tile_size_;
    std::unique_ptr<octomap::OcTree> octree_;
    std::unordered_map<octomap::OcTreeKey, IWLOMeta, OcTreeKeyHash> iwlo_meta_;
    bool dirty_;

    /**
     * Convert log-odds to probability
     */
    static double log_odds_to_probability(double log_odds);

    /**
     * Convert intensity to weight (sigmoid function)
     */
    static double intensity_to_weight(double intensity, const IWLOParams& params);

    /**
     * Compute learning rate alpha
     */
    static double compute_alpha(int observation_count, const IWLOParams& params);

    /**
     * Save IWLO metadata to binary file
     */
    bool save_iwlo_meta(const std::string& filepath);

    /**
     * Load IWLO metadata from binary file
     */
    bool load_iwlo_meta(const std::string& filepath);

    /**
     * Sync IWLO metadata to OcTree (for visualization)
     */
    void sync_to_octree();
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__TILE_H_
