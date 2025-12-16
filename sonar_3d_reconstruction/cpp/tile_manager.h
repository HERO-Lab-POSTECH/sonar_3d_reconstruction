#ifndef SONAR_3D_RECONSTRUCTION__TILE_MANAGER_H_
#define SONAR_3D_RECONSTRUCTION__TILE_MANAGER_H_

#include "tile.h"
#include <string>
#include <vector>
#include <set>

namespace sonar_3d_reconstruction
{

/**
 * Global map metadata
 */
struct MapMetadata
{
    double resolution;      // Voxel resolution in meters
    double tile_size;       // Tile size in meters
    std::string version;    // Format version
    TileIndex min_tile;     // Minimum tile index
    TileIndex max_tile;     // Maximum tile index

    MapMetadata()
        : resolution(0.05)
        , tile_size(10.0)
        , version("1.0")
        , min_tile(0, 0, 0)
        , max_tile(0, 0, 0)
    {}
};

/**
 * TileManager: Manages tile I/O and path operations
 */
class TileManager
{
public:
    /**
     * Constructor
     * @param base_path Base directory for map tiles
     * @param tile_size Tile size in meters
     * @param resolution Voxel resolution in meters
     */
    TileManager(const std::string& base_path, double tile_size, double resolution);

    ~TileManager() = default;

    /**
     * Initialize map directory (create if needed)
     * @return True if successful
     */
    bool initialize();

    /**
     * Convert world coordinates to tile index
     * @param x, y, z World coordinates
     * @return Tile index
     */
    TileIndex world_to_tile_index(double x, double y, double z) const;

    /**
     * Convert world point to tile index
     * @param point World coordinates
     * @return Tile index
     */
    TileIndex world_to_tile_index(const octomap::point3d& point) const;

    /**
     * Get tile directory path
     * @param idx Tile index
     * @return Directory path
     */
    std::string get_tile_directory(const TileIndex& idx) const;

    /**
     * Check if tile exists on disk
     * @param idx Tile index
     * @return True if tile directory exists
     */
    bool tile_exists(const TileIndex& idx) const;

    /**
     * List all existing tiles
     * @return Vector of tile indices
     */
    std::vector<TileIndex> list_all_tiles() const;

    /**
     * Get tiles intersecting a bounding box
     * @param min_bound Minimum corner
     * @param max_bound Maximum corner
     * @return Vector of tile indices
     */
    std::vector<TileIndex> get_tiles_in_region(
        const Eigen::Vector3d& min_bound,
        const Eigen::Vector3d& max_bound) const;

    /**
     * Save global metadata
     * @return True if successful
     */
    bool save_metadata();

    /**
     * Load global metadata
     * @return True if successful
     */
    bool load_metadata();

    /**
     * Update metadata bounds with new tile
     * @param idx Tile index to include
     */
    void update_bounds(const TileIndex& idx);

    /**
     * Get base path
     */
    const std::string& get_base_path() const { return base_path_; }

    /**
     * Get tile size
     */
    double get_tile_size() const { return tile_size_; }

    /**
     * Get resolution
     */
    double get_resolution() const { return resolution_; }

    /**
     * Get metadata
     */
    const MapMetadata& get_metadata() const { return metadata_; }

    /**
     * Delete a tile from disk
     * @param idx Tile index
     * @return True if successful
     */
    bool delete_tile(const TileIndex& idx);

    /**
     * Get total disk usage in bytes
     * @return Total size of all tile files
     */
    size_t get_disk_usage() const;

private:
    std::string base_path_;
    double tile_size_;
    double resolution_;
    MapMetadata metadata_;

    /**
     * Get metadata file path
     */
    std::string get_metadata_path() const;

    /**
     * Parse tile index from directory name
     * @param dirname Directory name (e.g., "tile_1_2_-3")
     * @param idx Output tile index
     * @return True if successfully parsed
     */
    bool parse_tile_dirname(const std::string& dirname, TileIndex& idx) const;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION__TILE_MANAGER_H_
