#include "outofcore_tile_mapper.h"
#include <iostream>
#include <algorithm>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <limits>
#include <cmath>

namespace sonar_3d_reconstruction
{

// RAII class to suppress stdout/stderr at file descriptor level
class SuppressOutput {
public:
    SuppressOutput() {
        std::cout.flush();
        std::cerr.flush();
        fflush(stdout);
        fflush(stderr);
        stdout_fd_ = dup(fileno(stdout));
        stderr_fd_ = dup(fileno(stderr));
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
    }
    ~SuppressOutput() {
        fflush(stdout);
        fflush(stderr);
        dup2(stdout_fd_, fileno(stdout));
        dup2(stderr_fd_, fileno(stderr));
        close(stdout_fd_);
        close(stderr_fd_);
    }
private:
    int stdout_fd_;
    int stderr_fd_;
};

OutofcoreTileMapper::OutofcoreTileMapper(const std::string& map_path,
                                         double resolution,
                                         double tile_size,
                                         size_t cache_size)
    : map_path_(map_path)
    , resolution_(resolution)
    , tile_size_(tile_size)
    , cache_size_(cache_size)
    , tile_manager_(map_path, tile_size, resolution)
    , tile_cache_(cache_size, [this](const TileIndex& idx, std::unique_ptr<Tile>& tile) {
        this->on_tile_eviction(idx, tile);
    })
    , occupied_threshold_(0.7)  // Default from config
{
    // Initialize tile manager
    if (!tile_manager_.initialize()) {
        std::cerr << "[OutofcoreTileMapper] Warning: Failed to initialize tile manager" << std::endl;
    }
}

OutofcoreTileMapper::~OutofcoreTileMapper()
{
    // Flush all tiles before destruction
    try {
        flush_all();
    } catch (const std::exception& e) {
        std::cerr << "[OutofcoreTileMapper] Error during destruction: " << e.what() << std::endl;
    }
}

void OutofcoreTileMapper::batch_update_iwlo(const Eigen::MatrixXd& points,
                                            const Eigen::VectorXd& intensities,
                                            const std::vector<bool>& is_occupied)
{
    if (points.rows() != intensities.rows() ||
        points.rows() != static_cast<int>(is_occupied.size())) {
        throw std::invalid_argument("batch_update_iwlo: size mismatch");
    }

    if (points.cols() != 3) {
        throw std::invalid_argument("batch_update_iwlo: points must be Nx3");
    }

    // Group points by tile
    auto groups = group_points_by_tile(points);

    // Update each tile
    for (const auto& [tile_idx, point_indices] : groups) {
        // Get or load tile
        Tile* tile = get_or_load_tile(tile_idx);

        // Prepare batch data for this tile
        Eigen::MatrixXd tile_points(point_indices.size(), 3);
        Eigen::VectorXd tile_intensities(point_indices.size());
        std::vector<bool> tile_occupied(point_indices.size());

        for (size_t i = 0; i < point_indices.size(); ++i) {
            int idx = point_indices[i];
            tile_points(i, 0) = points(idx, 0);
            tile_points(i, 1) = points(idx, 1);
            tile_points(i, 2) = points(idx, 2);
            tile_intensities(i) = intensities(idx);
            tile_occupied[i] = is_occupied[idx];
        }

        // Update tile
        tile->batch_update(tile_points, tile_intensities, tile_occupied, iwlo_params_);

        // Update bounds
        tile_manager_.update_bounds(tile_idx);
    }
    // Saving is handled by periodic timer in 3d_mapper_node.py
}

void OutofcoreTileMapper::set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                                          double L_min, double L_max)
{
    iwlo_params_.sharpness = sharpness;
    iwlo_params_.decay_rate = decay_rate;
    iwlo_params_.min_alpha = min_alpha;
    iwlo_params_.L_min = L_min;
    iwlo_params_.L_max = L_max;
}

void OutofcoreTileMapper::set_log_odds_params(double log_odds_occupied, double log_odds_free)
{
    iwlo_params_.log_odds_occupied = log_odds_occupied;
    iwlo_params_.log_odds_free = log_odds_free;
}

void OutofcoreTileMapper::set_intensity_params(double intensity_threshold, double intensity_max)
{
    iwlo_params_.intensity_threshold = intensity_threshold;
    iwlo_params_.intensity_max = intensity_max;
}

void OutofcoreTileMapper::set_occupied_threshold(double threshold)
{
    occupied_threshold_ = threshold;
}

void OutofcoreTileMapper::set_adaptive_params(bool enabled, double threshold, double max_ratio)
{
    iwlo_params_.adaptive_enabled = enabled;
    iwlo_params_.adaptive_threshold = threshold;
    iwlo_params_.adaptive_max_ratio = max_ratio;
}

Eigen::MatrixXd OutofcoreTileMapper::get_occupied_voxels(double min_probability)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    std::vector<OccupiedVoxel> all_voxels;

    // Collect from all cached tiles
    tile_cache_.for_each([&](const TileIndex&, std::unique_ptr<Tile>& tile) {
        if (tile) {
            auto voxels = tile->get_occupied_voxels(min_probability);
            all_voxels.insert(all_voxels.end(), voxels.begin(), voxels.end());
        }
    });

    // Convert to matrix
    if (all_voxels.empty()) {
        return Eigen::MatrixXd(0, 4);
    }

    Eigen::MatrixXd result(all_voxels.size(), 4);
    for (size_t i = 0; i < all_voxels.size(); ++i) {
        result(i, 0) = all_voxels[i].x;
        result(i, 1) = all_voxels[i].y;
        result(i, 2) = all_voxels[i].z;
        result(i, 3) = all_voxels[i].probability;
    }

    return result;
}

Eigen::MatrixXd OutofcoreTileMapper::get_all_occupied_voxels(double min_probability)
{
    // Get ALL tiles from disk (not just cached)
    auto tile_indices = tile_manager_.list_all_tiles();

    std::vector<OccupiedVoxel> all_voxels;

    // Load and query each tile
    for (const auto& idx : tile_indices) {
        Tile* tile = get_or_load_tile(idx);
        if (tile) {
            auto voxels = tile->get_occupied_voxels(min_probability);
            all_voxels.insert(all_voxels.end(), voxels.begin(), voxels.end());
        }
    }

    // Convert to matrix
    if (all_voxels.empty()) {
        return Eigen::MatrixXd(0, 4);
    }

    Eigen::MatrixXd result(all_voxels.size(), 4);
    for (size_t i = 0; i < all_voxels.size(); ++i) {
        result(i, 0) = all_voxels[i].x;
        result(i, 1) = all_voxels[i].y;
        result(i, 2) = all_voxels[i].z;
        result(i, 3) = all_voxels[i].probability;
    }

    return result;
}

Eigen::MatrixXd OutofcoreTileMapper::get_occupied_voxels_in_region(
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    double min_probability)
{
    // Get tiles in region
    auto tile_indices = tile_manager_.get_tiles_in_region(min_bound, max_bound);

    std::vector<OccupiedVoxel> all_voxels;

    // Load and query each tile
    for (const auto& idx : tile_indices) {
        Tile* tile = get_or_load_tile(idx);
        auto voxels = tile->get_occupied_voxels(min_probability);

        // Filter voxels within bounds
        for (const auto& v : voxels) {
            if (v.x >= min_bound.x() && v.x <= max_bound.x() &&
                v.y >= min_bound.y() && v.y <= max_bound.y() &&
                v.z >= min_bound.z() && v.z <= max_bound.z()) {
                all_voxels.push_back(v);
            }
        }
    }

    // Convert to matrix
    if (all_voxels.empty()) {
        return Eigen::MatrixXd(0, 4);
    }

    Eigen::MatrixXd result(all_voxels.size(), 4);
    for (size_t i = 0; i < all_voxels.size(); ++i) {
        result(i, 0) = all_voxels[i].x;
        result(i, 1) = all_voxels[i].y;
        result(i, 2) = all_voxels[i].z;
        result(i, 3) = all_voxels[i].probability;
    }

    return result;
}

MemoryStats OutofcoreTileMapper::get_memory_usage() const
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    size_t total_nodes = 0;
    size_t total_voxels = 0;
    double memory_mb = 0.0;

    tile_cache_.for_each([&](const TileIndex&, const std::unique_ptr<Tile>& tile) {
        if (tile) {
            total_voxels += tile->get_num_voxels();
            memory_mb += tile->get_memory_usage() / (1024.0 * 1024.0);

            if (tile->get_octree()) {
                total_nodes += tile->get_octree()->calcNumNodes();
            }
        }
    });

    double efficiency = (total_nodes > 0) ? static_cast<double>(total_voxels) / total_nodes : 0.0;

    return MemoryStats(total_nodes, total_voxels, memory_mb, efficiency);
}

size_t OutofcoreTileMapper::get_num_nodes() const
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    size_t total_nodes = 0;
    tile_cache_.for_each([&](const TileIndex&, const std::unique_ptr<Tile>& tile) {
        if (tile && tile->get_octree()) {
            total_nodes += tile->get_octree()->calcNumNodes();
        }
    });

    return total_nodes;
}

void OutofcoreTileMapper::clear()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // Clear cache without saving
    tile_cache_.clear(false);

    // Delete all tiles from disk
    auto tiles = tile_manager_.list_all_tiles();
    for (const auto& idx : tiles) {
        tile_manager_.delete_tile(idx);
    }

    // Save updated metadata
    tile_manager_.save_metadata();
}

void OutofcoreTileMapper::flush_all()
{
    flush_and_get_dirty_tiles();  // Ignore returned indices
}

std::vector<TileIndex> OutofcoreTileMapper::flush_and_get_dirty_tiles()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);
    std::vector<TileIndex> flushed_indices;

    tile_cache_.for_each([this, &flushed_indices](const TileIndex& idx, std::unique_ptr<Tile>& tile) {
        if (tile && tile->is_dirty()) {
            std::string tile_dir = tile_manager_.get_tile_directory(idx);
            if (tile->save(tile_dir)) {
                flushed_indices.push_back(idx);
            } else {
                std::cerr << "[OutofcoreTileMapper] Failed to save tile " << idx.to_string() << std::endl;
            }
        }
    });

    // Save metadata
    tile_manager_.save_metadata();

    return flushed_indices;
}

bool OutofcoreTileMapper::flush_tile(const TileIndex& idx)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    auto* tile_ptr = tile_cache_.get(idx);
    if (!tile_ptr || !*tile_ptr) {
        return true;  // Tile not in cache
    }

    if ((*tile_ptr)->is_dirty()) {
        std::string tile_dir = tile_manager_.get_tile_directory(idx);
        return (*tile_ptr)->save(tile_dir);
    }

    return true;
}

std::unique_ptr<octomap::OcTree> OutofcoreTileMapper::get_merged_octree()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    auto merged = std::make_unique<octomap::OcTree>(resolution_);

    tile_cache_.for_each([&](const TileIndex&, std::unique_ptr<Tile>& tile) {
        if (tile && tile->get_octree()) {
            // Iterate through leaf nodes and insert into merged tree
            for (auto it = tile->get_octree()->begin_leafs();
                 it != tile->get_octree()->end_leafs(); ++it) {
                merged->updateNode(it.getCoordinate(), it->getLogOdds() > 0);
            }
        }
    });

    return merged;
}

std::unique_ptr<octomap::OcTree> OutofcoreTileMapper::get_full_merged_octree(double min_probability)
{
    auto merged = std::make_unique<octomap::OcTree>(resolution_);

    // Get all tiles
    auto tile_indices = tile_manager_.list_all_tiles();

    size_t total_voxels = 0;

    for (const auto& idx : tile_indices) {
        // Load tile
        Tile* tile = get_or_load_tile(idx);

        if (tile) {
            // Use tile's get_occupied_voxels which filters using iwlo_meta_
            // This ensures consistency with pointcloud output
            auto voxels = tile->get_occupied_voxels(min_probability);
            total_voxels += voxels.size();

            // Add each voxel to merged octree
            for (const auto& v : voxels) {
                octomap::point3d pt(v.x, v.y, v.z);
                octomap::OcTreeNode* node = merged->updateNode(pt, true, true);
                if (node) {
                    // Convert probability back to log-odds
                    double log_odds = std::log(v.probability / (1.0 - v.probability));
                    node->setLogOdds(log_odds);
                }
            }
        }
    }

    // Prune identical adjacent nodes to optimize tree structure
    merged->prune();

    // Update inner occupancy for proper visualization
    merged->updateInnerOccupancy();

    return merged;
}

bool OutofcoreTileMapper::save_merged_octree(const std::string& filepath)
{
    auto merged = get_full_merged_octree();
    SuppressOutput suppress;
    return merged->writeBinary(filepath);
}

std::pair<std::vector<int8_t>, std::string> OutofcoreTileMapper::get_octree_binary(double min_probability)
{
    // Get merged octree (debug output happens inside)
    auto merged = get_full_merged_octree(min_probability);
    if (!merged || merged->size() == 0) {
        return {{}, ""};
    }

    // Suppress stdout/stderr only during serialization
    SuppressOutput suppress;

    // Serialize to stringstream
    std::stringstream ss;
    merged->writeBinaryData(ss);

    // Convert to vector<int8_t>
    std::string str = ss.str();
    std::vector<int8_t> data(str.begin(), str.end());

    return {data, merged->getTreeType()};
}

size_t OutofcoreTileMapper::get_total_tile_count() const
{
    return tile_manager_.list_all_tiles().size();
}

std::vector<TileIndex> OutofcoreTileMapper::get_all_tile_indices() const
{
    return tile_manager_.list_all_tiles();
}

size_t OutofcoreTileMapper::get_disk_usage() const
{
    return tile_manager_.get_disk_usage();
}

void OutofcoreTileMapper::preload_region(const Eigen::Vector3d& min_bound,
                                         const Eigen::Vector3d& max_bound)
{
    auto tile_indices = tile_manager_.get_tiles_in_region(min_bound, max_bound);

    for (const auto& idx : tile_indices) {
        if (tile_manager_.tile_exists(idx)) {
            get_or_load_tile(idx);
        }
    }
}

void OutofcoreTileMapper::reload_tiles(const std::vector<TileIndex>& indices)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    for (const auto& idx : indices) {
        // Remove from cache first (force reload from disk)
        tile_cache_.remove(idx);

        // Load from disk
        if (tile_manager_.tile_exists(idx)) {
            auto tile = std::make_unique<Tile>(idx, resolution_, tile_size_);
            std::string tile_dir = tile_manager_.get_tile_directory(idx);
            if (tile->load(tile_dir)) {
                tile_cache_.put(idx, std::move(tile));
            }
        }
    }
}

size_t OutofcoreTileMapper::prune_all()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    size_t total_pruned = 0;

    tile_cache_.for_each([&](const TileIndex&, std::unique_ptr<Tile>& tile) {
        if (tile) {
            total_pruned += tile->prune();
        }
    });

    return total_pruned;
}

Tile* OutofcoreTileMapper::get_or_load_tile(const TileIndex& idx)
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // Check cache first
    auto* tile_ptr = tile_cache_.get(idx);
    if (tile_ptr && *tile_ptr) {
        return tile_ptr->get();
    }

    // Create or load tile
    auto tile = std::make_unique<Tile>(idx, resolution_, tile_size_);

    std::string tile_dir = tile_manager_.get_tile_directory(idx);
    if (tile_manager_.tile_exists(idx)) {
        if (!tile->load(tile_dir)) {
            std::cerr << "[OutofcoreTileMapper] Warning: Failed to load tile " << idx.to_string() << std::endl;
        }
    }

    // Put in cache (may evict LRU)
    Tile* raw_ptr = tile.get();
    tile_cache_.put(idx, std::move(tile));

    return raw_ptr;
}

void OutofcoreTileMapper::on_tile_eviction(const TileIndex& idx, std::unique_ptr<Tile>& tile)
{
    if (tile && tile->is_dirty()) {
        std::string tile_dir = tile_manager_.get_tile_directory(idx);
        if (tile->save(tile_dir)) {
            // Record index on successful save (for visualizer notification)
            std::lock_guard<std::mutex> lock(saved_tiles_mutex_);
            recently_saved_tiles_.push_back(idx);
        }
    }
}

void OutofcoreTileMapper::save_dirty_tiles_early()
{
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // Only execute when cache is not yet full
    // When cache is full, tiles are saved during eviction
    if (tile_cache_.size() >= cache_size_) {
        return;
    }

    std::vector<TileIndex> saved;

    tile_cache_.for_each([&](const TileIndex& idx, std::unique_ptr<Tile>& tile) {
        if (tile && tile->is_dirty()) {
            std::string tile_dir = tile_manager_.get_tile_directory(idx);
            if (tile->save(tile_dir)) {
                tile->mark_clean();
                saved.push_back(idx);
            }
        }
    });

    if (!saved.empty()) {
        std::lock_guard<std::mutex> lock2(saved_tiles_mutex_);
        recently_saved_tiles_.insert(recently_saved_tiles_.end(), saved.begin(), saved.end());
    }
}

std::vector<TileIndex> OutofcoreTileMapper::get_and_clear_saved_tiles()
{
    std::lock_guard<std::mutex> lock(saved_tiles_mutex_);
    std::vector<TileIndex> result = std::move(recently_saved_tiles_);
    recently_saved_tiles_.clear();
    return result;
}

std::unordered_map<TileIndex, std::vector<int>, TileIndexHash>
OutofcoreTileMapper::group_points_by_tile(const Eigen::MatrixXd& points)
{
    std::unordered_map<TileIndex, std::vector<int>, TileIndexHash> groups;

    for (int i = 0; i < points.rows(); ++i) {
        TileIndex idx = tile_manager_.world_to_tile_index(
            points(i, 0), points(i, 1), points(i, 2));
        groups[idx].push_back(i);
    }

    return groups;
}

}  // namespace sonar_3d_reconstruction
