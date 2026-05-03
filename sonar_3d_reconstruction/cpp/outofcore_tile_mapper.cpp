#include "outofcore_tile_mapper.h"
#include "suppress_output.h"
#include <algorithm>
#include <sstream>
#include <limits>
#include <cmath>

namespace sonar_3d_reconstruction
{

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
    unsigned int max_depth = merged->getTreeDepth();

    // Convert probability to log-odds threshold
    double log_odds_threshold = -std::numeric_limits<double>::infinity();
    if (min_probability > 0.0 && min_probability < 1.0) {
        log_odds_threshold = std::log(min_probability / (1.0 - min_probability));
    }

    // Get all tiles
    auto tile_indices = tile_manager_.list_all_tiles();

    for (const auto& idx : tile_indices) {
        // Load tile
        Tile* tile = get_or_load_tile(idx);

        if (tile && tile->get_octree()) {
            octomap::OcTree* src_tree = tile->get_octree();

            // Iterate through leaf nodes (preserves pruned structure)
            for (auto it = src_tree->begin_leafs(); it != src_tree->end_leafs(); ++it) {
                float log_odds = it->getLogOdds();

                // Skip voxels at or below threshold (consistent with pointcloud's > comparison)
                if (log_odds <= log_odds_threshold) {
                    continue;
                }

                unsigned int node_depth = it.getDepth();
                octomap::point3d center = it.getCoordinate();

                if (node_depth == max_depth) {
                    // Max depth node: copy directly
                    octomap::OcTreeNode* node = merged->updateNode(center, true, true);
                    if (node) {
                        node->setLogOdds(log_odds);
                    }
                } else {
                    // Pruned (larger) node: expand into max-res voxels with same log_odds
                    // They will be merged back by prune() since they have identical values
                    double node_size = it.getSize();
                    double half_size = node_size / 2.0;

                    octomap::point3d min_pt(center.x() - half_size + resolution_/2,
                                           center.y() - half_size + resolution_/2,
                                           center.z() - half_size + resolution_/2);
                    octomap::point3d max_pt(center.x() + half_size - resolution_/2,
                                           center.y() + half_size - resolution_/2,
                                           center.z() + half_size - resolution_/2);

                    for (double x = min_pt.x(); x <= max_pt.x() + resolution_/4; x += resolution_) {
                        for (double y = min_pt.y(); y <= max_pt.y() + resolution_/4; y += resolution_) {
                            for (double z = min_pt.z(); z <= max_pt.z() + resolution_/4; z += resolution_) {
                                octomap::OcTreeNode* node = merged->updateNode(
                                    octomap::point3d(x, y, z), true, true);
                                if (node) {
                                    node->setLogOdds(log_odds);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Prune merged tree
    merged->prune();

    // Update inner occupancy for proper visualization
    merged->updateInnerOccupancy();

    // Debug: count leaves
    size_t leaf_count = 0;
    for (auto it = merged->begin_leafs(); it != merged->end_leafs(); ++it) {
        leaf_count++;
    }
    std::cerr << "[OctoMap] threshold=" << min_probability
              << " log_odds_th=" << log_odds_threshold
              << " leaves=" << leaf_count << std::endl;

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

// ============== Ray-casting API ==============

double OutofcoreTileMapper::ray_cast_depth(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    double max_range,
    double step_size,
    double min_probability)
{
    // Convert probability to log-odds threshold
    double min_log_odds;
    if (min_probability <= 0.0) {
        min_log_odds = -10.0;
    } else if (min_probability >= 1.0) {
        min_log_odds = 10.0;
    } else {
        min_log_odds = std::log(min_probability / (1.0 - min_probability));
    }

    // Normalize direction
    Eigen::Vector3d dir = direction.normalized();
    if (dir.norm() < 1e-10) {
        return -1.0;
    }

    // Track current tile to avoid repeated lookups
    TileIndex current_tile_idx(-999999, -999999, -999999);
    octomap::OcTree* current_octree = nullptr;

    int num_steps = static_cast<int>(std::ceil(max_range / step_size));
    for (int i = 0; i <= num_steps; ++i) {
        double t = i * step_size;
        if (t > max_range) break;

        Eigen::Vector3d point = origin + t * dir;

        // Check tile boundary crossing
        TileIndex idx = tile_manager_.world_to_tile_index(
            point.x(), point.y(), point.z());

        if (idx != current_tile_idx) {
            current_tile_idx = idx;
            if (tile_manager_.tile_exists(idx)) {
                Tile* tile = get_or_load_tile(idx);
                current_octree = (tile) ? tile->get_octree() : nullptr;
            } else {
                current_octree = nullptr;
            }
        }

        if (!current_octree) continue;

        // Query voxel
        octomap::point3d oct_point(point.x(), point.y(), point.z());
        octomap::OcTreeNode* node = current_octree->search(oct_point);
        if (node && node->getLogOdds() > static_cast<float>(min_log_odds)) {
            return t;
        }
    }

    return -1.0;
}

Eigen::VectorXd OutofcoreTileMapper::batch_ray_cast_depth(
    const Eigen::Vector3d& origin,
    const Eigen::MatrixXd& directions,
    double max_range,
    double step_size,
    double min_probability)
{
    if (directions.cols() != 3) {
        throw std::invalid_argument("batch_ray_cast_depth: directions must be Nx3");
    }

    int N = directions.rows();
    Eigen::VectorXd depths(N);

    for (int i = 0; i < N; ++i) {
        Eigen::Vector3d dir = directions.row(i).transpose();
        depths(i) = ray_cast_depth(origin, dir, max_range, step_size, min_probability);
    }

    return depths;
}

// ============== Batch Occupancy Check ==============

Eigen::VectorXi OutofcoreTileMapper::batch_check_occupied(
    const Eigen::MatrixXd& points,
    double min_probability,
    int tolerance)
{
    if (points.cols() != 3) {
        throw std::invalid_argument("batch_check_occupied: points must be Nx3");
    }

    // Convert probability to log-odds threshold
    double min_log_odds;
    if (min_probability <= 0.0) {
        min_log_odds = -10.0;
    } else if (min_probability >= 1.0) {
        min_log_odds = 10.0;
    } else {
        min_log_odds = std::log(min_probability / (1.0 - min_probability));
    }

    int N = points.rows();
    Eigen::VectorXi result = Eigen::VectorXi::Zero(N);
    double step = resolution_;

    // Track current tile to avoid repeated lookups
    TileIndex current_tile_idx(-999999, -999999, -999999);
    octomap::OcTree* current_octree = nullptr;

    // Lambda to check a single point against the octree
    auto check_point = [&](double x, double y, double z) -> bool {
        TileIndex idx = tile_manager_.world_to_tile_index(x, y, z);
        if (idx != current_tile_idx) {
            current_tile_idx = idx;
            if (tile_manager_.tile_exists(idx)) {
                Tile* tile = get_or_load_tile(idx);
                current_octree = (tile) ? tile->get_octree() : nullptr;
            } else {
                current_octree = nullptr;
            }
        }
        if (!current_octree) return false;

        octomap::point3d oct_point(x, y, z);
        octomap::OcTreeNode* node = current_octree->search(oct_point);
        return (node && node->getLogOdds() > static_cast<float>(min_log_odds));
    };

    for (int i = 0; i < N; ++i) {
        double x = points(i, 0);
        double y = points(i, 1);
        double z = points(i, 2);

        if (tolerance <= 0) {
            // Exact voxel check only
            if (check_point(x, y, z)) {
                result(i) = 1;
            }
        } else {
            // Check neighborhood: ±tolerance voxels in each direction
            bool found = false;
            for (int dx = -tolerance; dx <= tolerance && !found; ++dx) {
                for (int dy = -tolerance; dy <= tolerance && !found; ++dy) {
                    for (int dz = -tolerance; dz <= tolerance && !found; ++dz) {
                        if (check_point(x + dx * step, y + dy * step, z + dz * step)) {
                            found = true;
                        }
                    }
                }
            }
            if (found) {
                result(i) = 1;
            }
        }
    }

    return result;
}

}  // namespace sonar_3d_reconstruction
