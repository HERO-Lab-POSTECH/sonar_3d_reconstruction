/**
 * @file tile.cpp
 * @brief Tile class implementation using OctreeStorage
 *
 * Phase 2 refactoring: Tile now delegates storage to OctreeStorage,
 * keeping only tile-specific logic (bounds, index, IWLO updates).
 */

#include "tile.h"
#include "octree_storage.h"
#include "iwlo_updater.h"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace sonar_3d_reconstruction
{

Tile::Tile(const TileIndex& index, double resolution, double tile_size)
    : index_(index)
    , tile_size_(tile_size)
    , storage_(std::make_unique<OctreeStorage>(resolution))
{
}

Tile::~Tile()
{
    // Unique pointer automatically cleans up
}

// =============================================================================
// Static Helper Methods (delegate to IWLOUpdater)
// =============================================================================

double Tile::log_odds_to_probability(double log_odds)
{
    return IWLOUpdater::log_odds_to_probability(log_odds);
}

double Tile::intensity_to_weight(double intensity, const IWLOParams& params)
{
    return IWLOUpdater::intensity_to_weight(intensity, params);
}

double Tile::compute_alpha(int observation_count, const IWLOParams& params)
{
    return IWLOUpdater::compute_alpha(observation_count, params);
}

// =============================================================================
// IWLO Update Methods
// =============================================================================

void Tile::update_voxel(const octomap::point3d& point,
                        double intensity,
                        bool /* is_occupied */,
                        const IWLOParams& params)
{
    // Get octree key for this point
    octomap::OcTreeKey key = storage_->coord_to_key(point);

    // Get or create IWLO metadata
    IWLOMeta& meta = storage_->get_or_create_meta(key);
    meta.observation_count++;
    int n = meta.observation_count;

    double current_log_odds = meta.log_odds;
    double current_prob = log_odds_to_probability(current_log_odds);

    // 1. Compute intensity-based weight
    double w_intensity = intensity_to_weight(intensity, params);

    // 2. Compute learning rate based on observation count
    double alpha_n = compute_alpha(n - 1, params);

    // 3. Compute adaptive scaling (bidirectional protection)
    double adapt_scale = 1.0;
    if (params.adaptive_enabled) {
        if (intensity > params.intensity_threshold) {
            // Occupied update: protect free voxels
            if (current_prob < params.adaptive_threshold) {
                adapt_scale = (current_prob / params.adaptive_threshold) * params.adaptive_max_ratio;
            }
        } else {
            // Free update: protect occupied voxels
            if (current_prob > (1.0 - params.adaptive_threshold)) {
                double protection_factor = (current_prob - (1.0 - params.adaptive_threshold)) / params.adaptive_threshold;
                adapt_scale = params.adaptive_max_ratio + (1.0 - params.adaptive_max_ratio) * (1.0 - protection_factor);
            }
        }
    }

    // 4. Compute log-odds update
    double delta_L;
    if (intensity > params.intensity_threshold) {
        // Occupied update: ΔL = L_occ × w(I) × α(n) × scale
        delta_L = params.log_odds_occupied * w_intensity * alpha_n * adapt_scale;
    } else {
        // Free space update: ΔL = L_free × α(n) × scale
        delta_L = params.log_odds_free * alpha_n * adapt_scale;
    }

    // 5. Apply update with saturation limits
    double new_log_odds = current_log_odds + delta_L;
    new_log_odds = std::max(params.L_min, std::min(params.L_max, new_log_odds));

    meta.log_odds = new_log_odds;
    storage_->mark_dirty();
}

void Tile::batch_update(const Eigen::MatrixXd& points,
                        const Eigen::VectorXd& intensities,
                        const std::vector<bool>& is_occupied,
                        const IWLOParams& params)
{
    if (points.rows() != intensities.rows() ||
        points.rows() != static_cast<int>(is_occupied.size())) {
        throw std::invalid_argument("Batch update: size mismatch");
    }

    for (int i = 0; i < points.rows(); ++i) {
        octomap::point3d pt(points(i, 0), points(i, 1), points(i, 2));
        update_voxel(pt, intensities(i), is_occupied[i], params);
    }
}

// =============================================================================
// Persistence (delegate to OctreeStorage)
// =============================================================================

bool Tile::save(const std::string& tile_dir)
{
    return storage_->save(tile_dir);
}

bool Tile::load(const std::string& tile_dir)
{
    return storage_->load(tile_dir);
}

// =============================================================================
// Query Methods (delegate to OctreeStorage)
// =============================================================================

std::vector<OccupiedVoxel> Tile::get_occupied_voxels(double min_probability) const
{
    return storage_->get_occupied_voxels(min_probability);
}

bool Tile::has_occupied_voxels(double occupied_threshold) const
{
    return storage_->has_occupied_voxels(occupied_threshold);
}

Eigen::MatrixXd Tile::get_occupied_voxels_matrix(double min_probability) const
{
    auto voxels = get_occupied_voxels(min_probability);

    if (voxels.empty()) {
        return Eigen::MatrixXd(0, 4);
    }

    Eigen::MatrixXd result(voxels.size(), 4);
    for (size_t i = 0; i < voxels.size(); ++i) {
        result(i, 0) = voxels[i].x;
        result(i, 1) = voxels[i].y;
        result(i, 2) = voxels[i].z;
        result(i, 3) = voxels[i].probability;
    }

    return result;
}

// =============================================================================
// OcTree Access (delegate to OctreeStorage)
// =============================================================================

octomap::OcTree* Tile::get_octree()
{
    return storage_->get_octree();
}

const octomap::OcTree* Tile::get_octree() const
{
    return storage_->get_octree();
}

// =============================================================================
// State Management (delegate to OctreeStorage)
// =============================================================================

bool Tile::is_dirty() const
{
    return storage_->is_dirty();
}

void Tile::mark_clean()
{
    storage_->mark_clean();
}

void Tile::mark_dirty()
{
    storage_->mark_dirty();
}

size_t Tile::get_num_voxels() const
{
    return storage_->get_num_voxels();
}

// =============================================================================
// Maintenance (delegate to OctreeStorage)
// =============================================================================

void Tile::sync_to_octree()
{
    storage_->sync_to_octree();
}

void Tile::clear()
{
    storage_->clear();
}

size_t Tile::prune()
{
    sync_to_octree();
    return storage_->prune();
}

size_t Tile::get_memory_usage() const
{
    return storage_->get_memory_usage();
}

// =============================================================================
// Tile-specific Methods (bounds, containment)
// =============================================================================

Eigen::Vector3d Tile::get_min_bound() const
{
    return Eigen::Vector3d(
        index_.x * tile_size_,
        index_.y * tile_size_,
        index_.z * tile_size_
    );
}

Eigen::Vector3d Tile::get_max_bound() const
{
    return Eigen::Vector3d(
        (index_.x + 1) * tile_size_,
        (index_.y + 1) * tile_size_,
        (index_.z + 1) * tile_size_
    );
}

bool Tile::contains(const octomap::point3d& point) const
{
    double min_x = index_.x * tile_size_;
    double min_y = index_.y * tile_size_;
    double min_z = index_.z * tile_size_;
    double max_x = (index_.x + 1) * tile_size_;
    double max_y = (index_.y + 1) * tile_size_;
    double max_z = (index_.z + 1) * tile_size_;

    return point.x() >= min_x && point.x() < max_x &&
           point.y() >= min_y && point.y() < max_y &&
           point.z() >= min_z && point.z() < max_z;
}

}  // namespace sonar_3d_reconstruction
