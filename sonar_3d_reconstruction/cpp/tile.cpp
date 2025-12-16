#include "tile.h"
#include <cmath>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

namespace sonar_3d_reconstruction
{

Tile::Tile(const TileIndex& index, double resolution, double tile_size)
    : index_(index)
    , resolution_(resolution)
    , tile_size_(tile_size)
    , octree_(std::make_unique<octomap::OcTree>(resolution))
    , dirty_(false)
{
    // Configure octree
    octree_->setOccupancyThres(0.5);
    octree_->setProbHit(0.7);
    octree_->setProbMiss(0.3);
}

Tile::~Tile()
{
    // Unique pointer automatically cleans up
}

double Tile::log_odds_to_probability(double log_odds)
{
    return 1.0 / (1.0 + std::exp(-log_odds));
}

double Tile::intensity_to_weight(double intensity, const IWLOParams& params)
{
    if (intensity <= params.intensity_threshold) {
        return 0.0;
    }

    // Normalize: [threshold, max] -> [0, 1]
    double normalized = (intensity - params.intensity_threshold) /
                        (params.intensity_max - params.intensity_threshold);
    normalized = std::max(0.0, std::min(1.0, normalized));

    // Sigmoid transformation centered at 0.5
    double x = params.sharpness * (normalized - 0.5);
    return 1.0 / (1.0 + std::exp(-x));
}

double Tile::compute_alpha(int observation_count, const IWLOParams& params)
{
    return std::max(params.min_alpha, 1.0 / (1.0 + params.decay_rate * observation_count));
}

void Tile::update_voxel(const octomap::point3d& point,
                        double intensity,
                        bool /* is_occupied */,  // Determined by intensity threshold
                        const IWLOParams& params)
{
    // Get octree key for this point
    octomap::OcTreeKey key = octree_->coordToKey(point);

    // Get or create IWLO metadata
    auto& meta = iwlo_meta_[key];
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
    dirty_ = true;
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

bool Tile::save(const std::string& tile_dir)
{
    // Create directory if needed
    try {
        fs::create_directories(tile_dir);
    } catch (const std::exception& e) {
        std::cerr << "[Tile] Failed to create directory: " << tile_dir
                  << " - " << e.what() << std::endl;
        return false;
    }

    // Sync IWLO metadata to octree before saving
    sync_to_octree();

    // Prune octree before saving (merge identical adjacent nodes)
    // This optimizes storage and enables proper visualization of large uniform regions
    octree_->prune();

    // Save octree to .bt file (suppress OctoMap stdout)
    std::string octree_path = tile_dir + "/octree.bt";
    std::streambuf* old_cout = std::cout.rdbuf(nullptr);
    bool write_ok = octree_->writeBinary(octree_path);
    std::cout.rdbuf(old_cout);
    if (!write_ok) {
        return false;
    }

    // Save IWLO metadata
    std::string meta_path = tile_dir + "/iwlo_meta.bin";
    if (!save_iwlo_meta(meta_path)) {
        std::cerr << "[Tile] Failed to save IWLO metadata: " << meta_path << std::endl;
        return false;
    }

    dirty_ = false;
    return true;
}

bool Tile::load(const std::string& tile_dir)
{
    // Load octree from .bt file (suppress OctoMap stdout)
    std::string octree_path = tile_dir + "/octree.bt";
    if (fs::exists(octree_path)) {
        std::streambuf* old_cout = std::cout.rdbuf(nullptr);
        bool read_ok = octree_->readBinary(octree_path);
        std::cout.rdbuf(old_cout);
        if (!read_ok) {
            return false;
        }
    } else {
        // No octree file - start with empty octree
        octree_->clear();
    }

    // Load IWLO metadata
    std::string meta_path = tile_dir + "/iwlo_meta.bin";
    if (fs::exists(meta_path)) {
        if (!load_iwlo_meta(meta_path)) {
            std::cerr << "[Tile] Failed to load IWLO metadata: " << meta_path << std::endl;
            return false;
        }
    } else {
        // No metadata file - start with empty metadata
        iwlo_meta_.clear();
    }

    dirty_ = false;
    return true;
}

bool Tile::save_iwlo_meta(const std::string& filepath)
{
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs) {
        return false;
    }

    // Write header: magic number + version + count
    const uint32_t magic = 0x49574C4F;  // "IWLO"
    const uint32_t version = 1;
    const uint64_t count = iwlo_meta_.size();

    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&count), sizeof(count));

    // Write each entry: key[3] + log_odds + observation_count
    for (const auto& [key, meta] : iwlo_meta_) {
        ofs.write(reinterpret_cast<const char*>(&key[0]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&key[1]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&key[2]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&meta.log_odds), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&meta.observation_count), sizeof(int));
    }

    return ofs.good();
}

bool Tile::load_iwlo_meta(const std::string& filepath)
{
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) {
        return false;
    }

    // Read header
    uint32_t magic, version;
    uint64_t count;

    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    ifs.read(reinterpret_cast<char*>(&count), sizeof(count));

    if (magic != 0x49574C4F) {  // "IWLO"
        std::cerr << "[Tile] Invalid IWLO metadata file: bad magic number" << std::endl;
        return false;
    }

    if (version != 1) {
        std::cerr << "[Tile] Unsupported IWLO metadata version: " << version << std::endl;
        return false;
    }

    // Read entries
    iwlo_meta_.clear();
    iwlo_meta_.reserve(count);

    for (uint64_t i = 0; i < count; ++i) {
        octomap::OcTreeKey key;
        IWLOMeta meta;

        ifs.read(reinterpret_cast<char*>(&key[0]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&key[1]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&key[2]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&meta.log_odds), sizeof(double));
        ifs.read(reinterpret_cast<char*>(&meta.observation_count), sizeof(int));

        iwlo_meta_[key] = meta;
    }

    return ifs.good();
}

void Tile::sync_to_octree()
{
    // Update octree nodes based on IWLO metadata
    // Preserve actual log_odds values (not just boolean occupied/free)
    for (const auto& [key, meta] : iwlo_meta_) {
        // Get coordinate from key
        octomap::point3d coord = octree_->keyToCoord(key);

        // Create/update node and set actual log_odds value
        octomap::OcTreeNode* node = octree_->updateNode(coord, true, true);  // lazy_eval=true
        if (node) {
            node->setLogOdds(static_cast<float>(meta.log_odds));
        }
    }

    // Update inner node occupancy after batch updates
    octree_->updateInnerOccupancy();
}

std::vector<OccupiedVoxel> Tile::get_occupied_voxels(double min_probability) const
{
    std::vector<OccupiedVoxel> result;
    result.reserve(iwlo_meta_.size() / 2);  // Estimate half are occupied

    double min_log_odds;
    if (min_probability >= 1.0) {
        min_log_odds = 10.0;  // Very high threshold
    } else if (min_probability <= 0.0) {
        min_log_odds = -10.0;  // Very low threshold
    } else {
        min_log_odds = std::log(min_probability / (1.0 - min_probability));
    }

    for (const auto& [key, meta] : iwlo_meta_) {
        if (meta.log_odds > min_log_odds) {
            octomap::point3d coord = octree_->keyToCoord(key);
            double prob = log_odds_to_probability(meta.log_odds);
            result.emplace_back(coord.x(), coord.y(), coord.z(), prob);
        }
    }

    return result;
}

bool Tile::has_occupied_voxels(double occupied_threshold) const
{
    // Convert probability threshold to log-odds
    double log_odds_threshold;
    if (occupied_threshold >= 1.0) {
        log_odds_threshold = 10.0;
    } else if (occupied_threshold <= 0.0) {
        log_odds_threshold = -10.0;
    } else {
        log_odds_threshold = std::log(occupied_threshold / (1.0 - occupied_threshold));
    }

    // Check if any voxel exceeds threshold (early exit)
    for (const auto& [key, meta] : iwlo_meta_) {
        if (meta.log_odds >= log_odds_threshold) {
            return true;
        }
    }
    return false;
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

void Tile::clear()
{
    octree_->clear();
    iwlo_meta_.clear();
    dirty_ = true;
}

size_t Tile::prune()
{
    // First sync IWLO metadata to octree
    sync_to_octree();

    // Get node count before pruning
    size_t nodes_before = octree_->calcNumNodes();

    // Prune the octree (merge homogeneous child nodes)
    octree_->prune();

    // Get node count after pruning
    size_t nodes_after = octree_->calcNumNodes();

    size_t pruned = (nodes_before > nodes_after) ? (nodes_before - nodes_after) : 0;

    if (pruned > 0) {
        dirty_ = true;
    }

    return pruned;
}

size_t Tile::get_memory_usage() const
{
    // Estimate memory usage
    // OcTree: ~32 bytes per node (rough estimate)
    size_t octree_mem = octree_->calcNumNodes() * 32;

    // IWLO metadata: key (6 bytes) + meta (12 bytes) + overhead (~16 bytes per entry)
    size_t meta_mem = iwlo_meta_.size() * (6 + 12 + 16);

    return octree_mem + meta_mem;
}

}  // namespace sonar_3d_reconstruction
