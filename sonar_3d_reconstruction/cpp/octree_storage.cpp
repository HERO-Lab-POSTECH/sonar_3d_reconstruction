/**
 * @file octree_storage.cpp
 * @brief Implementation of OctreeStorage
 */

#include "octree_storage.h"
#include "iwlo_updater.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <cmath>

namespace fs = std::filesystem;

namespace sonar_3d_reconstruction
{

OctreeStorage::OctreeStorage(double resolution)
    : resolution_(resolution)
    , octree_(std::make_unique<octomap::OcTree>(resolution))
    , dirty_(false)
{
}

// =============================================================================
// Core Operations
// =============================================================================

double OctreeStorage::get_log_odds(const octomap::OcTreeKey& key) const
{
    auto it = iwlo_meta_.find(key);
    if (it != iwlo_meta_.end()) {
        return it->second.log_odds;
    }
    return 0.0;  // Default: unknown (P=0.5)
}

void OctreeStorage::set_log_odds(const octomap::OcTreeKey& key, double value)
{
    iwlo_meta_[key].log_odds = value;
    dirty_ = true;
}

int OctreeStorage::get_observation_count(const octomap::OcTreeKey& key) const
{
    auto it = iwlo_meta_.find(key);
    if (it != iwlo_meta_.end()) {
        return it->second.observation_count;
    }
    return 0;
}

int OctreeStorage::increment_observation_count(const octomap::OcTreeKey& key)
{
    dirty_ = true;
    return ++iwlo_meta_[key].observation_count;
}

IWLOMeta& OctreeStorage::get_or_create_meta(const octomap::OcTreeKey& key)
{
    dirty_ = true;
    return iwlo_meta_[key];
}

// =============================================================================
// Coordinate Operations
// =============================================================================

octomap::OcTreeKey OctreeStorage::coord_to_key(const octomap::point3d& point) const
{
    return octree_->coordToKey(point);
}

octomap::point3d OctreeStorage::key_to_coord(const octomap::OcTreeKey& key) const
{
    return octree_->keyToCoord(key);
}

// =============================================================================
// Query Operations
// =============================================================================

std::vector<OccupiedVoxel> OctreeStorage::get_occupied_voxels(double min_probability) const
{
    std::vector<OccupiedVoxel> result;
    result.reserve(iwlo_meta_.size() / 2);

    double min_log_odds;
    if (min_probability >= 1.0) {
        min_log_odds = 10.0;
    } else if (min_probability <= 0.0) {
        min_log_odds = -10.0;
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

size_t OctreeStorage::get_num_voxels() const
{
    return iwlo_meta_.size();
}

bool OctreeStorage::has_occupied_voxels(double min_probability) const
{
    double min_log_odds;
    if (min_probability >= 1.0) {
        min_log_odds = 10.0;
    } else if (min_probability <= 0.0) {
        min_log_odds = -10.0;
    } else {
        min_log_odds = std::log(min_probability / (1.0 - min_probability));
    }

    for (const auto& [key, meta] : iwlo_meta_) {
        if (meta.log_odds > min_log_odds) {
            return true;
        }
    }
    return false;
}

// =============================================================================
// Persistence Operations
// =============================================================================

bool OctreeStorage::save(const std::string& path)
{
    try {
        fs::create_directories(path);
    } catch (const std::exception& e) {
        std::cerr << "[OctreeStorage] Failed to create directory: " << path
                  << " - " << e.what() << std::endl;
        return false;
    }

    // Sync metadata to octree before saving
    sync_to_octree();

    // Prune octree
    octree_->prune();

    // Save octree
    std::string octree_path = path + "/octree.bt";
    if (!octree_->writeBinary(octree_path)) {
        return false;
    }

    // Save IWLO metadata
    std::string meta_path = path + "/iwlo_meta.bin";
    if (!save_iwlo_meta(meta_path)) {
        std::cerr << "[OctreeStorage] Failed to save IWLO metadata" << std::endl;
        return false;
    }

    dirty_ = false;
    return true;
}

bool OctreeStorage::load(const std::string& path)
{
    // Load octree
    std::string octree_path = path + "/octree.bt";
    if (fs::exists(octree_path)) {
        if (!octree_->readBinary(octree_path)) {
            return false;
        }
    } else {
        octree_->clear();
    }

    // Load IWLO metadata
    std::string meta_path = path + "/iwlo_meta.bin";
    if (fs::exists(meta_path)) {
        if (!load_iwlo_meta(meta_path)) {
            std::cerr << "[OctreeStorage] Failed to load IWLO metadata" << std::endl;
            return false;
        }
    } else {
        iwlo_meta_.clear();
    }

    dirty_ = false;
    return true;
}

bool OctreeStorage::save_iwlo_meta(const std::string& filepath)
{
    std::ofstream ofs(filepath, std::ios::binary);
    if (!ofs) {
        return false;
    }

    const uint32_t magic = 0x49574C4F;  // "IWLO"
    const uint32_t version = 1;
    const uint64_t count = iwlo_meta_.size();

    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&count), sizeof(count));

    for (const auto& [key, meta] : iwlo_meta_) {
        ofs.write(reinterpret_cast<const char*>(&key[0]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&key[1]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&key[2]), sizeof(unsigned short));
        ofs.write(reinterpret_cast<const char*>(&meta.log_odds), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&meta.observation_count), sizeof(int));
    }

    return ofs.good();
}

bool OctreeStorage::load_iwlo_meta(const std::string& filepath)
{
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) {
        return false;
    }

    uint32_t magic, version;
    uint64_t count;

    ifs.read(reinterpret_cast<char*>(&magic), sizeof(magic));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(version));
    ifs.read(reinterpret_cast<char*>(&count), sizeof(count));

    if (magic != 0x49574C4F) {
        std::cerr << "[OctreeStorage] Invalid IWLO metadata file: bad magic" << std::endl;
        return false;
    }

    if (version != 1) {
        std::cerr << "[OctreeStorage] Unsupported version: " << version << std::endl;
        return false;
    }

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

// =============================================================================
// Maintenance Operations
// =============================================================================

void OctreeStorage::clear()
{
    octree_->clear();
    iwlo_meta_.clear();
    dirty_ = true;
}

void OctreeStorage::sync_to_octree()
{
    for (const auto& [key, meta] : iwlo_meta_) {
        octomap::point3d coord = octree_->keyToCoord(key);
        octomap::OcTreeNode* node = octree_->updateNode(coord, true, true);
        if (node) {
            node->setLogOdds(static_cast<float>(meta.log_odds));
        }
    }
    octree_->updateInnerOccupancy();
}

size_t OctreeStorage::prune()
{
    size_t before = octree_->getNumLeafNodes();
    octree_->prune();
    size_t after = octree_->getNumLeafNodes();
    return before - after;
}

size_t OctreeStorage::get_memory_usage() const
{
    size_t octree_mem = octree_->memoryUsage();
    size_t meta_mem = iwlo_meta_.size() * (sizeof(octomap::OcTreeKey) + sizeof(IWLOMeta));
    return octree_mem + meta_mem;
}

// =============================================================================
// Helper Functions
// =============================================================================

double OctreeStorage::log_odds_to_probability(double log_odds)
{
    return IWLOUpdater::log_odds_to_probability(log_odds);
}

}  // namespace sonar_3d_reconstruction
