#include "tile_manager.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <cmath>
#include <regex>
#include <sstream>

namespace fs = std::filesystem;

namespace sonar_3d_reconstruction
{

TileManager::TileManager(const std::string& base_path, double tile_size, double resolution)
    : base_path_(base_path)
    , tile_size_(tile_size)
    , resolution_(resolution)
{
    metadata_.resolution = resolution;
    metadata_.tile_size = tile_size;
    metadata_.version = "1.0";
}

bool TileManager::initialize()
{
    try {
        // Create base directory if needed
        fs::create_directories(base_path_);

        // Try to load existing metadata
        if (fs::exists(get_metadata_path())) {
            if (!load_metadata()) {
                std::cerr << "[TileManager] Warning: Failed to load existing metadata" << std::endl;
            }
        } else {
            // Save initial metadata
            if (!save_metadata()) {
                std::cerr << "[TileManager] Warning: Failed to save initial metadata" << std::endl;
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Failed to initialize: " << e.what() << std::endl;
        return false;
    }
}

TileIndex TileManager::world_to_tile_index(double x, double y, double z) const
{
    // Use floor to handle negative coordinates correctly
    int ix = static_cast<int>(std::floor(x / tile_size_));
    int iy = static_cast<int>(std::floor(y / tile_size_));
    int iz = static_cast<int>(std::floor(z / tile_size_));

    return TileIndex(ix, iy, iz);
}

TileIndex TileManager::world_to_tile_index(const octomap::point3d& point) const
{
    return world_to_tile_index(point.x(), point.y(), point.z());
}

std::string TileManager::get_tile_directory(const TileIndex& idx) const
{
    std::ostringstream oss;
    oss << base_path_ << "/tile_" << idx.x << "_" << idx.y << "_" << idx.z;
    return oss.str();
}

bool TileManager::tile_exists(const TileIndex& idx) const
{
    return fs::exists(get_tile_directory(idx));
}

std::vector<TileIndex> TileManager::list_all_tiles() const
{
    std::vector<TileIndex> tiles;

    if (!fs::exists(base_path_)) {
        return tiles;
    }

    try {
        for (const auto& entry : fs::directory_iterator(base_path_)) {
            if (entry.is_directory()) {
                TileIndex idx;
                if (parse_tile_dirname(entry.path().filename().string(), idx)) {
                    tiles.push_back(idx);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Error listing tiles: " << e.what() << std::endl;
    }

    return tiles;
}

std::vector<TileIndex> TileManager::get_tiles_in_region(
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound) const
{
    std::vector<TileIndex> tiles;

    // Get tile index range
    TileIndex min_idx = world_to_tile_index(min_bound.x(), min_bound.y(), min_bound.z());
    TileIndex max_idx = world_to_tile_index(max_bound.x(), max_bound.y(), max_bound.z());

    // Iterate over all tiles in range
    for (int ix = min_idx.x; ix <= max_idx.x; ++ix) {
        for (int iy = min_idx.y; iy <= max_idx.y; ++iy) {
            for (int iz = min_idx.z; iz <= max_idx.z; ++iz) {
                tiles.emplace_back(ix, iy, iz);
            }
        }
    }

    return tiles;
}

std::string TileManager::get_metadata_path() const
{
    return base_path_ + "/metadata.json";
}

bool TileManager::save_metadata()
{
    std::string filepath = get_metadata_path();

    try {
        std::ofstream ofs(filepath);
        if (!ofs) {
            return false;
        }

        // Simple JSON format (no external library dependency)
        ofs << "{\n";
        ofs << "  \"version\": \"" << metadata_.version << "\",\n";
        ofs << "  \"resolution\": " << metadata_.resolution << ",\n";
        ofs << "  \"tile_size\": " << metadata_.tile_size << ",\n";
        ofs << "  \"bounds\": {\n";
        ofs << "    \"min\": [" << metadata_.min_tile.x << ", "
            << metadata_.min_tile.y << ", " << metadata_.min_tile.z << "],\n";
        ofs << "    \"max\": [" << metadata_.max_tile.x << ", "
            << metadata_.max_tile.y << ", " << metadata_.max_tile.z << "]\n";
        ofs << "  }\n";
        ofs << "}\n";

        return ofs.good();
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Failed to save metadata: " << e.what() << std::endl;
        return false;
    }
}

bool TileManager::load_metadata()
{
    std::string filepath = get_metadata_path();

    try {
        std::ifstream ifs(filepath);
        if (!ifs) {
            return false;
        }

        // Simple JSON parsing (no external library)
        std::string content((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());

        // Parse version
        std::regex version_regex("\"version\"\\s*:\\s*\"([^\"]+)\"");
        std::smatch match;
        if (std::regex_search(content, match, version_regex)) {
            metadata_.version = match[1];
        }

        // Parse resolution
        std::regex resolution_regex("\"resolution\"\\s*:\\s*([0-9.]+)");
        if (std::regex_search(content, match, resolution_regex)) {
            metadata_.resolution = std::stod(match[1]);
            resolution_ = metadata_.resolution;
        }

        // Parse tile_size
        std::regex tile_size_regex("\"tile_size\"\\s*:\\s*([0-9.]+)");
        if (std::regex_search(content, match, tile_size_regex)) {
            metadata_.tile_size = std::stod(match[1]);
            tile_size_ = metadata_.tile_size;
        }

        // Parse bounds min
        std::regex bounds_min_regex("\"min\"\\s*:\\s*\\[\\s*(-?[0-9]+)\\s*,\\s*(-?[0-9]+)\\s*,\\s*(-?[0-9]+)\\s*\\]");
        if (std::regex_search(content, match, bounds_min_regex)) {
            metadata_.min_tile.x = std::stoi(match[1]);
            metadata_.min_tile.y = std::stoi(match[2]);
            metadata_.min_tile.z = std::stoi(match[3]);
        }

        // Parse bounds max
        std::regex bounds_max_regex("\"max\"\\s*:\\s*\\[\\s*(-?[0-9]+)\\s*,\\s*(-?[0-9]+)\\s*,\\s*(-?[0-9]+)\\s*\\]");
        if (std::regex_search(content, match, bounds_max_regex)) {
            metadata_.max_tile.x = std::stoi(match[1]);
            metadata_.max_tile.y = std::stoi(match[2]);
            metadata_.max_tile.z = std::stoi(match[3]);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Failed to load metadata: " << e.what() << std::endl;
        return false;
    }
}

void TileManager::update_bounds(const TileIndex& idx)
{
    metadata_.min_tile.x = std::min(metadata_.min_tile.x, idx.x);
    metadata_.min_tile.y = std::min(metadata_.min_tile.y, idx.y);
    metadata_.min_tile.z = std::min(metadata_.min_tile.z, idx.z);

    metadata_.max_tile.x = std::max(metadata_.max_tile.x, idx.x);
    metadata_.max_tile.y = std::max(metadata_.max_tile.y, idx.y);
    metadata_.max_tile.z = std::max(metadata_.max_tile.z, idx.z);
}

bool TileManager::delete_tile(const TileIndex& idx)
{
    std::string tile_dir = get_tile_directory(idx);

    if (!fs::exists(tile_dir)) {
        return true;  // Already deleted
    }

    try {
        fs::remove_all(tile_dir);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Failed to delete tile " << idx.to_string()
                  << ": " << e.what() << std::endl;
        return false;
    }
}

size_t TileManager::get_disk_usage() const
{
    size_t total_size = 0;

    if (!fs::exists(base_path_)) {
        return 0;
    }

    try {
        for (const auto& entry : fs::recursive_directory_iterator(base_path_)) {
            if (entry.is_regular_file()) {
                total_size += entry.file_size();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[TileManager] Error calculating disk usage: " << e.what() << std::endl;
    }

    return total_size;
}

bool TileManager::parse_tile_dirname(const std::string& dirname, TileIndex& idx) const
{
    // Expected format: "tile_X_Y_Z" where X, Y, Z can be negative
    std::regex tile_regex("tile_(-?[0-9]+)_(-?[0-9]+)_(-?[0-9]+)");
    std::smatch match;

    if (std::regex_match(dirname, match, tile_regex)) {
        idx.x = std::stoi(match[1]);
        idx.y = std::stoi(match[2]);
        idx.z = std::stoi(match[3]);
        return true;
    }

    return false;
}

}  // namespace sonar_3d_reconstruction
