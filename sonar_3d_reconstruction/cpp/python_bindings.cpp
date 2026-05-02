#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "octree_mapper.h"
#include "probability_updater.h"
#include "outofcore_tile_mapper.h"

namespace py = pybind11;

PYBIND11_MODULE(sonar_3d_reconstruction_cpp, m) {
    m.doc() = "High-performance C++ backend for sonar 3D reconstruction";

    // MemoryStats structure
    py::class_<sonar_3d_reconstruction::MemoryStats>(m, "MemoryStats")
        .def(py::init<>())
        .def(py::init<size_t, size_t, double, double>(),
             py::arg("nodes"), py::arg("leaf_nodes"), py::arg("mem_mb"), py::arg("efficiency"))
        .def_readwrite("num_nodes", &sonar_3d_reconstruction::MemoryStats::num_nodes)
        .def_readwrite("num_leaf_nodes", &sonar_3d_reconstruction::MemoryStats::num_leaf_nodes)
        .def_readwrite("memory_mb", &sonar_3d_reconstruction::MemoryStats::memory_mb)
        .def_readwrite("memory_efficiency", &sonar_3d_reconstruction::MemoryStats::memory_efficiency)
        .def("__repr__", [](const sonar_3d_reconstruction::MemoryStats& stats) {
            return "<MemoryStats: " + 
                   std::to_string(stats.num_nodes) + " nodes, " +
                   std::to_string(stats.memory_mb) + " MB>";
        });

    // OctreeMapper class
    py::class_<sonar_3d_reconstruction::OctreeMapper>(m, "OctreeMapper")
        .def(py::init<double, double, double, double, double>(),
             py::arg("resolution") = 0.05,
             py::arg("prob_hit") = 0.7,
             py::arg("prob_miss") = 0.3,
             py::arg("prob_thres_min") = 0.12,
             py::arg("prob_thres_max") = 0.97,
             "Create OctreeMapper with specified parameters")
        .def("update_point", &sonar_3d_reconstruction::OctreeMapper::update_point,
             py::arg("point"), py::arg("occupied"),
             "Update single point in octree")
        .def("batch_update", &sonar_3d_reconstruction::OctreeMapper::batch_update,
             py::arg("points"), py::arg("occupied_flags"),
             "Batch update multiple points")
        .def("batch_update_with_log_odds", &sonar_3d_reconstruction::OctreeMapper::batch_update_with_log_odds,
             py::arg("points"), py::arg("log_odds_updates"),
             "Batch update with log-odds values (Python SimpleOctree compatible)")
        .def("insert_ray", &sonar_3d_reconstruction::OctreeMapper::insert_ray,
             py::arg("origin"), py::arg("endpoint"),
             "Insert ray from origin to endpoint")
        .def("get_occupied_voxels", &sonar_3d_reconstruction::OctreeMapper::get_occupied_voxels,
             py::arg("min_probability") = -1.0,
             "Get all occupied voxels above threshold")
        .def("get_memory_usage", &sonar_3d_reconstruction::OctreeMapper::get_memory_usage,
             "Get memory usage statistics")
        .def("prune_tree", &sonar_3d_reconstruction::OctreeMapper::prune_tree,
             "Prune unnecessary nodes")
        .def("clear", &sonar_3d_reconstruction::OctreeMapper::clear,
             "Clear all data")
        .def("get_resolution", &sonar_3d_reconstruction::OctreeMapper::get_resolution,
             "Get voxel resolution")
        .def("get_num_nodes", &sonar_3d_reconstruction::OctreeMapper::get_num_nodes,
             "Get total number of nodes")
        .def("set_probability_params", &sonar_3d_reconstruction::OctreeMapper::set_probability_params,
             py::arg("prob_hit"), py::arg("prob_miss"),
             "Set probability parameters")
        .def("set_occupancy_thresholds", &sonar_3d_reconstruction::OctreeMapper::set_occupancy_thresholds,
             py::arg("min_thresh"), py::arg("max_thresh"),
             "Set occupancy thresholds")
        .def("save_to_file", &sonar_3d_reconstruction::OctreeMapper::save_to_file,
             py::arg("filename"),
             "Save octree to file")
        .def("load_from_file", &sonar_3d_reconstruction::OctreeMapper::load_from_file,
             py::arg("filename"),
             "Load octree from file");

    // ProbabilityUpdater class (main interface for Python)
    py::class_<sonar_3d_reconstruction::ProbabilityUpdater>(m, "ProbabilityUpdater")
        .def(py::init<double>(),
             py::arg("resolution") = 0.05,
             "Create ProbabilityUpdater with specified resolution")
        .def("set_log_odds_params", &sonar_3d_reconstruction::ProbabilityUpdater::set_log_odds_params,
             py::arg("log_odds_occupied"), py::arg("log_odds_free"),
             "Set log-odds parameters")
        .def("set_adaptive_params", &sonar_3d_reconstruction::ProbabilityUpdater::set_adaptive_params,
             py::arg("adaptive_enabled"), py::arg("adaptive_threshold"), py::arg("adaptive_max_ratio"),
             "Set adaptive update parameters")
        .def("set_clamping_thresholds", &sonar_3d_reconstruction::ProbabilityUpdater::set_clamping_thresholds,
             py::arg("min_prob"), py::arg("max_prob"),
             "Set probability clamping thresholds")
        // batch_update() removed - using IWLO only
        .def("get_occupied_voxels", &sonar_3d_reconstruction::ProbabilityUpdater::get_occupied_voxels,
             py::arg("min_probability") = 0.5,
             "Get all occupied voxels above threshold")
        .def("get_memory_usage", &sonar_3d_reconstruction::ProbabilityUpdater::get_memory_usage,
             "Get memory usage statistics")
        .def("get_num_nodes", &sonar_3d_reconstruction::ProbabilityUpdater::get_num_nodes,
             "Get number of nodes")
        .def("get_resolution", &sonar_3d_reconstruction::ProbabilityUpdater::get_resolution,
             "Get voxel resolution")
        .def("prune_tree", &sonar_3d_reconstruction::ProbabilityUpdater::prune_tree,
             "Prune unnecessary nodes")
        .def("clear", &sonar_3d_reconstruction::ProbabilityUpdater::clear,
             "Clear all data")
        .def("set_occupied_threshold", &sonar_3d_reconstruction::ProbabilityUpdater::set_occupied_threshold,
             py::arg("occupied_thresh"),
             "Set occupied threshold (prob >= this = occupied)")
        .def("set_intensity_params", &sonar_3d_reconstruction::ProbabilityUpdater::set_intensity_params,
             py::arg("intensity_threshold"), py::arg("intensity_max"),
             "Set intensity normalization parameters")
        // batch_update_weighted_average() removed - using IWLO only
        .def("set_iwlo_params", &sonar_3d_reconstruction::ProbabilityUpdater::set_iwlo_params,
             py::arg("sharpness"), py::arg("decay_rate"), py::arg("min_alpha"),
             py::arg("L_min"), py::arg("L_max"),
             "Set IWLO (Intensity-Weighted Log-Odds) parameters")
        .def("batch_update_iwlo", &sonar_3d_reconstruction::ProbabilityUpdater::batch_update_iwlo,
             py::arg("points"), py::arg("intensities"), py::arg("is_occupied"),
             "Batch update with IWLO method (combines Log-Odds with Weighted Average)")
        .def("set_incremental_sync", &sonar_3d_reconstruction::ProbabilityUpdater::set_incremental_sync,
             py::arg("enable") = true,
             "Enable or disable incremental synchronization")
        .def("force_full_sync", &sonar_3d_reconstruction::ProbabilityUpdater::force_full_sync,
             "Force full synchronization from internal storage to octree");

    // TileIndex structure
    py::class_<sonar_3d_reconstruction::TileIndex>(m, "TileIndex")
        .def(py::init<>())
        .def(py::init<int, int, int>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &sonar_3d_reconstruction::TileIndex::x)
        .def_readwrite("y", &sonar_3d_reconstruction::TileIndex::y)
        .def_readwrite("z", &sonar_3d_reconstruction::TileIndex::z)
        .def("to_string", &sonar_3d_reconstruction::TileIndex::to_string)
        .def("__repr__", [](const sonar_3d_reconstruction::TileIndex& idx) {
            return "<TileIndex: " + idx.to_string() + ">";
        })
        .def("__eq__", &sonar_3d_reconstruction::TileIndex::operator==);

    // OutofcoreTileMapper class (out-of-core disk-based mapper)
    py::class_<sonar_3d_reconstruction::OutofcoreTileMapper>(m, "OutofcoreTileMapper")
        .def(py::init<const std::string&, double, double, size_t>(),
             py::arg("map_path"),
             py::arg("resolution") = 0.05,
             py::arg("tile_size") = 10.0,
             py::arg("cache_size") = 16,
             "Create OutofcoreTileMapper with tile-based disk storage")
        // IWLO update (main interface)
        .def("batch_update_iwlo", &sonar_3d_reconstruction::OutofcoreTileMapper::batch_update_iwlo,
             py::arg("points"), py::arg("intensities"), py::arg("is_occupied"),
             "Batch update with IWLO method")
        // Parameter setters
        .def("set_iwlo_params", &sonar_3d_reconstruction::OutofcoreTileMapper::set_iwlo_params,
             py::arg("sharpness"), py::arg("decay_rate"), py::arg("min_alpha"),
             py::arg("L_min"), py::arg("L_max"),
             "Set IWLO parameters")
        .def("set_log_odds_params", &sonar_3d_reconstruction::OutofcoreTileMapper::set_log_odds_params,
             py::arg("log_odds_occupied"), py::arg("log_odds_free"),
             "Set log-odds parameters")
        .def("set_intensity_params", &sonar_3d_reconstruction::OutofcoreTileMapper::set_intensity_params,
             py::arg("intensity_threshold"), py::arg("intensity_max"),
             "Set intensity parameters")
        .def("set_occupied_threshold", &sonar_3d_reconstruction::OutofcoreTileMapper::set_occupied_threshold,
             py::arg("threshold"),
             "Set occupied threshold for visualization filtering")
        .def("set_adaptive_params", &sonar_3d_reconstruction::OutofcoreTileMapper::set_adaptive_params,
             py::arg("enabled"), py::arg("threshold"), py::arg("max_ratio"),
             "Set adaptive parameters")
        // Getters
        .def("get_occupied_voxels", &sonar_3d_reconstruction::OutofcoreTileMapper::get_occupied_voxels,
             py::arg("min_probability") = 0.5,
             "Get occupied voxels from cached tiles only")
        .def("get_all_occupied_voxels", &sonar_3d_reconstruction::OutofcoreTileMapper::get_all_occupied_voxels,
             py::arg("min_probability") = 0.5,
             "Get ALL occupied voxels from ALL tiles (loads from disk)")
        .def("get_occupied_voxels_in_region", &sonar_3d_reconstruction::OutofcoreTileMapper::get_occupied_voxels_in_region,
             py::arg("min_bound"), py::arg("max_bound"), py::arg("min_probability") = 0.5,
             "Get occupied voxels in a specific region")
        .def("get_memory_usage", &sonar_3d_reconstruction::OutofcoreTileMapper::get_memory_usage,
             "Get memory usage statistics")
        .def("get_resolution", &sonar_3d_reconstruction::OutofcoreTileMapper::get_resolution,
             "Get voxel resolution")
        .def("get_num_nodes", &sonar_3d_reconstruction::OutofcoreTileMapper::get_num_nodes,
             "Get total number of nodes across cached tiles")
        // Tile management
        .def("flush_all", &sonar_3d_reconstruction::OutofcoreTileMapper::flush_all,
             "Flush all dirty tiles to disk")
        .def("flush_tile", &sonar_3d_reconstruction::OutofcoreTileMapper::flush_tile,
             py::arg("idx"),
             "Flush specific tile to disk")
        .def("clear", &sonar_3d_reconstruction::OutofcoreTileMapper::clear,
             "Clear all data")
        // Octree export
        .def("save_merged_octree", &sonar_3d_reconstruction::OutofcoreTileMapper::save_merged_octree,
             py::arg("filepath"),
             "Save merged octree to .bt file")
        .def("get_octree_binary", [](sonar_3d_reconstruction::OutofcoreTileMapper& self, double min_probability) {
             auto result = self.get_octree_binary(min_probability);
             // Convert vector<int8_t> to py::bytes
             return py::make_tuple(
                 py::bytes(reinterpret_cast<const char*>(result.first.data()), result.first.size()),
                 result.second
             );
         }, py::arg("min_probability") = 0.5,
            "Get octree as binary data (data, tree_id) for ROS octomap_msgs, filtered by probability threshold")
        // Statistics
        .def("get_cached_tile_count", &sonar_3d_reconstruction::OutofcoreTileMapper::get_cached_tile_count,
             "Get number of tiles in cache")
        .def("get_total_tile_count", &sonar_3d_reconstruction::OutofcoreTileMapper::get_total_tile_count,
             "Get total number of tiles on disk")
        .def("get_all_tile_indices", &sonar_3d_reconstruction::OutofcoreTileMapper::get_all_tile_indices,
             "Get list of all tile indices")
        .def("get_disk_usage", &sonar_3d_reconstruction::OutofcoreTileMapper::get_disk_usage,
             "Get disk usage in bytes")
        // Region preloading
        .def("preload_region", &sonar_3d_reconstruction::OutofcoreTileMapper::preload_region,
             py::arg("min_bound"), py::arg("max_bound"),
             "Preload tiles in a region")
        // Selective tile reload (for visualization optimization)
        .def("reload_tiles", &sonar_3d_reconstruction::OutofcoreTileMapper::reload_tiles,
             py::arg("indices"),
             "Reload specific tiles from disk (for visualization sync)")
        .def("flush_and_get_dirty_tiles", &sonar_3d_reconstruction::OutofcoreTileMapper::flush_and_get_dirty_tiles,
             "Flush all dirty tiles and return their indices")
        // Pruning
        .def("prune_all", &sonar_3d_reconstruction::OutofcoreTileMapper::prune_all,
             "Prune all cached tiles (merge homogeneous octree nodes)")
        // Eviction notification (for visualizer sync)
        .def("get_and_clear_saved_tiles", &sonar_3d_reconstruction::OutofcoreTileMapper::get_and_clear_saved_tiles,
             "Get and clear list of recently saved tiles (from eviction)")
        // Ray-casting API
        .def("ray_cast_depth", &sonar_3d_reconstruction::OutofcoreTileMapper::ray_cast_depth,
             py::arg("origin"), py::arg("direction"), py::arg("max_range"),
             py::arg("step_size"), py::arg("min_probability") = 0.7,
             "Ray-cast to find depth of first occupied voxel along a direction")
        .def("batch_ray_cast_depth", &sonar_3d_reconstruction::OutofcoreTileMapper::batch_ray_cast_depth,
             py::arg("origin"), py::arg("directions"), py::arg("max_range"),
             py::arg("step_size"), py::arg("min_probability") = 0.7,
             "Batch ray-cast for multiple directions from a single origin")
        .def("batch_check_occupied", &sonar_3d_reconstruction::OutofcoreTileMapper::batch_check_occupied,
             py::arg("points"), py::arg("min_probability") = 0.7, py::arg("tolerance") = 0,
             "Batch check if points are occupied (1=occupied, 0=free/unknown). "
             "tolerance: number of neighboring voxels to check (0=exact, 1=3x3x3, 2=5x5x5)");

    // Module version info
    m.attr("__version__") = "1.0.0";
    m.attr("__author__") = "Sonar 3D Reconstruction Team";
    
    // Add some module-level documentation
    m.def("get_version", []() { return "1.0.0"; }, "Get module version");
    m.def("get_build_info", []() {
        std::string info = "Built with:";
        // OpenMP removed for thread-safety with OctoMap
        info += " Eigen3 OctoMap pybind11";
        return info;
    }, "Get build configuration info");
}