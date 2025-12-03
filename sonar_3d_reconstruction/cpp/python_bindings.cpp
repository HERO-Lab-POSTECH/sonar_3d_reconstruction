#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "octree_mapper.h"
#include "ray_casting.h"
#include "probability_updater.h"

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

    // RayCasting class
    py::class_<sonar_3d_reconstruction::RayCasting>(m, "RayCasting")
        .def(py::init<>(),
             "Create RayCasting utility")
        .def("generate_ray_points", &sonar_3d_reconstruction::RayCasting::generate_ray_points,
             py::arg("origin"), py::arg("endpoint"), py::arg("resolution"), py::arg("max_range") = -1.0,
             "Generate points along a ray")
        .def("cast_multiple_rays", &sonar_3d_reconstruction::RayCasting::cast_multiple_rays,
             py::arg("origin"), py::arg("endpoints"), py::arg("resolution"), py::arg("max_range") = -1.0,
             "Cast multiple rays in parallel")
        .def("generate_sonar_beam_pattern", &sonar_3d_reconstruction::RayCasting::generate_sonar_beam_pattern,
             py::arg("origin"), py::arg("range"), py::arg("bearing_angle"), 
             py::arg("vertical_aperture"), py::arg("num_vertical_samples") = 5,
             "Generate sonar beam pattern")
        .def("ray_sphere_intersection", &sonar_3d_reconstruction::RayCasting::ray_sphere_intersection,
             py::arg("ray_origin"), py::arg("ray_direction"), py::arg("sphere_center"), py::arg("sphere_radius"),
             "Check ray-sphere intersection")
        .def("is_point_in_sonar_fov", &sonar_3d_reconstruction::RayCasting::is_point_in_sonar_fov,
             py::arg("sonar_origin"), py::arg("sonar_direction"), py::arg("point"),
             py::arg("horizontal_fov"), py::arg("vertical_aperture"), py::arg("max_range"),
             "Check if point is in sonar FOV")
        .def("compute_sonar_angles", &sonar_3d_reconstruction::RayCasting::compute_sonar_angles,
             py::arg("sonar_origin"), py::arg("sonar_direction"), py::arg("point"),
             "Compute bearing and elevation angles")
        .def("generate_beam_voxels", &sonar_3d_reconstruction::RayCasting::generate_beam_voxels,
             py::arg("origin"), py::arg("direction"), py::arg("range"), py::arg("beam_width"), py::arg("resolution"),
             "Generate voxels within beam volume");

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
        .def("batch_update", &sonar_3d_reconstruction::ProbabilityUpdater::batch_update,
             py::arg("points"), py::arg("log_odds_updates"), py::arg("is_occupied"),
             "Batch update with adaptive probability updates")
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
        .def("set_thresholds", &sonar_3d_reconstruction::ProbabilityUpdater::set_thresholds,
             py::arg("occupied_thresh"), py::arg("free_thresh"),
             "Set thresholds for weighted average update")
        .def("set_intensity_params", &sonar_3d_reconstruction::ProbabilityUpdater::set_intensity_params,
             py::arg("intensity_threshold"), py::arg("intensity_max"),
             "Set intensity normalization parameters")
        .def("batch_update_weighted_average", &sonar_3d_reconstruction::ProbabilityUpdater::batch_update_weighted_average,
             py::arg("points"), py::arg("intensities"), py::arg("is_occupied"),
             "Batch update with weighted average method (voxelmap_fusion style)");

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