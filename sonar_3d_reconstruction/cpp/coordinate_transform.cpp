#include "coordinate_transform.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace py = pybind11;

namespace sonar_3d_reconstruction
{

// CoordinateTransform implementation
CoordinateTransform::CoordinateTransform()
{
}

CoordinateTransform::~CoordinateTransform()
{
}

}  // namespace sonar_3d_reconstruction

// Python bindings for the new Eigen3-based coordinate transform system
PYBIND11_MODULE(coordinate_transform, m) {
    m.doc() = "Eigen3-based high-performance coordinate transformation system for sonar 3D reconstruction";
    
    using namespace sonar_3d_reconstruction;
    
    // ========================================================================
    // TransformConfig binding
    // ========================================================================
    py::class_<TransformConfig>(m, "TransformConfig")
        .def(py::init<>())
        .def_readwrite("sonar_position", &TransformConfig::sonar_position,
                      "Sonar mounting position relative to base_link [x, y, z] in meters")
        .def_readwrite("sonar_orientation", &TransformConfig::sonar_orientation,
                      "Sonar mounting orientation as RPY angles [roll, pitch, yaw] in radians")
        .def_readwrite("enable_validation", &TransformConfig::enable_validation,
                      "Enable numerical validation of transformations")
        .def_readwrite("position_tolerance", &TransformConfig::position_tolerance,
                      "Position validation tolerance in meters")
        .def_readwrite("rotation_tolerance", &TransformConfig::rotation_tolerance,
                      "Rotation validation tolerance in radians")
        .def("__repr__", [](const TransformConfig& config) {
            return "<TransformConfig: position=" + 
                   std::to_string(config.sonar_position.transpose().norm()) + 
                   "m, orientation=" + 
                   std::to_string(config.sonar_orientation.transpose().norm()) + 
                   "rad>";
        });
    
    // ========================================================================
    // TransformResult binding
    // ========================================================================
    py::class_<TransformResult>(m, "TransformResult")
        .def(py::init<>())
        .def_readonly("transformed_points", &TransformResult::transformed_points,
                     "Vector of transformed 3D points")
        .def_readonly("success", &TransformResult::success,
                     "Whether transformation was successful")
        .def_readonly("error_message", &TransformResult::error_message,
                     "Error description if transformation failed")
        .def_readonly("input_count", &TransformResult::input_count,
                     "Number of input points")
        .def_readonly("output_count", &TransformResult::output_count,
                     "Number of successfully transformed points")
        .def_readonly("processing_time_ms", &TransformResult::processing_time_ms,
                     "Processing time in milliseconds")
        .def("__repr__", [](const TransformResult& result) {
            return "<TransformResult: " + std::to_string(result.output_count) + 
                   "/" + std::to_string(result.input_count) + 
                   " points, " + std::to_string(result.processing_time_ms) + "ms>";
        });
    
    // ========================================================================
    // Statistics binding
    // ========================================================================
    py::class_<CoordinateTransform::Statistics>(m, "Statistics")
        .def(py::init<>())
        .def_readonly("total_points_processed", &CoordinateTransform::Statistics::total_points_processed)
        .def_readonly("total_processing_time_ms", &CoordinateTransform::Statistics::total_processing_time_ms)
        .def_readonly("batch_count", &CoordinateTransform::Statistics::batch_count)
        .def("get_avg_processing_time_ms", &CoordinateTransform::Statistics::get_avg_processing_time_ms,
             "Get average processing time per batch in milliseconds")
        .def("get_avg_points_per_batch", &CoordinateTransform::Statistics::get_avg_points_per_batch,
             "Get average number of points per batch")
        .def("__repr__", [](const CoordinateTransform::Statistics& stats) {
            return "<Statistics: " + std::to_string(stats.total_points_processed) + 
                   " points, " + std::to_string(stats.batch_count) + " batches>";
        });
    
    // ========================================================================
    // CoordinateTransform main class binding
    // ========================================================================
    py::class_<CoordinateTransform>(m, "CoordinateTransform")
        .def(py::init<const TransformConfig&>(), py::arg("config") = TransformConfig(),
             "Initialize coordinate transformer with configuration")
        
        // Static transformation matrix creation methods
        .def_static("create_transform_matrix", &CoordinateTransform::create_transform_matrix,
                   py::arg("position"), py::arg("rpy"),
                   "Create 4x4 transformation matrix from position and RPY angles")
        .def_static("quaternion_to_matrix", &CoordinateTransform::quaternion_to_matrix,
                   py::arg("quaternion"),
                   "Convert quaternion to 3x3 rotation matrix")
        .def_static("pose_to_transform", &CoordinateTransform::pose_to_transform,
                   py::arg("position"), py::arg("quaternion"),
                   "Create transformation matrix from position and quaternion")
        .def_static("validate_transform", &CoordinateTransform::validate_transform,
                   py::arg("transform"),
                   "Validate transformation matrix for numerical stability")
        
        // Point transformation methods
        .def("transform_point", &CoordinateTransform::transform_point,
             py::arg("point_sonar"), py::arg("T_base_to_world"),
             "Transform single point from sonar to world coordinates")
        .def("transform_points", 
             py::overload_cast<const std::vector<Eigen::Vector3d>&, const Eigen::Matrix4d&>
             (&CoordinateTransform::transform_points, py::const_),
             py::arg("points_sonar"), py::arg("T_base_to_world"),
             "Transform batch of points using transformation matrix")
        .def("transform_points",
             py::overload_cast<const std::vector<Eigen::Vector3d>&, const Eigen::Vector3d&, const Eigen::Quaterniond&>
             (&CoordinateTransform::transform_points, py::const_),
             py::arg("points_sonar"), py::arg("position"), py::arg("quaternion"),
             "Transform batch of points using position and quaternion")
        
        // Configuration and statistics
        .def("get_sonar_to_base_transform", &CoordinateTransform::get_sonar_to_base_transform,
             py::return_value_policy::reference_internal,
             "Get sonar to base_link transformation matrix")
        .def("update_config", &CoordinateTransform::update_config,
             py::arg("config"),
             "Update configuration and recompute transforms")
        .def("get_config", &CoordinateTransform::get_config,
             py::return_value_policy::reference_internal,
             "Get current configuration")
        .def("get_statistics", &CoordinateTransform::get_statistics,
             py::return_value_policy::reference_internal,
             "Get performance statistics")
        .def("reset_statistics", &CoordinateTransform::reset_statistics,
             "Reset performance statistics")
        .def("__repr__", [](const CoordinateTransform& transform) {
            const auto& stats = transform.get_statistics();
            return "<CoordinateTransform: " + 
                   std::to_string(stats.total_points_processed) + 
                   " points processed>";
        });
    
    // ========================================================================
    // Coordinate utilities namespace
    // ========================================================================
    py::module_ utils = m.def_submodule("utils", "Coordinate transformation utilities");
    
    utils.def("rpy_to_rotation_matrix", &coordinate_utils::rpy_to_rotation_matrix,
              py::arg("roll"), py::arg("pitch"), py::arg("yaw"),
              "Convert RPY angles to 3x3 rotation matrix");
    utils.def("rotation_matrix_to_rpy", &coordinate_utils::rotation_matrix_to_rpy,
              py::arg("rotation_matrix"),
              "Convert 3x3 rotation matrix to RPY angles");
    utils.def("is_point_valid", &coordinate_utils::is_point_valid,
              py::arg("point"),
              "Check if point has valid (finite) coordinates");
    utils.def("compose_transforms", &coordinate_utils::compose_transforms,
              py::arg("T1"), py::arg("T2"),
              "Compose two transformation matrices: T2 * T1");
    utils.def("extract_translation", &coordinate_utils::extract_translation,
              py::arg("transform"),
              "Extract translation vector from transformation matrix");
    utils.def("extract_rotation", &coordinate_utils::extract_rotation,
              py::arg("transform"),
              "Extract rotation matrix from transformation matrix");
    
    // ========================================================================
    // Convenience functions for Python integration
    // ========================================================================
    
    // High-level wrapper function matching Python 3d_mapper.py interface
    m.def("sonar_to_world_transform", [](
        const std::vector<std::vector<double>>& points_list,
        const std::vector<double>& sonar_position,
        const std::vector<double>& sonar_orientation,
        const std::vector<double>& robot_position,
        const std::vector<double>& robot_quaternion) -> py::list {
        
        // Convert Python lists to Eigen vectors
        TransformConfig config;
        config.sonar_position = Eigen::Vector3d(sonar_position.data());
        config.sonar_orientation = Eigen::Vector3d(sonar_orientation.data());
        
        CoordinateTransform transformer(config);
        
        // Convert input points
        std::vector<Eigen::Vector3d> points_sonar;
        points_sonar.reserve(points_list.size());
        for (const auto& pt_list : points_list) {
            if (pt_list.size() >= 3) {
                points_sonar.emplace_back(pt_list[0], pt_list[1], pt_list[2]);
            }
        }
        
        // Create transformation
        Eigen::Vector3d position(robot_position.data());
        Eigen::Quaterniond quaternion(robot_quaternion[3], robot_quaternion[0], 
                                     robot_quaternion[1], robot_quaternion[2]);
        
        // Transform points
        auto result = transformer.transform_points(points_sonar, position, quaternion);
        
        // Convert back to Python list
        py::list output_points;
        for (const auto& pt : result.transformed_points) {
            py::list pt_list;
            pt_list.append(pt[0]);
            pt_list.append(pt[1]);
            pt_list.append(pt[2]);
            output_points.append(pt_list);
        }
        
        return output_points;
    }, py::arg("points"), py::arg("sonar_position"), py::arg("sonar_orientation"),
       py::arg("robot_position"), py::arg("robot_quaternion"),
       "High-level interface: transform sonar points to world coordinates");
    
    // Batch processing function
    m.def("batch_sonar_to_world", [](
        const std::vector<std::vector<std::vector<double>>>& frame_points,
        const std::vector<double>& sonar_position,
        const std::vector<double>& sonar_orientation,
        const std::vector<std::vector<double>>& robot_positions,
        const std::vector<std::vector<double>>& robot_quaternions) -> py::list {
        
        // Setup transformer
        TransformConfig config;
        config.sonar_position = Eigen::Vector3d(sonar_position.data());
        config.sonar_orientation = Eigen::Vector3d(sonar_orientation.data());
        
        CoordinateTransform transformer(config);
        
        py::list output_frames;
        
        // Process each frame
        for (size_t frame_idx = 0; frame_idx < frame_points.size() && 
                                  frame_idx < robot_positions.size() &&
                                  frame_idx < robot_quaternions.size(); ++frame_idx) {
            
            // Convert frame points
            std::vector<Eigen::Vector3d> points_sonar;
            points_sonar.reserve(frame_points[frame_idx].size());
            for (const auto& pt_list : frame_points[frame_idx]) {
                if (pt_list.size() >= 3) {
                    points_sonar.emplace_back(pt_list[0], pt_list[1], pt_list[2]);
                }
            }
            
            // Robot pose for this frame
            const auto& pos_list = robot_positions[frame_idx];
            const auto& quat_list = robot_quaternions[frame_idx];
            
            if (pos_list.size() >= 3 && quat_list.size() >= 4) {
                Eigen::Vector3d position(pos_list.data());
                Eigen::Quaterniond quaternion(quat_list[3], quat_list[0], quat_list[1], quat_list[2]);
                
                // Transform
                auto result = transformer.transform_points(points_sonar, position, quaternion);
                
                // Convert to Python
                py::list frame_output;
                for (const auto& pt : result.transformed_points) {
                    py::list pt_list;
                    pt_list.append(pt[0]);
                    pt_list.append(pt[1]);
                    pt_list.append(pt[2]);
                    frame_output.append(pt_list);
                }
                
                output_frames.append(frame_output);
            }
        }
        
        return output_frames;
    }, py::arg("frame_points"), py::arg("sonar_position"), py::arg("sonar_orientation"),
       py::arg("robot_positions"), py::arg("robot_quaternions"),
       "Batch processing: transform multiple frames of sonar points to world coordinates");
    
    // Version and build info
    m.attr("__version__") = "1.0.0";
    m.attr("__build_info__") = "Eigen3 + OpenMP optimized";
}