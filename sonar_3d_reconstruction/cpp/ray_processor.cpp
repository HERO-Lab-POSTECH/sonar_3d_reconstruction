#include "ray_casting.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace py = pybind11;

namespace sonar_3d_reconstruction
{

// High-performance sonar ray processing class
class RayProcessor
{
public:
    RayProcessor() = default;
    
    // Calculate 3D coordinates for multiple bearing angles (OpenMP parallelized)
    std::vector<std::vector<double>> process_sonar_rays(
        const std::vector<double>& ranges,
        const std::vector<double>& bearings,
        const std::vector<double>& vertical_angles,
        const std::vector<double>& sensor_origin,
        const std::vector<double>& sensor_rotation)
    {
        std::vector<std::vector<double>> points;
        points.reserve(ranges.size() * bearings.size() * vertical_angles.size());
        
        // Create Eigen rotation matrix
        Eigen::Matrix3d R = create_rotation_matrix(sensor_rotation[0], 
                                                  sensor_rotation[1], 
                                                  sensor_rotation[2]);
        
        #pragma omp parallel for collapse(3)
        for (size_t r = 0; r < ranges.size(); ++r) {
            for (size_t b = 0; b < bearings.size(); ++b) {
                for (size_t v = 0; v < vertical_angles.size(); ++v) {
                    double range = ranges[r];
                    double bearing = bearings[b];
                    double vertical = vertical_angles[v];
                    
                    // Calculate 3D coordinates in sonar coordinate system
                    Eigen::Vector3d sonar_point;
                    sonar_point << range * cos(vertical) * cos(bearing),
                                   range * cos(vertical) * sin(bearing),
                                   range * sin(vertical);
                    
                    // Apply sensor rotation and position transform
                    Eigen::Vector3d world_point = R * sonar_point;
                    world_point[0] += sensor_origin[0];
                    world_point[1] += sensor_origin[1];
                    world_point[2] += sensor_origin[2];
                    
                    #pragma omp critical
                    {
                        points.push_back({world_point[0], world_point[1], world_point[2]});
                    }
                }
            }
        }
        
        return points;
    }
    
    // Ray tracing using DDA algorithm
    std::vector<std::vector<double>> trace_ray_dda(
        const std::vector<double>& start,
        const std::vector<double>& end,
        double voxel_size = 0.1)
    {
        std::vector<std::vector<double>> voxels;
        
        Eigen::Vector3d start_pos(start[0], start[1], start[2]);
        Eigen::Vector3d end_pos(end[0], end[1], end[2]);
        
        // DDA algorithm implementation
        Eigen::Vector3d direction = (end_pos - start_pos).normalized();
        double distance = (end_pos - start_pos).norm();
        
        int steps = static_cast<int>(distance / voxel_size) + 1;
        
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            Eigen::Vector3d current_pos = start_pos + t * (end_pos - start_pos);
            
            voxels.push_back({current_pos[0], current_pos[1], current_pos[2]});
        }
        
        return voxels;
    }
    
    // Batch ray processing (process multiple rays simultaneously)
    std::vector<std::vector<std::vector<double>>> process_ray_batch(
        const std::vector<std::vector<double>>& starts,
        const std::vector<std::vector<double>>& ends,
        double voxel_size = 0.1)
    {
        std::vector<std::vector<std::vector<double>>> batch_result;
        batch_result.reserve(starts.size());
        
        #pragma omp parallel for
        for (size_t i = 0; i < starts.size(); ++i) {
            auto ray_voxels = trace_ray_dda(starts[i], ends[i], voxel_size);
            
            #pragma omp critical
            {
                batch_result.push_back(ray_voxels);
            }
        }
        
        return batch_result;
    }

private:
    // Create rotation matrix from roll, pitch, yaw angles
    Eigen::Matrix3d create_rotation_matrix(double roll, double pitch, double yaw)
    {
        Eigen::Matrix3d Rx, Ry, Rz;
        
        Rx << 1, 0, 0,
              0, cos(roll), -sin(roll),
              0, sin(roll), cos(roll);
              
        Ry << cos(pitch), 0, sin(pitch),
              0, 1, 0,
              -sin(pitch), 0, cos(pitch);
              
        Rz << cos(yaw), -sin(yaw), 0,
              sin(yaw), cos(yaw), 0,
              0, 0, 1;
              
        return Rz * Ry * Rx;
    }
};

}  // namespace sonar_3d_reconstruction

PYBIND11_MODULE(ray_processor, m) {
    m.doc() = "High-performance sonar ray processing module";

    py::class_<sonar_3d_reconstruction::RayProcessor>(m, "RayProcessor")
        .def(py::init<>())
        .def("process_sonar_rays", &sonar_3d_reconstruction::RayProcessor::process_sonar_rays,
             "Process sonar rays (parallelized)",
             py::arg("ranges"), py::arg("bearings"), py::arg("vertical_angles"),
             py::arg("sensor_origin"), py::arg("sensor_rotation"))
        .def("trace_ray_dda", &sonar_3d_reconstruction::RayProcessor::trace_ray_dda,
             "DDA ray tracing",
             py::arg("start"), py::arg("end"), py::arg("voxel_size") = 0.1)
        .def("process_ray_batch", &sonar_3d_reconstruction::RayProcessor::process_ray_batch,
             "Batch ray processing",
             py::arg("starts"), py::arg("ends"), py::arg("voxel_size") = 0.1);
}