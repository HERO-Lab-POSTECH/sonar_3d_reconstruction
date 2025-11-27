#include "octree_mapper.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace py = pybind11;

namespace sonar_3d_reconstruction
{

OctreeMapper::OctreeMapper()
{
}

OctreeMapper::~OctreeMapper()
{
}

// OctoMap 기반 옥트리 매핑 기능
class OctreeMapping
{
public:
    OctreeMapping(double resolution = 0.1) : octree_(resolution) {}
    
    void update_octree(const std::vector<std::vector<double>>& points,
                      const std::vector<double>& sensor_origin)
    {
        octomap::point3d origin(sensor_origin[0], sensor_origin[1], sensor_origin[2]);
        
        #pragma omp parallel for
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            octomap::point3d endpoint(point[0], point[1], point[2]);
            octree_.insertRay(origin, endpoint);
        }
    }
    
    std::vector<std::vector<double>> get_occupied_points() const
    {
        std::vector<std::vector<double>> occupied_points;
        
        for (auto it = octree_.begin_leafs(); it != octree_.end_leafs(); ++it) {
            if (octree_.isNodeOccupied(*it)) {
                auto coord = it.getCoordinate();
                occupied_points.push_back({coord.x(), coord.y(), coord.z()});
            }
        }
        
        return occupied_points;
    }
    
    void save_octree(const std::string& filename)
    {
        octree_.writeBinary(filename);
    }
    
    void load_octree(const std::string& filename)
    {
        octree_.readBinary(filename);
    }
    
private:
    octomap::OcTree octree_;
};

}  // namespace sonar_3d_reconstruction

PYBIND11_MODULE(octree_mapper, m) {
    m.doc() = "OctoMap 기반 옥트리 매핑 모듈";
    
    py::class_<sonar_3d_reconstruction::OctreeMapping>(m, "OctreeMapping")
        .def(py::init<double>(), py::arg("resolution") = 0.1)
        .def("update_octree", &sonar_3d_reconstruction::OctreeMapping::update_octree,
             "옥트리 업데이트", py::arg("points"), py::arg("sensor_origin"))
        .def("get_occupied_points", &sonar_3d_reconstruction::OctreeMapping::get_occupied_points,
             "점유된 포인트 반환")
        .def("save_octree", &sonar_3d_reconstruction::OctreeMapping::save_octree,
             "옥트리 저장", py::arg("filename"))
        .def("load_octree", &sonar_3d_reconstruction::OctreeMapping::load_octree,
             "옥트리 로드", py::arg("filename"));
}