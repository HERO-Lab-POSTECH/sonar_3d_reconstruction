#include <pybind11/pybind11.h>

#include "octree_mapper.h"
#include "ray_casting.h"
#include "probability_updater.h"

namespace py = pybind11;

PYBIND11_MODULE(sonar_3d_reconstruction_cpp, m) {
    m.doc() = "pybind11 sonar_3d_reconstruction plugin";

    py::class_<sonar_3d_reconstruction::OctreeMapper>(m, "OctreeMapper")
        .def(py::init<>());

    py::class_<sonar_3d_reconstruction::RayCasting>(m, "RayCasting")
        .def(py::init<>());

    py::class_<sonar_3d_reconstruction::ProbabilityUpdater>(m, "ProbabilityUpdater")
        .def(py::init<>());
}