/*
* Binding file to convert frame_transforms from c++ to a file useable in python
*/

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "frame_transforms.h"

namespace py = pybind11;
using namespace px4_ros_com::frame_transforms;

/*
* CONVERSION FUNCTIONS
*/

// Convert numpy arrays to Eigen::Quaterniond types
Eigen::Quaterniond numpy_to_eigen_quaternion(py::array_t<float> arr) {
    if (arr.size() != 4)
        throw std::runtime_error("Input should be a numpy array of size 4.");
    auto r = arr.unchecked<1>();  // Unchecked access to numpy array data
    std::array<float, 4> q {r(0), r(1), r(2), r(3)};
    return utils::quaternion::array_to_eigen_quat(q);
}

// Converts Eigen::Quaterniond to a standard C++ container
std::vector<double> eigen_quaternion_to_vector(const Eigen::Quaterniond &q) {
    return std::vector<double>{q.w(), q.x(), q.y(), q.z()};
}


PYBIND11_MODULE(frame_transforms, m) {
    m.doc() = "Utility functions that assist with converting between different reference frames and pose representations"; // optional module docstring

    /*
    * UTILS 
    */
    m.def("quaternion_get_yaw", [](py::array_t<float> arr) {
        return utils::quaternion::quaternion_get_yaw(numpy_to_eigen_quaternion(arr));
    }, "A function that computes the yaw from a quaternion");

    /*
    * CONVERSIONS 
    */
    m.def("px4_to_ros_orientation", [](py::array_t<float> arr) {
        Eigen::Quaterniond in = numpy_to_eigen_quaternion(arr);
        Eigen::Quaterniond out = px4_to_ros_orientation(in);

        return eigen_quaternion_to_vector(out);
    }, "Transform from orientation represented in PX4 format to ROS format");

    m.def("ros_to_px4_orientation", [](py::array_t<float> arr) {
        Eigen::Quaterniond in = numpy_to_eigen_quaternion(arr);
        Eigen::Quaterniond out = ros_to_px4_orientation(in);
        
        return eigen_quaternion_to_vector(out);
    }, "Transform from orientation represented in ROS format to PX4 format");
}