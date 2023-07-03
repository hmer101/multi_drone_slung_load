#include <pybind11/pybind11.h>
#include "example_pybind.h"

namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function that adds two numbers", py::arg("i"), py::arg("j"));
}