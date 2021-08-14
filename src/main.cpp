#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "mapf_map/mapf_map.h"
#include "agent/agent.h"
#include "algorithm/CBSH2/CBSHSearch.h"

namespace py = pybind11;

PYBIND11_MODULE(cbsrl, m) {
    py::class_<mapf::CBSH::CBSHSearch>(m, "CBSHRL")
        .def(py::init<>())
        .def("getstate", &mapf::CBSH::CBSHSearch::GetState)
        .def("reset", &mapf::CBSH::CBSHSearch::Reset)
        .def("step", &mapf::CBSH::CBSHSearch::Step)
        .def("isdone", &mapf::CBSH::CBSHSearch::isDone)
        .def("getreward", &mapf::CBSH::CBSHSearch::GetReward);
}