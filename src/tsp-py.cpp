#include "tsp.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_PLUGIN(pytsp) {
    py::module m("pytsp", "Algorithms to solve TSP.");
    m.def("random_distance_matrix", &tsp::randomDistanceMatrix);
    m.def("neighbor_matrix", &tsp::neighborMatrix);

    {
      auto submodule = m.def_submodule("heuristic_nearest");
      submodule.def("solve", &tsp::heuristic_nearest::solve);
    }

    {
      auto submodule = m.def_submodule("dynamic_programming");
      submodule.def("solve", &tsp::dynamic_programming::solve,
          py::arg("distance_matrix"), py::arg("allowPruning") = true);
      submodule.def("solve_with_heuristic", &tsp::dynamic_programming::solveWithHeuristic,
          py::arg("distance_matrix"), py::arg("allowPruning") = true);
    }

    {
      auto submodule = m.def_submodule("brute_force");
      submodule.def("solve", &tsp::brute_force::solve);
    }

    {
      auto submodule = m.def_submodule("approximative_kopt");
      submodule.def("solve_2opt", &tsp::approximative_kopt::solve2opt,
          py::arg("distance_matrix"), py::arg("initial_guess") = tsp::path_t());
      submodule.def("solve_3opt", &tsp::approximative_kopt::solve3opt,
          py::arg("distance_matrix"), py::arg("initial_guess") = tsp::path_t());
    }
}
