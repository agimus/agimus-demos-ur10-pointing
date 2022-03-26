#include "dynamic-graph/python/module.hh"

#include "contact-simulation.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap)
{
  bp::import("dynamic_graph");
  dg::python::exposeEntity<dg::agimus::ContactSimulation>();
}
