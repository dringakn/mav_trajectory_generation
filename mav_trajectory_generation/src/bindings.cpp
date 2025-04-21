#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <Eigen/Core>

#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/segment.h>

namespace nb  = nanobind;
using namespace mav_trajectory_generation;

// 10 coefficients ⇒ optimize snap (4th derivative) in 3D
using PolyOpt3D = PolynomialOptimization<10>;

NB_MODULE(mav_trajectory, m) {
    m.doc() = "Nanobind bindings for mav_trajectory_generation";

    // Vertex
    nb::class_<Vertex>(m, "Vertex")
        .def(nb::init<size_t>(), nb::arg("dimension"))

        // addConstraint(int, double)
        .def("add_constraint",
             (void (Vertex::*)(int, double))
               &Vertex::addConstraint,
             nb::arg("derivative"),
             nb::arg("value"))

        // addConstraint(int, VectorXd)
        .def("add_constraint_vec",
             (void (Vertex::*)(int, const Eigen::VectorXd&))
               &Vertex::addConstraint,
             nb::arg("derivative"),
             nb::arg("values"))

        // makeStartOrEnd(double)
        .def("make_start_or_end",
             (void (Vertex::*)(double, int))
               &Vertex::makeStartOrEnd,
             nb::arg("value"),
             nb::arg("up_to_derivative"))

        // makeStartOrEnd(VectorXd)
        .def("make_start_or_end_vec",
             (void (Vertex::*)(const Eigen::VectorXd&, int))
               &Vertex::makeStartOrEnd,
             nb::arg("values"),
             nb::arg("up_to_derivative"))

        .def("remove_constraint",
             &Vertex::removeConstraint,
             nb::arg("derivative"))

        .def("has_constraint",
             &Vertex::hasConstraint,
             nb::arg("derivative"))

        // wrap getConstraint → returns Eigen::VectorXd (empty if missing)
        .def("get_constraint",
             [](const Vertex &v, int d) {
                Eigen::VectorXd c;
                if (v.getConstraint(d, &c))
                    return c;
                else
                    return Eigen::VectorXd();  // empty indicates “not set”
             },
             nb::arg("derivative"))
        ;

    // estimateSegmentTimes
    m.def("estimate_segment_times",
          &estimateSegmentTimes,
          nb::arg("vertices"),
          nb::arg("v_max"),
          nb::arg("a_max"));

    // PolynomialOptimization<10>
    nb::class_<PolyOpt3D>(m, "PolynomialOptimization")
        .def(nb::init<size_t>(), nb::arg("dimension"))
        .def("setup_from_vertices",
             &PolyOpt3D::setupFromVertices,
             nb::arg("vertices"),
             nb::arg("segment_times"),
             nb::arg("derivative_to_optimize"))
        .def("solve_linear",
             &PolyOpt3D::solveLinear)
        .def("get_segments",
             [](PolyOpt3D &opt) {
                 Segment::Vector segs;
                 opt.getSegments(&segs);
                 return segs;
             });

    // Segment
    nb::class_<Segment>(m, "Segment")
        .def("get_time", &Segment::getTime)

        // returns list of coefficient lists: [[c0,c1...], [...], ...]
        .def("get_polynomials",
             [](const Segment &s) {
                const auto &vec = s.getPolynomialsRef();
                std::vector<std::vector<double>> out;
                out.reserve(vec.size());
                for (const auto &poly : vec) {
                    Eigen::VectorXd c = poly.getCoefficients();
                    out.emplace_back(c.data(), c.data() + c.size());
                }
                return out;
             });
}
