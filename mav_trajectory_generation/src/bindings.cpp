#include <nanobind/nanobind.h>
#include <Eigen/Core>
#include <nanobind/eigen/dense.h> // Eigen ↔ NumPy converters
#include <nanobind/stl/vector.h>  // for std::vector<double> ↔ list

#include <mav_trajectory_generation/motion_defines.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/segment.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>

namespace nb = nanobind;
using namespace Eigen;
using namespace mav_trajectory_generation;

// 10 coefficients ⇒ optimize snap (4th derivative) in 3D
using PolyOpt3D = PolynomialOptimization<10>;

// alias the 10th‐order nonlinear optimizer
template <int N>
using PolyOptNonLin = PolynomialOptimizationNonLinear<N>;

NB_MODULE(mav_trajectory_generation_py, m)
{
     m.doc() = "Nanobind bindings for mav_trajectory_generation";

     // expose mav_trajectory_generation::derivative_order as a submodule
     auto d = m.def_submodule("derivative_order", "Derivative order constants");
     using namespace mav_trajectory_generation::derivative_order;
     d.attr("POSITION") = POSITION;
     d.attr("VELOCITY") = VELOCITY;
     d.attr("ACCELERATION") = ACCELERATION;
     d.attr("JERK") = JERK;
     d.attr("SNAP") = SNAP;
     d.attr("ORIENTATION") = ORIENTATION;
     d.attr("ANGULAR_VELOCITY") = ANGULAR_VELOCITY;
     d.attr("ANGULAR_ACCELERATION") = ANGULAR_ACCELERATION;
     d.attr("INVALID") = INVALID;

     // Vertex
     nb::class_<Vertex>(m, "Vertex")
         .def(nb::init<size_t>(), nb::arg("dimension"))

         // addConstraint(int, double)
         .def("add_constraint",
              (void (Vertex::*)(int, double))&Vertex::addConstraint,
              nb::arg("derivative"),
              nb::arg("value"))

         // addConstraint(int, VectorXd)
         .def("add_constraint_vec",
              (void (Vertex::*)(int, const Eigen::VectorXd &))&Vertex::addConstraint,
              nb::arg("derivative"),
              nb::arg("values"))

         // makeStartOrEnd(double)
         .def("make_start_or_end",
              (void (Vertex::*)(double, int))&Vertex::makeStartOrEnd,
              nb::arg("value"),
              nb::arg("up_to_derivative"))

         // makeStartOrEnd(VectorXd)
         .def("make_start_or_end_vec",
              (void (Vertex::*)(const Eigen::VectorXd &, int))&Vertex::makeStartOrEnd,
              nb::arg("values"),
              nb::arg("up_to_derivative"))

         .def("remove_constraint",
              &Vertex::removeConstraint,
              nb::arg("derivative"))

         .def("has_constraint",
              &Vertex::hasConstraint,
              nb::arg("derivative"))

         // wrap getConstraint → returns Eigen::VectorXd (empty if missing)
         .def("get_constraint", [](const Vertex &v, int d)
              {
                   Eigen::VectorXd c;
                   if (v.getConstraint(d, &c))
                        return c;
                   else
                        return Eigen::VectorXd(); // empty indicates “not set”
              },
              nb::arg("derivative"));

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
              [](PolyOpt3D &opt)
              {
                   Segment::Vector segs;
                   opt.getSegments(&segs);
                   return segs;
              })
         .def("get_trajectory", [](PolyOpt3D &opt, Trajectory &traj)
              { opt.getTrajectory(&traj); }, nb::arg("trajectory"));

     // Segment
     nb::class_<Segment>(m, "Segment")
         .def("get_time", &Segment::getTime)

         // returns list of coefficient lists: [[c0,c1...], [...], ...]
         .def("get_polynomials",
              [](const Segment &s)
              {
                   const auto &vec = s.getPolynomialsRef();
                   std::vector<std::vector<double>> out;
                   out.reserve(vec.size());
                   for (const auto &poly : vec)
                   {
                        Eigen::VectorXd c = poly.getCoefficients();
                        out.emplace_back(c.data(), c.data() + c.size());
                   }
                   return out;
              });

     // NLOPT algorithm
     nb::enum_<nlopt::algorithm>(m, "NloptAlgorithm")
         // Global derivative‐free
         .value("GN_DIRECT", nlopt::GN_DIRECT)
         .value("GN_DIRECT_L", nlopt::GN_DIRECT_L)
         .value("GN_DIRECT_L_RAND", nlopt::GN_DIRECT_L_RAND)
         .value("GN_DIRECT_NOSCAL", nlopt::GN_DIRECT_NOSCAL)
         .value("GN_DIRECT_L_NOSCAL", nlopt::GN_DIRECT_L_NOSCAL)
         .value("GN_DIRECT_L_RAND_NOSCAL", nlopt::GN_DIRECT_L_RAND_NOSCAL)
         .value("GN_ORIG_DIRECT", nlopt::GN_ORIG_DIRECT)
         .value("GN_ORIG_DIRECT_L", nlopt::GN_ORIG_DIRECT_L)

         // Global gradient‐based
         .value("GD_STOGO", nlopt::GD_STOGO)
         .value("GD_STOGO_RAND", nlopt::GD_STOGO_RAND)
         .value("GD_MLSL", nlopt::GD_MLSL)
         .value("GD_MLSL_LDS", nlopt::GD_MLSL_LDS)

         // Local gradient‐based
         .value("LD_LBFGS_NOCEDAL", nlopt::LD_LBFGS_NOCEDAL)
         .value("LD_LBFGS", nlopt::LD_LBFGS)
         .value("LD_VAR1", nlopt::LD_VAR1)
         .value("LD_VAR2", nlopt::LD_VAR2)
         .value("LD_TNEWTON", nlopt::LD_TNEWTON)
         .value("LD_TNEWTON_RESTART", nlopt::LD_TNEWTON_RESTART)
         .value("LD_TNEWTON_PRECOND", nlopt::LD_TNEWTON_PRECOND)
         .value("LD_TNEWTON_PRECOND_RESTART", nlopt::LD_TNEWTON_PRECOND_RESTART)
         .value("LD_MMA", nlopt::LD_MMA)
         .value("LD_AUGLAG", nlopt::LD_AUGLAG)
         .value("LD_AUGLAG_EQ", nlopt::LD_AUGLAG_EQ)
         .value("LD_SLSQP", nlopt::LD_SLSQP)
         .value("LD_CCSAQ", nlopt::LD_CCSAQ)

         // Local derivative‐free
         .value("LN_PRAXIS", nlopt::LN_PRAXIS)
         .value("LN_COBYLA", nlopt::LN_COBYLA)
         .value("LN_NEWUOA", nlopt::LN_NEWUOA)
         .value("LN_NEWUOA_BOUND", nlopt::LN_NEWUOA_BOUND)
         .value("LN_NELDERMEAD", nlopt::LN_NELDERMEAD)
         .value("LN_SBPLX", nlopt::LN_SBPLX)
         .value("LN_AUGLAG", nlopt::LN_AUGLAG)
         .value("LN_AUGLAG_EQ", nlopt::LN_AUGLAG_EQ)
         .value("LN_BOBYQA", nlopt::LN_BOBYQA)

         // Mixed‐strategy & miscellaneous
         .value("GN_CRS2_LM", nlopt::GN_CRS2_LM)
         .value("GN_MLSL", nlopt::GN_MLSL)
         .value("GN_MLSL_LDS", nlopt::GN_MLSL_LDS)
         .value("AUGLAG", nlopt::AUGLAG)
         .value("AUGLAG_EQ", nlopt::AUGLAG_EQ)
         .value("G_MLSL", nlopt::G_MLSL)
         .value("G_MLSL_LDS", nlopt::G_MLSL_LDS)
         .value("GN_ISRES", nlopt::GN_ISRES)
         .value("GN_ESCH", nlopt::GN_ESCH)

         // sentinel
         .value("NUM_ALGORITHMS", nlopt::NUM_ALGORITHMS);


     // bind the TimeAllocMethod enum
     nb::enum_<NonlinearOptimizationParameters::TimeAllocMethod>(m, "TimeAllocMethod")
         .value("kSquaredTime", NonlinearOptimizationParameters::kSquaredTime)
         .value("kRichterTime", NonlinearOptimizationParameters::kRichterTime)
         .value("kMellingerOuterLoop", NonlinearOptimizationParameters::kMellingerOuterLoop)
         .value("kSquaredTimeAndConstraints", NonlinearOptimizationParameters::kSquaredTimeAndConstraints)
         .value("kRichterTimeAndConstraints", NonlinearOptimizationParameters::kRichterTimeAndConstraints)
         .value("kUnknown", NonlinearOptimizationParameters::kUnknown);

     // Nonlinear optimizer
     nb::class_<PolyOptNonLin<10>>(m, "PolynomialOptimizationNonLinear")
         .def(nb::init<size_t, const NonlinearOptimizationParameters &>(),
              nb::arg("dimension"), nb::arg("parameters"))
         .def("setup_from_vertices",
              &PolyOptNonLin<10>::setupFromVertices,
              nb::arg("vertices"), nb::arg("segment_times"),
              nb::arg("derivative_to_optimize"))
         .def("add_maximum_magnitude_constraint",
              &PolyOptNonLin<10>::addMaximumMagnitudeConstraint,
              nb::arg("derivative"), nb::arg("max_magnitude"))
         .def("solve_linear", &PolyOptNonLin<10>::solveLinear)
         .def("optimize", &PolyOptNonLin<10>::optimize)
         .def("get_trajectory", [](PolyOptNonLin<10> &opt, Trajectory &traj)
              { opt.getTrajectory(&traj); }, nb::arg("trajectory"))
         .def("get_optimization_info", &PolyOptNonLin<10>::getOptimizationInfo);

     // Nonlinear params
     nb::class_<NonlinearOptimizationParameters>(m, "NonlinearOptimizationParameters")
         .def(nb::init<>())
         .def_rw("f_abs", &NonlinearOptimizationParameters::f_abs)
         .def_rw("f_rel", &NonlinearOptimizationParameters::f_rel)
         .def_rw("x_rel", &NonlinearOptimizationParameters::x_rel)
         .def_rw("x_abs", &NonlinearOptimizationParameters::x_abs)
         .def_rw("initial_stepsize_rel", &NonlinearOptimizationParameters::initial_stepsize_rel)
         .def_rw("equality_constraint_tolerance", &NonlinearOptimizationParameters::equality_constraint_tolerance)
         .def_rw("inequality_constraint_tolerance", &NonlinearOptimizationParameters::inequality_constraint_tolerance)
         .def_rw("max_iterations", &NonlinearOptimizationParameters::max_iterations)
         .def_rw("time_penalty", &NonlinearOptimizationParameters::time_penalty)
         .def_rw("algorithm", &NonlinearOptimizationParameters::algorithm)
         .def_rw("random_seed", &NonlinearOptimizationParameters::random_seed)
         .def_rw("use_soft_constraints", &NonlinearOptimizationParameters::use_soft_constraints)
         .def_rw("soft_constraint_weight", &NonlinearOptimizationParameters::soft_constraint_weight)
         .def_rw("time_alloc_method", &NonlinearOptimizationParameters::time_alloc_method)
         .def_rw("print_debug_info", &NonlinearOptimizationParameters::print_debug_info)
         .def_rw("print_debug_info_time_allocation", &NonlinearOptimizationParameters::print_debug_info_time_allocation);


     // OptimizationInfo
     nb::class_<OptimizationInfo>(m, "OptimizationInfo")
         .def_ro("n_iterations", &OptimizationInfo::n_iterations)
         .def_ro("stopping_reason", &OptimizationInfo::stopping_reason)
         .def_ro("cost_trajectory", &OptimizationInfo::cost_trajectory)
         .def_ro("cost_time", &OptimizationInfo::cost_time)
         .def_ro("cost_soft_constraints", &OptimizationInfo::cost_soft_constraints)
         .def_ro("optimization_time", &OptimizationInfo::optimization_time)
         // maxima is std::map<int,Extremum>—you can bind it if needed
         ;

     // Trajectory
     nb::class_<Trajectory>(m, "Trajectory")
         .def(nb::init<>())
         .def("scale_segment_times_to_meet_constraints",
              &Trajectory::scaleSegmentTimesToMeetConstraints,
              nb::arg("v_max"), nb::arg("a_max"))
         .def("get_segments",
              [](const Trajectory &t)
              {
                   // use the const accessor and return a copy
                   // return t.segments();
                   Segment::Vector segs;
                   t.getSegments(&segs);
                   return segs;
              })
         .def("get_trajectory_with_appended_dimension", [](const Trajectory &in, const Trajectory &yaw, Trajectory &out)
              { in.getTrajectoryWithAppendedDimension(yaw, &out); }, nb::arg("yaw_trajectory"), nb::arg("out"));

     // Helpers
     m.def("nlopt_return_value_to_string", [](nlopt::algorithm v)
           { return nlopt::returnValueToString(v); }, nb::arg("return_value"));

}
