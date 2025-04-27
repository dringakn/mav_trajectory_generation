#include <nanobind/nanobind.h>
#include <Eigen/Core>
#include <nanobind/eigen/dense.h> // Eigen ↔ NumPy converters
#include <nanobind/stl/vector.h>  // for std::vector<double> ↔ list
#include <yaml-cpp/yaml.h>

#include <mav_trajectory_generation/motion_defines.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/segment.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/io.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

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
     // using namespace mav_trajectory_generation::derivative_order;
     d.attr("POSITION") = mav_trajectory_generation::derivative_order::POSITION;
     d.attr("VELOCITY") = mav_trajectory_generation::derivative_order::VELOCITY;
     d.attr("ACCELERATION") = mav_trajectory_generation::derivative_order::ACCELERATION;
     d.attr("JERK") = mav_trajectory_generation::derivative_order::JERK;
     d.attr("SNAP") = mav_trajectory_generation::derivative_order::SNAP;
     d.attr("ORIENTATION") = mav_trajectory_generation::derivative_order::ORIENTATION;
     d.attr("ANGULAR_VELOCITY") = mav_trajectory_generation::derivative_order::ANGULAR_VELOCITY;
     d.attr("ANGULAR_ACCELERATION") = mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION;
     d.attr("INVALID") = mav_trajectory_generation::derivative_order::INVALID;

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

     /*
     estimateSegmentTimes
          Makes a rough estimate based on v_max and a_max about the time
          required to get from one vertex to the next.
          Calculate the velocity assuming instantaneous constant acceleration a_max
          and straight line rest-to-rest trajectories.
          The time_factor \in [1..Inf] increases the allocated time making the segments
          slower and thus feasibility more likely. This method does not take into
          account the start and goal velocity and acceleration.     
     */
     m.def("estimate_segment_times",
           &estimateSegmentTimes,
           nb::arg("vertices"),
           nb::arg("v_max"),
           nb::arg("a_max"));

     /*
     estimateSegmentTimesNfabian
          Makes a rough estimate based on v_max and a_max about the time
          required to get from one vertex to the next.
          t_est = 2 * distance/v_max * (1 + magic_fabian_constant * v_max/a_max * exp(-distance/v_max * 2);
          magic_fabian_constant was determined to 6.5 in a student project ...
     */           
     m.def("estimate_segment_times_nfabian",
           &estimateSegmentTimesNfabian,
           nb::arg("vertices"),
           nb::arg("v_max"),
           nb::arg("a_max"),
           nb::arg("magic_fabian_constant") = 6.5);

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
          // evaluation
          .def("evaluate",
               &Trajectory::evaluate,
               nb::arg("t"),
               nb::arg("derivative_order"))
          .def("evaluate_range",
               [](const Trajectory &self,
                    double t_start, double t_end, double dt, int der) {
                    std::vector<Eigen::VectorXd> out_vals;
                    std::vector<double> out_times;
                    self.evaluateRange(
                         t_start, t_end, dt, der,
                         &out_vals, &out_times);
                    return std::make_pair(out_vals, out_times);
               },
               nb::arg("t_start"),
               nb::arg("t_end"),
               nb::arg("dt"),
               nb::arg("derivative_order"))
          .def("get_segments",
              [](const Trajectory &t)
              {
                   // use the const accessor and return a copy
                   // return t.segments();
                   Segment::Vector segs;
                   t.getSegments(&segs);
                   return segs;
              })
          .def("get_trajectory_with_single_dimension", &Trajectory::getTrajectoryWithSingleDimension, nb::arg("dimension"))
          .def("get_trajectory_with_appended_dimension", [](const Trajectory &in, const Trajectory &yaw, Trajectory &out)
              { in.getTrajectoryWithAppendedDimension(yaw, &out); }, 
               nb::arg("yaw_trajectory"), nb::arg("out"))
          // timing / scaling
          .def("get_segment_times",
               &Trajectory::getSegmentTimes)

          .def("scale_segment_times",
               &Trajectory::scaleSegmentTimes,
               nb::arg("scaling"))

          .def("scale_segment_times_to_meet_constraints",
               &Trajectory::scaleSegmentTimesToMeetConstraints,
               nb::arg("v_max"),
               nb::arg("a_max"))

          // kinematic limits
          .def("compute_max_velocity_and_acceleration",
               [](const Trajectory &self) {
               double v_max, a_max;
               self.computeMaxVelocityAndAcceleration(&v_max, &a_max);
               return std::make_pair(v_max, a_max);
               })

          // merging & offsets
          .def("add_trajectories",
               [](const Trajectory &self,
               const std::vector<Trajectory> &list) {
               Trajectory merged;
               self.addTrajectories(list, &merged);
               return merged;
               },
               nb::arg("trajectories"))

          .def("offset_trajectory",
               &Trajectory::offsetTrajectory,
               nb::arg("A_r_B"))              
          ;

     // Helpers
     m.def("nlopt_return_value_to_string", [](nlopt::algorithm v)
           { return nlopt::returnValueToString(v); }, nb::arg("return_value"));

     // --- YAML serialization
     m.def("trajectory_to_yaml",
          [](const Trajectory &traj) {
          YAML::Node node = trajectoryToYaml(traj);
          YAML::Emitter out;
          out << node;
          return std::string(out.c_str());
          },
          nb::arg("trajectory"),
          "Serialize a Trajectory into a YAML string.");

     m.def("trajectory_from_yaml",
          [](const std::string &yaml_str) {
          YAML::Node node = YAML::Load(yaml_str);
          Trajectory traj;
          if (!trajectoryFromYaml(node, &traj))
               throw std::runtime_error("Failed to parse trajectory YAML");
          return traj;
          },
          nb::arg("yaml"),
          "Parse a YAML string into a new Trajectory.");

     m.def("segments_to_yaml",
          [](const std::vector<Segment> &segs) {
          YAML::Node node = segmentsToYaml(segs);
          YAML::Emitter out; out << node;
          return std::string(out.c_str());
          },
          nb::arg("segments"),
          "Serialize a list of Segments to a YAML string.");

     m.def("segments_from_yaml",
          [](const std::string &yaml_str) {
          YAML::Node node = YAML::Load(yaml_str);
          std::vector<Segment> segs;
          if (!segmentsFromYaml(node, &segs))
               throw std::runtime_error("Failed to parse segments YAML");
          return segs;
          },
          nb::arg("yaml"),
          "Parse a YAML string into a list of Segments.");

     // --- File I/O
     m.def("write_segments",
          [](const std::string &fn, const std::vector<Segment> &segs) {
          if (!segmentsToFile(fn, segs))
               throw std::runtime_error("Could not write segments to " + fn);
          },
          nb::arg("filename"), nb::arg("segments"),
          "Write a list of Segments to a YAML file.");

     m.def("read_segments",
          [](const std::string &fn) {
          std::vector<Segment> segs;
          if (!segmentsFromFile(fn, &segs))
               throw std::runtime_error("Could not read segments from " + fn);
          return segs;
          },
          nb::arg("filename"),
          "Read a list of Segments from a YAML file.");

     // --- Trajectory sampling to MATLAB-style file
     m.def("write_sampled_trajectory",
          &sampledTrajectoryStatesToFile,
          nb::arg("filename"),
          nb::arg("trajectory"),
          "Sample a Trajectory at 0.01 s and dump to a text file.");

     // (Optional) expose the low-level sampler itself:
     m.def("sample_whole_trajectory",
          [](const Trajectory &traj, double dt){
          mav_msgs::EigenTrajectoryPoint::Vector pts;
          if (!sampleWholeTrajectory(traj, dt, &pts))
               throw std::runtime_error("Trajectory sampling failed");
          return pts;
          },
          nb::arg("trajectory"), nb::arg("dt"),
          "Return a list of EigenTrajectoryPoint sampled every dt seconds.");
}
