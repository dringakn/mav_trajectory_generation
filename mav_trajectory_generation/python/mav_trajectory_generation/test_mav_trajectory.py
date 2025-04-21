#!/usr/bin/env python3

import mav_trajectory as mtg

v = mtg.Vertex(3)
v.make_start_or_end_vec([0,0,1], mtg.derivative_order.SNAP)
print(v.get_constraint(0))  # Eigen vector

times = mtg.estimate_segment_times([v, v], v_max=2.0, a_max=2.0)

opt = mtg.PolynomialOptimization(3)
opt.setup_from_vertices([v, v], times, mtg.derivative_order.SNAP)
opt.solve_linear()
segs = opt.get_segments()
print(segs[0].get_time(), segs[0].get_polynomials())
