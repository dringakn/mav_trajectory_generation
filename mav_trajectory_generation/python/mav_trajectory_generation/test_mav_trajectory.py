#!/usr/bin/env python3

import mav_trajectory_generation as mtg
import numpy as np

# construct two 3‑D vertices
v0 = mtg.Vertex(3)
v1 = mtg.Vertex(3)

# build numpy arrays for the positions
pos0 = np.array([0.0, 0.0, 1.0], dtype=float)
pos1 = np.array([1.0, 0.0, 1.0], dtype=float)

# mark start/end constraints (snap) on each
v0.make_start_or_end_vec(pos0, mtg.derivative_order.SNAP)
v1.make_start_or_end_vec(pos1, mtg.derivative_order.SNAP)

print("v0 position constraint:", v0.get_constraint(0))

# estimate segment times (distance is non‑zero → time > 0)
times = mtg.estimate_segment_times([v0, v1], v_max=2.0, a_max=2.0)

# set up & solve a snap‑optimal trajectory in 3D
opt = mtg.PolynomialOptimization(3)
opt.setup_from_vertices([v0, v1], times, mtg.derivative_order.SNAP)
opt.solve_linear()

# inspect the first segment
segs = opt.get_segments()
print("First segment duration:", segs[0].get_time())
print("First segment polynomials:", segs[0].get_polynomials())
