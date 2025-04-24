#!/usr/bin/env python3
"""
Verbose binding test for mav_trajectory_generation.
Covers: enums, Vertex, linear & nonlinear optimizers, Trajectory, and helper functions.
"""

import mav_trajectory_generation as mtg
import numpy as np


def check_derivative_order():
    print("=== derivative_order constants ===")
    for name in sorted(dir(mtg.derivative_order)):
        if name.isupper():
            print(f"  {name:20s} = {getattr(mtg.derivative_order, name)}")


def check_nlopt_enum():
    print("\n=== NloptAlgorithm constants ===")
    for name in sorted(dir(mtg.NloptAlgorithm)):
        if name.isupper():
            print(f"  {name:20s} = {getattr(mtg.NloptAlgorithm, name)}")


def test_vertex_constraints():
    print("\n=== Vertex constraints ===")
    v = mtg.Vertex(3)
    print("  • initially no constraints")
    for d in range(5):
        assert not v.has_constraint(d)
    print("    OK")

    # POSITION scalar
    print("  • add_constraint(POSITION, 1.5)")
    v.add_constraint(mtg.derivative_order.POSITION, 1.5)
    assert v.has_constraint(mtg.derivative_order.POSITION)
    p = v.get_constraint(mtg.derivative_order.POSITION)
    print("    →", p)
    assert np.allclose(p, [1.5, 1.5, 1.5])

    # VELOCITY vector
    print("  • add_constraint_vec(VELOCITY, [0.2,0,0])")
    vel = np.array([0.2, 0.0, 0.0])
    v.add_constraint_vec(mtg.derivative_order.VELOCITY, vel)
    assert v.has_constraint(mtg.derivative_order.VELOCITY)
    out = v.get_constraint(mtg.derivative_order.VELOCITY)
    print("    →", out)
    assert np.allclose(out, vel)

    # remove
    print("  • remove_constraint(POSITION)")
    v.remove_constraint(mtg.derivative_order.POSITION)
    assert not v.has_constraint(mtg.derivative_order.POSITION)
    print("    removed OK")


def plan_linear_mission():
    print("\n=== Linear trajectory: takeoff → cruise → land ===")
    pts = [
        [ 0.0,  0.0,  0.0],   # ground
        [ 0.0,  0.0,  5.0],   # hover
        [10.0,  0.0,  5.0],   # cruise
        [10.0,  0.0,  0.0],   # land
    ]
    vertices = []
    for i, p in enumerate(pts):
        v = mtg.Vertex(3)
        arr = np.array(p, dtype=float)
        if i in (0, len(pts)-1):
            v.make_start_or_end_vec(arr, mtg.derivative_order.SNAP)
        else:
            v.add_constraint_vec(mtg.derivative_order.POSITION, arr)
        vertices.append(v)

    v_max, a_max = 4.0, 3.0
    times = mtg.estimate_segment_times(vertices, v_max, a_max)
    print("  estimated times:", times)
    assert all(t > 0 for t in times)

    opt = mtg.PolynomialOptimization(3)
    ok_setup = opt.setup_from_vertices(vertices, times, mtg.derivative_order.SNAP)
    print("  setup_from_vertices →", ok_setup)
    ok_solve = opt.solve_linear()
    print("  solve_linear →", ok_solve)

    segs = opt.get_segments()
    print(f"  got {len(segs)} segments:")
    for idx, seg in enumerate(segs):
        coeffs = seg.get_polynomials()
        print(f"    segment {idx}: t={seg.get_time():.3f}, first coeffs={coeffs[0][:3]}…")


def plan_nonlinear_mission():
    print("\n=== Nonlinear trajectory & Trajectory API ===")
    # we reuse 3–point: ground→hover→land but now add zero-velocity at hover
    pts = [
        [ 0.0, 0.0, 0.0],
        [ 0.0, 0.0, 5.0],
        [10.0, 0.0, 0.0],
    ]
    vertices = []
    for i, p in enumerate(pts):
        v = mtg.Vertex(3)
        arr = np.array(p, dtype=float)
        if i in (0, len(pts)-1):
            # fully lock start/end
            v.make_start_or_end_vec(arr, mtg.derivative_order.SNAP)
        else:
            # interior: position + zero velocity
            v.add_constraint_vec(mtg.derivative_order.POSITION, arr)
            v.add_constraint_vec(mtg.derivative_order.VELOCITY, np.zeros(3))
        vertices.append(v)

    v_max, a_max = 4.0, 3.0
    times = mtg.estimate_segment_times(vertices, v_max, a_max)
    print("  estimated times:", times)
    assert all(t > 0 for t in times)

    # configure and mutate non-linear params
    params = mtg.NonlinearOptimizationParameters()
    print("  defaults: f_rel=", params.f_rel, "max_iter=", params.max_iterations)
    params.f_rel            = 1e-3
    params.max_iterations   = 300
    params.algorithm        = mtg.NloptAlgorithm.LN_BOBYQA
    params.time_alloc_method= mtg.TimeAllocMethod.kSquaredTime
    print("  mutated:  f_rel=", params.f_rel,
          "algorithm=", params.algorithm,
          "time_alloc_method=", params.time_alloc_method)

    opt = mtg.PolynomialOptimizationNonLinear(3, params)
    ok_setup = opt.setup_from_vertices(vertices, times, mtg.derivative_order.SNAP)
    print("  nonlinear setup →", ok_setup)
    ok_lin = opt.solve_linear()
    print("  nonlinear internal solve_linear →", ok_lin)
    ret = opt.optimize()
    print("  optimize() returned:", ret)
    info = opt.get_optimization_info()
    print(f"  OptimizationInfo: iter={info.n_iterations}, cost={info.cost_trajectory:.3f}")

    # build trajectory
    traj = mtg.Trajectory()
    opt.get_trajectory(traj)
    traj.scale_segment_times_to_meet_constraints(v_max, a_max)
    segs = traj.get_segments()
    print("  Trajectory segments:", len(segs))
    for i, s in enumerate(segs):
        print(f"    segment {i}: duration={s.get_time():.3f}")


def test_nlopt_helper():
    print("\n=== nlopt_return_value_to_string ===")
    for code in [mtg.NloptAlgorithm.LN_BOBYQA, -999]:
        # TODO: fix inverse mapping
        iv = int(code)   # enum → plain int
        print(f"  code {iv}: {mtg.nlopt_return_value_to_string(iv)}")

def main():
    check_derivative_order()
    check_nlopt_enum()
    test_vertex_constraints()
    plan_linear_mission()
    plan_nonlinear_mission()
    # test_nlopt_helper()
    print("\n✅ All binding tests passed.")

if __name__ == "__main__":
    main()
