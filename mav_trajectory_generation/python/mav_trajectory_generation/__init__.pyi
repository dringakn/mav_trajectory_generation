# mav_trajectory_generation/__init__.pyi

from typing import List, Sequence, Union
import numpy as np

# derivative_order submodule
class derivative_order:
    POSITION: int
    VELOCITY: int
    ACCELERATION: int
    JERK: int
    SNAP: int
    ORIENTATION: int
    ANGULAR_VELOCITY: int
    ANGULAR_ACCELERATION: int
    INVALID: int

# NloptAlgorithm enum
class NloptAlgorithm:
    # Global derivative‐free
    GN_DIRECT: int
    GN_DIRECT_L: int
    GN_DIRECT_L_RAND: int
    GN_DIRECT_NOSCAL: int
    GN_DIRECT_L_NOSCAL: int
    GN_DIRECT_L_RAND_NOSCAL: int
    GN_ORIG_DIRECT: int
    GN_ORIG_DIRECT_L: int

    # Global gradient‐based
    GD_STOGO: int
    GD_STOGO_RAND: int
    GD_MLSL: int
    GD_MLSL_LDS: int

    # Local gradient‐based
    LD_LBFGS_NOCEDAL: int
    LD_LBFGS: int
    LD_VAR1: int
    LD_VAR2: int
    LD_TNEWTON: int
    LD_TNEWTON_RESTART: int
    LD_TNEWTON_PRECOND: int
    LD_TNEWTON_PRECOND_RESTART: int
    LD_MMA: int
    LD_AUGLAG: int
    LD_AUGLAG_EQ: int
    LD_SLSQP: int
    LD_CCSAQ: int

    # Local derivative‐free
    LN_PRAXIS: int
    LN_COBYLA: int
    LN_NEWUOA: int
    LN_NEWUOA_BOUND: int
    LN_NELDERMEAD: int
    LN_SBPLX: int
    LN_AUGLAG: int
    LN_AUGLAG_EQ: int
    LN_BOBYQA: int

    # Mixed‐strategy & miscellaneous
    GN_CRS2_LM: int
    GN_MLSL: int
    GN_MLSL_LDS: int
    AUGLAG: int
    AUGLAG_EQ: int
    G_MLSL: int
    G_MLSL_LDS: int
    GN_ISRES: int
    GN_ESCH: int
    
    # sentinel
    NUM_ALGORITHMS: int

# TimeAllocMethod enum
class TimeAllocMethod:
    kSquaredTime: int
    kRichterTime: int
    kMellingerOuterLoop: int
    kSquaredTimeAndConstraints: int
    kRichterTimeAndConstraints: int
    kUnknown: int

class Vertex:
    def __init__(self, dimension: int) -> None: ...
    def add_constraint(self, derivative: int, value: float) -> None: ...
    def add_constraint_vec(self, derivative: int, values: np.ndarray) -> None: ...
    def make_start_or_end(self, value: float, up_to_derivative: int) -> None: ...
    def make_start_or_end_vec(self, values: np.ndarray, up_to_derivative: int) -> None: ...
    def remove_constraint(self, derivative: int) -> None: ...
    def has_constraint(self, derivative: int) -> bool: ...
    def get_constraint(self, derivative: int) -> np.ndarray: ...

def estimate_segment_times(
    vertices: List[Vertex],
    v_max: float,
    a_max: float
) -> List[float]: ...

class PolynomialOptimization:
    def __init__(self, dimension: int) -> None: ...
    def setup_from_vertices(
        self,
        vertices: List[Vertex],
        segment_times: Sequence[float],
        derivative_to_optimize: int
    ) -> None: ...
    def solve_linear(self) -> None: ...
    def get_segments(self) -> List["Segment"]: ...

class Segment:
    def get_time(self) -> float: ...
    def get_polynomials(self) -> List[List[float]]: ...

class NonlinearOptimizationParameters:
    f_abs: float
    f_rel: float
    x_rel: float
    x_abs: float
    initial_stepsize_rel: float
    equality_constraint_tolerance: float
    inequality_constraint_tolerance: float
    max_iterations: int
    time_penalty: float
    algorithm: NloptAlgorithm
    random_seed: int
    use_soft_constraints: bool
    soft_constraint_weight: float
    time_alloc_method: TimeAllocMethod
    print_debug_info: bool
    print_debug_info_time_allocation: bool

    def __init__(self) -> None: ...

class PolynomialOptimizationNonLinear:
    def __init__(self, dimension: int, parameters: NonlinearOptimizationParameters) -> None: ...
    def setup_from_vertices(
        self,
        vertices: List[Vertex],
        segment_times: Sequence[float],
        derivative_to_optimize: int
    ) -> bool: ...
    def add_maximum_magnitude_constraint(self, derivative: int, max_magnitude: float) -> bool: ...
    def solve_linear(self) -> bool: ...
    def optimize(self) -> int: ...
    def get_trajectory(self, trajectory: "Trajectory") -> None: ...
    def get_optimization_info(self) -> "OptimizationInfo": ...

class OptimizationInfo:
    n_iterations: int
    stopping_reason: int
    cost_trajectory: float
    cost_time: float
    cost_soft_constraints: float
    optimization_time: float

    # if you bound `maxima`, you can add:
    # maxima: Dict[int, Extremum]

class Trajectory:
    def __init__(self) -> None: ...
    def scale_segment_times_to_meet_constraints(self, v_max: float, a_max: float) -> None: ...
    def get_segments(self) -> List[Segment]: ...

def nlopt_return_value_to_string(return_value: int) -> str: ...

__all__ = [
    "Vertex",
    "PolynomialOptimization",
    "estimate_segment_times",
    "Segment",
    "derivative_order",
    "NloptAlgorithm",
    "NonlinearOptimizationParameters",
    "TimeAllocMethod",
    "PolynomialOptimizationNonLinear",
    "OptimizationInfo",
    "Trajectory",
    "nlopt_return_value_to_string",
]
