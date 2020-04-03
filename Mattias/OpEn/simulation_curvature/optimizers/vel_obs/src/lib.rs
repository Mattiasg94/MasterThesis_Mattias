//
// Auto-generated file by OptimizationEngine
// See https://alphaville.github.io/optimization-engine/
//
// Generated at: 2020-04-02 12:13:36.360374
//

use icasadi;

use optimization_engine::{constraints::*, panoc::*, alm::*, *};

// ---Private Constants----------------------------------------------------------------------------------

/// Tolerance of inner solver
const EPSILON_TOLERANCE: f64 = 0.001;

/// Initial tolerance
const INITIAL_EPSILON_TOLERANCE: f64 = 0.001;

/// Update factor for inner tolerance
const EPSILON_TOLERANCE_UPDATE_FACTOR: f64 = 0.1;

/// Delta tolerance
const DELTA_TOLERANCE: f64 = 0.01;

/// LBFGS memory
const LBFGS_MEMORY: usize = 10;

/// Maximum number of iterations of the inner solver
const MAX_INNER_ITERATIONS: usize = 500;

/// Maximum number of outer iterations
const MAX_OUTER_ITERATIONS: usize = 6;

/// Maximum execution duration in microseconds
const MAX_DURATION_MICROS: u64 = 5000000;

/// Penalty update factor
const PENALTY_UPDATE_FACTOR: f64 = 2.0;

/// Initial penalty
const INITIAL_PENALTY_PARAMETER: f64 = 100.0;

/// Sufficient decrease coefficient
const SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT: f64 = 0.1;


// ---Public Constants-----------------------------------------------------------------------------------

/// Number of decision variables
pub const VEL_OBS_NUM_DECISION_VARIABLES: usize = 15;

/// Number of parameters
pub const VEL_OBS_NUM_PARAMETERS: usize = 62;

/// Number of parameters associated with augmented Lagrangian
pub const VEL_OBS_N1: usize = 1;

/// Number of penalty constraints
pub const VEL_OBS_N2: usize = 0;



// ---Parameters of the constraints----------------------------------------------------------------------

const CONSTRAINTS_XMIN :Option<&[f64]> = Some(&[0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,]);
const CONSTRAINTS_XMAX :Option<&[f64]> = Some(&[0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,]);





// ---Parameters of ALM-type constraints (Set C)---------------------------------------------------------




// ---Parameters of ALM-type constraints (Set Y)---------------------------------------------------------
/// Constraints: Centre of Euclidean Ball
const SET_Y_BALL_XC: Option<&[f64]> = None;

/// Constraints: Radius of Euclidean Ball
const SET_Y_BALL_RADIUS : f64 = 1000000000000.0;




// ---Internal private helper functions------------------------------------------------------------------

/// Make constraints U
fn make_constraints() -> impl Constraint {

    let bounds = Rectangle::new(CONSTRAINTS_XMIN, CONSTRAINTS_XMAX);
    bounds
}

/// Make set C
fn make_set_c() -> impl Constraint {
    let set_c = Zero::new();
    set_c
}


/// Make set Y
fn make_set_y() -> impl Constraint {
    let set_y = BallInf::new(SET_Y_BALL_XC, SET_Y_BALL_RADIUS);
    set_y
}


// ---Main public API functions--------------------------------------------------------------------------


/// Initialisation of the solver
pub fn initialize_solver() -> AlmCache {
    let panoc_cache = PANOCCache::new(VEL_OBS_NUM_DECISION_VARIABLES, EPSILON_TOLERANCE, LBFGS_MEMORY);
    let alm_cache = AlmCache::new(panoc_cache, VEL_OBS_N1, VEL_OBS_N2);

    alm_cache
}


/// Solver interface
pub fn solve(
    p: &[f64],
    alm_cache: &mut AlmCache,
    u: &mut [f64],
    y0: &Option<Vec<f64>>,
    c0: &Option<f64>,
) -> Result<AlmOptimizerStatus, SolverError> {

    assert_eq!(p.len(), VEL_OBS_NUM_PARAMETERS, "Wrong number of parameters (p)");
    assert_eq!(u.len(), VEL_OBS_NUM_DECISION_VARIABLES, "Wrong number of decision variables (u)");

    let psi = |u: &[f64], xi: &[f64], cost: &mut f64| -> Result<(), SolverError> {
        icasadi::cost(&u, &xi, &p, cost);
        Ok(())
    };
    let grad_psi = |u: &[f64], xi: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        icasadi::grad(&u, &xi, &p, grad);
        Ok(())
    };
    
    let f1 = |u: &[f64], res: &mut [f64]| -> Result<(), SolverError> {
        icasadi::mapping_f1(&u, &p, res);
        Ok(())
    };
    let bounds = make_constraints();

    let set_y = make_set_y();
    let set_c = make_set_c();
    let alm_problem = AlmProblem::new(
        bounds,
        Some(set_c),
        Some(set_y),
        psi,
        grad_psi,
        Some(f1),
        NO_MAPPING,
        VEL_OBS_N1,
        VEL_OBS_N2,
    );

    let mut alm_optimizer = AlmOptimizer::new(alm_cache, alm_problem)
        .with_delta_tolerance(DELTA_TOLERANCE)
        .with_epsilon_tolerance(EPSILON_TOLERANCE)
        .with_initial_inner_tolerance(INITIAL_EPSILON_TOLERANCE)
        .with_inner_tolerance_update_factor(EPSILON_TOLERANCE_UPDATE_FACTOR)
        .with_max_duration(std::time::Duration::from_micros(MAX_DURATION_MICROS))
        .with_max_outer_iterations(MAX_OUTER_ITERATIONS)
        .with_max_inner_iterations(MAX_INNER_ITERATIONS)
        .with_initial_penalty(c0.unwrap_or(INITIAL_PENALTY_PARAMETER))
        .with_penalty_update_factor(PENALTY_UPDATE_FACTOR)
        .with_sufficient_decrease_coefficient(SUFFICIENT_INFEASIBILITY_DECREASE_COEFFICIENT);

    // solve the problem using `u` an the initial condition `u` and
    // initial vector of Lagrange multipliers, if provided;
    // returns the problem status (instance of `AlmOptimizerStatus`)
    if let Some(y0_) = y0 {
        let mut alm_optimizer = alm_optimizer.with_initial_lagrange_multipliers(&y0_);
        alm_optimizer.solve(u)
    } else {
        alm_optimizer.solve(u)
    }

}