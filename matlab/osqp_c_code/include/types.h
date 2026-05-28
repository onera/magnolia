#ifndef OSQP_TYPES_H
#define OSQP_TYPES_H


#include "glob_opts.h"
#include "constants.h"


/******************
* Internal types *
******************/

/**
 *  Matrix in compressed-column form.
 *  The structure is used internally to store matrices in the triplet form as well,
 *  but the API requires that the matrices are in the CSC format.
 */
typedef struct {
  c_int    nzmax; ///< maximum number of entries
  c_int    m;     ///< number of rows
  c_int    n;     ///< number of columns
  c_int   *p;     ///< column pointers (size n+1); col indices (size nzmax) start from 0 when using triplet format (direct KKT matrix formation)
  c_int   *i;     ///< row indices, size nzmax starting from 0
  c_float *x;     ///< numerical values, size nzmax
  c_int    nz;    ///< number of entries in triplet matrix, -1 for csc
} csc;

/**
 * Linear system solver structure (sublevel objects initialize it differently)
 */

typedef struct linsys_solver LinSysSolver;

/**
 * OSQP Timer for statistics
 */
typedef struct OSQP_TIMER OSQPTimer;

/**
 * Problem scaling matrices stored as vectors
 */
typedef struct {
  c_float  c;    ///< cost function scaling
  c_float *D;    ///< primal variable scaling
  c_float *E;    ///< dual variable scaling
  c_float  cinv; ///< cost function rescaling
  c_float *Dinv; ///< primal variable rescaling
  c_float *Einv; ///< dual variable rescaling
} OSQPScaling;

/**
 * Solution structure
 */
typedef struct {
  c_float *x; ///< primal solution
  c_float *y; ///< Lagrange multiplier associated to \f$l <= Ax <= u\f$
} OSQPSolution;


/**
 * Solver return information
 */
typedef struct {
  c_int iter;          ///< number of iterations taken
  char  status[32];    ///< status string, e.g. 'solved'
  c_int status_val;    ///< status as c_int, defined in constants.h


  c_float obj_val;     ///< primal objective
  c_float pri_res;     ///< norm of primal residual
  c_float dua_res;     ///< norm of dual residual


} OSQPInfo;


/**********************************
* Main structures and Data Types *
**********************************/

/**
 * Data structure
 */
typedef struct {
  c_int    n; ///< number of variables n
  c_int    m; ///< number of constraints m
  csc     *P; ///< the upper triangular part of the quadratic cost matrix P in csc format (size n x n).
  csc     *A; ///< linear constraints matrix A in csc format (size m x n)
  c_float *q; ///< dense array for linear part of cost function (size n)
  c_float *l; ///< dense array for lower bound (size m)
  c_float *u; ///< dense array for upper bound (size m)
} OSQPData;


/**
 * Settings struct
 */
typedef struct {
  c_float rho;                    ///< ADMM step rho
  c_float sigma;                  ///< ADMM step sigma
  c_int   scaling;                ///< heuristic data scaling iterations; if 0, then disabled.


  c_int                   max_iter;      ///< maximum number of iterations
  c_float                 eps_abs;       ///< absolute convergence tolerance
  c_float                 eps_rel;       ///< relative convergence tolerance
  c_float                 eps_prim_inf;  ///< primal infeasibility tolerance
  c_float                 eps_dual_inf;  ///< dual infeasibility tolerance
  c_float                 alpha;         ///< relaxation parameter
  enum linsys_solver_type linsys_solver; ///< linear system solver to use


  c_int scaled_termination;              ///< boolean, use scaled termination criteria
  c_int check_termination;               ///< integer, check termination interval; if 0, then termination checking is disabled
  c_int warm_start;                      ///< boolean, warm start

} OSQPSettings;


/**
 * OSQP Workspace
 */
typedef struct {
  /// Problem data to work on (possibly scaled)
  OSQPData *data;

  /// Linear System solver structure
  LinSysSolver *linsys_solver;


  /**
   * @name Vector used to store a vectorized rho parameter
   * @{
   */
  c_float *rho_vec;     ///< vector of rho values
  c_float *rho_inv_vec; ///< vector of inv rho values

  /** @} */


  /**
   * @name Iterates
   * @{
   */
  c_float *x;        ///< Iterate x
  c_float *y;        ///< Iterate y
  c_float *z;        ///< Iterate z
  c_float *xz_tilde; ///< Iterate xz_tilde

  c_float *x_prev;   ///< Previous x

  /**< NB: Used also as workspace vector for dual residual */
  c_float *z_prev;   ///< Previous z

  /**< NB: Used also as workspace vector for primal residual */

  /**
   * @name Primal and dual residuals workspace variables
   *
   * Needed for residuals computation, tolerances computation,
   * approximate tolerances computation and adapting rho
   * @{
   */
  c_float *Ax;  ///< scaled A * x
  c_float *Px;  ///< scaled P * x
  c_float *Aty; ///< scaled A' * y

  /** @} */

  /**
   * @name Primal infeasibility variables
   * @{
   */
  c_float *delta_y;   ///< difference between consecutive dual iterates
  c_float *Atdelta_y; ///< A' * delta_y

  /** @} */

  /**
   * @name Dual infeasibility variables
   * @{
   */
  c_float *delta_x;  ///< difference between consecutive primal iterates
  c_float *Pdelta_x; ///< P * delta_x
  c_float *Adelta_x; ///< A * delta_x

  /** @} */

  /**
   * @name Temporary vectors used in scaling
   * @{
   */

  c_float *D_temp;   ///< temporary primal variable scaling vectors
  c_float *D_temp_A; ///< temporary primal variable scaling vectors storing norms of A columns
  c_float *E_temp;   ///< temporary constraints scaling vectors storing norms of A' columns


  /** @} */

  OSQPSettings *settings; ///< problem settings
  OSQPScaling  *scaling;  ///< scaling vectors
  OSQPSolution *solution; ///< problem solution
  OSQPInfo     *info;     ///< solver information


} OSQPWorkspace;


/**
 * Define linsys_solver prototype structure
 *
 * NB: The details are defined when the linear solver is initialized depending
 *      on the choice
 */
struct linsys_solver {
  enum linsys_solver_type type;                 ///< linear system solver type functions
  c_int (*solve)(LinSysSolver *self,
                 c_float      *b);              ///< solve linear system


};


#endif // ifndef OSQP_TYPES_H
