#ifndef OSQP_H
#define OSQP_H


/* Includes */
#include "types.h"
#include "util.h" // Needed for osqp_set_default_settings functions


// Library to deal with sparse matrices enabled only if embedded not defined

/********************
* Main Solver API  *
********************/

/**
 * @name Main solver API
 * @{
 */

/**
 * Set default settings from constants.h file
 * assumes settings already allocated in memory
 * @param settings settings structure
 */
void osqp_set_default_settings(OSQPSettings *settings);


/**
 * Solve quadratic program
 *
 * The final solver information is stored in the \a work->info  structure
 *
 * The solution is stored in the  \a work->solution  structure
 *
 * If the problem is primal infeasible, the certificate is stored
 * in \a work->delta_y
 *
 * If the problem is dual infeasible, the certificate is stored in \a
 * work->delta_x
 *
 * @param  work Workspace allocated
 * @return      Exitflag for errors
 */
c_int osqp_solve(OSQPWorkspace *work);


/** @} */


/********************************************
* Sublevel API                             *
*                                          *
* Edit data without performing setup again *
********************************************/

/**
 * @name Sublevel API
 * @{
 */

/**
 * Update linear cost in the problem
 * @param  work  Workspace
 * @param  q_new New linear cost
 * @return       Exitflag for errors and warnings
 */
c_int osqp_update_lin_cost(OSQPWorkspace *work,
                           const c_float *q_new);


/**
 * Update lower and upper bounds in the problem constraints
 * @param  work   Workspace
 * @param  l_new New lower bound
 * @param  u_new New upper bound
 * @return        Exitflag: 1 if new lower bound is not <= than new upper bound
 */
c_int osqp_update_bounds(OSQPWorkspace *work,
                         const c_float *l_new,
                         const c_float *u_new);


/**
 * Update lower bound in the problem constraints
 * @param  work   Workspace
 * @param  l_new New lower bound
 * @return        Exitflag: 1 if new lower bound is not <= than upper bound
 */
c_int osqp_update_lower_bound(OSQPWorkspace *work,
                              const c_float *l_new);


/**
 * Update upper bound in the problem constraints
 * @param  work   Workspace
 * @param  u_new New upper bound
 * @return        Exitflag: 1 if new upper bound is not >= than lower bound
 */
c_int osqp_update_upper_bound(OSQPWorkspace *work,
                              const c_float *u_new);


/**
 * Warm start primal and dual variables
 * @param  work Workspace structure
 * @param  x    Primal variable
 * @param  y    Dual variable
 * @return      Exitflag
 */
c_int osqp_warm_start(OSQPWorkspace *work,
                      const c_float *x,
                      const c_float *y);


/**
 * Warm start primal variable
 * @param  work Workspace structure
 * @param  x    Primal variable
 * @return      Exitflag
 */
c_int osqp_warm_start_x(OSQPWorkspace *work,
                        const c_float *x);


/**
 * Warm start dual variable
 * @param  work Workspace structure
 * @param  y    Dual variable
 * @return      Exitflag
 */
c_int osqp_warm_start_y(OSQPWorkspace *work,
                        const c_float *y);


/** @} */


/**
 * @name Update settings
 * @{
 */


/**
 * Update max_iter setting
 * @param  work         Workspace
 * @param  max_iter_new New max_iter setting
 * @return              Exitflag
 */
c_int osqp_update_max_iter(OSQPWorkspace *work,
                           c_int          max_iter_new);


/**
 * Update absolute tolernace value
 * @param  work        Workspace
 * @param  eps_abs_new New absolute tolerance value
 * @return             Exitflag
 */
c_int osqp_update_eps_abs(OSQPWorkspace *work,
                          c_float        eps_abs_new);


/**
 * Update relative tolernace value
 * @param  work        Workspace
 * @param  eps_rel_new New relative tolerance value
 * @return             Exitflag
 */
c_int osqp_update_eps_rel(OSQPWorkspace *work,
                          c_float        eps_rel_new);


/**
 * Update primal infeasibility tolerance
 * @param  work          Workspace
 * @param  eps_prim_inf_new  New primal infeasibility tolerance
 * @return               Exitflag
 */
c_int osqp_update_eps_prim_inf(OSQPWorkspace *work,
                               c_float        eps_prim_inf_new);


/**
 * Update dual infeasibility tolerance
 * @param  work          Workspace
 * @param  eps_dual_inf_new  New dual infeasibility tolerance
 * @return               Exitflag
 */
c_int osqp_update_eps_dual_inf(OSQPWorkspace *work,
                               c_float        eps_dual_inf_new);


/**
 * Update relaxation parameter alpha
 * @param  work  Workspace
 * @param  alpha_new New relaxation parameter value
 * @return       Exitflag
 */
c_int osqp_update_alpha(OSQPWorkspace *work,
                        c_float        alpha_new);


/**
 * Update warm_start setting
 * @param  work           Workspace
 * @param  warm_start_new New warm_start setting
 * @return                Exitflag
 */
c_int osqp_update_warm_start(OSQPWorkspace *work,
                             c_int          warm_start_new);


/**
 * Update scaled_termination setting
 * @param  work                 Workspace
 * @param  scaled_termination_new  New scaled_termination setting
 * @return                      Exitflag
 */
c_int osqp_update_scaled_termination(OSQPWorkspace *work,
                                     c_int          scaled_termination_new);

/**
 * Update check_termination setting
 * @param  work                   Workspace
 * @param  check_termination_new  New check_termination setting
 * @return                        Exitflag
 */
c_int osqp_update_check_termination(OSQPWorkspace *work,
                                    c_int          check_termination_new);


/** @} */


#endif // ifndef OSQP_H
