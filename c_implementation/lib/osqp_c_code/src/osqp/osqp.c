#include "../../include/osqp.h"
#include "../../include/auxil.h"
#include "../../include/util.h"
#include "../../include/scaling.h"
#include "../../include/glob_opts.h"
#include "../../include/error.h"


/**********************
* Main API Functions *
**********************/
void osqp_set_default_settings(OSQPSettings *settings) {

  settings->rho           = (c_float)RHO;            /* ADMM step */
  settings->sigma         = (c_float)SIGMA;          /* ADMM step */
  settings->scaling = SCALING;                       /* heuristic problem scaling */

  settings->max_iter      = MAX_ITER;                /* maximum iterations to
                                                        take */
  settings->eps_abs       = (c_float)EPS_ABS;        /* absolute convergence
                                                        tolerance */
  settings->eps_rel       = (c_float)EPS_REL;        /* relative convergence
                                                        tolerance */
  settings->eps_prim_inf  = (c_float)EPS_PRIM_INF;   /* primal infeasibility
                                                        tolerance */
  settings->eps_dual_inf  = (c_float)EPS_DUAL_INF;   /* dual infeasibility
                                                        tolerance */
  settings->alpha         = (c_float)ALPHA;          /* relaxation parameter */
  settings->linsys_solver = LINSYS_SOLVER;           /* relaxation parameter */


  settings->scaled_termination = SCALED_TERMINATION; /* Evaluate scaled
                                                        termination criteria*/
  settings->check_termination  = CHECK_TERMINATION;  /* Interval for evaluating
                                                        termination criteria */
  settings->warm_start         = WARM_START;         /* warm starting */

}


c_int osqp_solve(OSQPWorkspace *work) {

  c_int exitflag;
  c_int iter;
  c_int compute_cost_function; // Boolean: compute the cost function in the loop or not
  c_int can_check_termination; // Boolean: check termination or not


  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);


  // Initialize variables
  exitflag              = 0;
  can_check_termination = 0;
  compute_cost_function = 0;                       // Never compute cost
                                                   // function during the
                                                   // iterations if no printing
                                                   // enabled


  // Initialize variables (cold start or warm start depending on settings)
  if (!work->settings->warm_start) cold_start(work);  // If not warm start ->
                                                      // set x, z, y to zero

  // Main ADMM algorithm
  for (iter = 1; iter <= work->settings->max_iter; iter++) {
    // Update x_prev, z_prev (preallocated, no malloc)
    swap_vectors(&(work->x), &(work->x_prev));
    swap_vectors(&(work->z), &(work->z_prev));

    /* ADMM STEPS */
    /* Compute \tilde{x}^{k+1}, \tilde{z}^{k+1} */
    update_xz_tilde(work);

    /* Compute x^{k+1} */
    update_x(work);

    /* Compute z^{k+1} */
    update_z(work);

    /* Compute y^{k+1} */
    update_y(work);

    /* End of ADMM Steps */


    // Can we check for termination ?
    can_check_termination = work->settings->check_termination &&
                            (iter % work->settings->check_termination == 0);


    if (can_check_termination) {
      // Update information and compute also objective value
      update_info(work, iter, compute_cost_function, 0);

      // Check algorithm termination
      if (check_termination(work, 0)) {
        // Terminate algorithm
        break;
      }
    }


  }        // End of ADMM for loop


  // Update information and check termination condition if it hasn't been done
  // during last iteration (max_iter reached or check_termination disabled)
  if (!can_check_termination) {
    /* Update information */

    // If no printing is enabled, update info directly
    update_info(work, iter - 1, compute_cost_function, 0);


    /* Check whether a termination criterion is triggered */
    check_termination(work, 0);
  }

  // Compute objective value in case it was not
  // computed during the iterations
  if (!compute_cost_function && has_solution(work->info)){
    work->info->obj_val = compute_obj_val(work, work->x);
  }


  /* if max iterations reached, change status accordingly */
  if (work->info->status_val == OSQP_UNSOLVED) {
    if (!check_termination(work, 1)) { // Try to check for approximate
      update_status(work->info, OSQP_MAX_ITER_REACHED);
    }
  }


  /* Update solve time */


  // Store solution
  store_solution(work);


// Define exit flag for quitting function


  return exitflag;
}


/************************
* Update problem data  *
************************/
c_int osqp_update_lin_cost(OSQPWorkspace *work, const c_float *q_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);


  // Replace q by the new vector
  prea_vec_copy(q_new, work->data->q, work->data->n);

  // Scaling
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->D, work->data->q, work->data->q, work->data->n);
    vec_mult_scalar(work->data->q, work->scaling->c, work->data->n);
  }

  // Reset solver information
  reset_info(work->info);


  return 0;
}

c_int osqp_update_bounds(OSQPWorkspace *work,
                         const c_float *l_new,
                         const c_float *u_new) {
  c_int i, exitflag = 0;

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);


  // Check if lower bound is smaller than upper bound
  for (i = 0; i < work->data->m; i++) {
    if (l_new[i] > u_new[i]) {
      return 1;
    }
  }

  // Replace l and u by the new vectors
  prea_vec_copy(l_new, work->data->l, work->data->m);
  prea_vec_copy(u_new, work->data->u, work->data->m);

  // Scaling
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->E, work->data->l, work->data->l, work->data->m);
    vec_ew_prod(work->scaling->E, work->data->u, work->data->u, work->data->m);
  }

  // Reset solver information
  reset_info(work->info);


  return exitflag;
}

c_int osqp_update_lower_bound(OSQPWorkspace *work, const c_float *l_new) {
  c_int i, exitflag = 0;

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);


  // Replace l by the new vector
  prea_vec_copy(l_new, work->data->l, work->data->m);

  // Scaling
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->E, work->data->l, work->data->l, work->data->m);
  }

  // Check if lower bound is smaller than upper bound
  for (i = 0; i < work->data->m; i++) {
    if (work->data->l[i] > work->data->u[i]) {
      return 1;
    }
  }

  // Reset solver information
  reset_info(work->info);


  return exitflag;
}

c_int osqp_update_upper_bound(OSQPWorkspace *work, const c_float *u_new) {
  c_int i, exitflag = 0;

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);


  // Replace u by the new vector
  prea_vec_copy(u_new, work->data->u, work->data->m);

  // Scaling
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->E, work->data->u, work->data->u, work->data->m);
  }

  // Check if upper bound is greater than lower bound
  for (i = 0; i < work->data->m; i++) {
    if (work->data->u[i] < work->data->l[i]) {
      return 1;
    }
  }

  // Reset solver information
  reset_info(work->info);


  return exitflag;
}

c_int osqp_warm_start(OSQPWorkspace *work, const c_float *x, const c_float *y) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Update warm_start setting to true
  if (!work->settings->warm_start) work->settings->warm_start = 1;

  // Copy primal and dual variables into the iterates
  prea_vec_copy(x, work->x, work->data->n);
  prea_vec_copy(y, work->y, work->data->m);

  // Scale iterates
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->Dinv, work->x, work->x, work->data->n);
    vec_ew_prod(work->scaling->Einv, work->y, work->y, work->data->m);
    vec_mult_scalar(work->y, work->scaling->c, work->data->m);
  }

  // Compute Ax = z and store it in z
  mat_vec(work->data->A, work->x, work->z, 0);

  return 0;
}

c_int osqp_warm_start_x(OSQPWorkspace *work, const c_float *x) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Update warm_start setting to true
  if (!work->settings->warm_start) work->settings->warm_start = 1;

  // Copy primal variable into the iterate x
  prea_vec_copy(x, work->x, work->data->n);

  // Scale iterate
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->Dinv, work->x, work->x, work->data->n);
  }

  // Compute Ax = z and store it in z
  mat_vec(work->data->A, work->x, work->z, 0);

  return 0;
}

c_int osqp_warm_start_y(OSQPWorkspace *work, const c_float *y) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Update warm_start setting to true
  if (!work->settings->warm_start) work->settings->warm_start = 1;

  // Copy dual variable into the iterate y
  prea_vec_copy(y, work->y, work->data->m);

  // Scale iterate
  if (work->settings->scaling) {
    vec_ew_prod(work->scaling->Einv, work->y, work->y, work->data->m);
    vec_mult_scalar(work->y, work->scaling->c, work->data->m);
  }

  return 0;
}


/****************************
* Update problem settings  *
****************************/
c_int osqp_update_max_iter(OSQPWorkspace *work, c_int max_iter_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that max_iter is positive
  if (max_iter_new <= 0) {
    return 1;
  }

  // Update max_iter
  work->settings->max_iter = max_iter_new;

  return 0;
}

c_int osqp_update_eps_abs(OSQPWorkspace *work, c_float eps_abs_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that eps_abs is positive
  if (eps_abs_new < 0.) {
    return 1;
  }

  // Update eps_abs
  work->settings->eps_abs = eps_abs_new;

  return 0;
}

c_int osqp_update_eps_rel(OSQPWorkspace *work, c_float eps_rel_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that eps_rel is positive
  if (eps_rel_new < 0.) {
    return 1;
  }

  // Update eps_rel
  work->settings->eps_rel = eps_rel_new;

  return 0;
}

c_int osqp_update_eps_prim_inf(OSQPWorkspace *work, c_float eps_prim_inf_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that eps_prim_inf is positive
  if (eps_prim_inf_new < 0.) {
    return 1;
  }

  // Update eps_prim_inf
  work->settings->eps_prim_inf = eps_prim_inf_new;

  return 0;
}

c_int osqp_update_eps_dual_inf(OSQPWorkspace *work, c_float eps_dual_inf_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that eps_dual_inf is positive
  if (eps_dual_inf_new < 0.) {
    return 1;
  }

  // Update eps_dual_inf
  work->settings->eps_dual_inf = eps_dual_inf_new;


  return 0;
}

c_int osqp_update_alpha(OSQPWorkspace *work, c_float alpha_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that alpha is between 0 and 2
  if ((alpha_new <= 0.) || (alpha_new >= 2.)) {
    return 1;
  }

  // Update alpha
  work->settings->alpha = alpha_new;

  return 0;
}

c_int osqp_update_warm_start(OSQPWorkspace *work, c_int warm_start_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that warm_start is either 0 or 1
  if ((warm_start_new != 0) && (warm_start_new != 1)) {
    return 1;
  }

  // Update warm_start
  work->settings->warm_start = warm_start_new;

  return 0;
}

c_int osqp_update_scaled_termination(OSQPWorkspace *work, c_int scaled_termination_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that scaled_termination is either 0 or 1
  if ((scaled_termination_new != 0) && (scaled_termination_new != 1)) {
    return 1;
  }

  // Update scaled_termination
  work->settings->scaled_termination = scaled_termination_new;

  return 0;
}

c_int osqp_update_check_termination(OSQPWorkspace *work, c_int check_termination_new) {

  // Check if workspace has been initialized
  if (!work) return osqp_error(OSQP_WORKSPACE_NOT_INIT_ERROR);

  // Check that check_termination is nonnegative
  if (check_termination_new < 0) {
    return 1;
  }

  // Update check_termination
  work->settings->check_termination = check_termination_new;

  return 0;
}


