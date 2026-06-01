#include "osqp.h" // For OSQP rho update
#include "auxil.h"
#include "proj.h"
#include "lin_alg.h"
#include "constants.h"
#include "scaling.h"
#include "util.h"

/***********************************************************
* Auxiliary functions needed to compute ADMM iterations * *
***********************************************************/


void swap_vectors(c_float **a, c_float **b) {
  c_float *temp;

  temp = *b;
  *b   = *a;
  *a   = temp;
}

void cold_start(OSQPWorkspace *work) {
  vec_set_scalar(work->x, 0., work->data->n);
  vec_set_scalar(work->z, 0., work->data->m);
  vec_set_scalar(work->y, 0., work->data->m);
}

static void compute_rhs(OSQPWorkspace *work) {
  c_int i; // Index

  for (i = 0; i < work->data->n; i++) {
    // Cycle over part related to x variables
    work->xz_tilde[i] = work->settings->sigma * work->x_prev[i] -
                        work->data->q[i];
  }

  for (i = 0; i < work->data->m; i++) {
    // Cycle over dual variable in the first step (nu)
    work->xz_tilde[i + work->data->n] = work->z_prev[i] - work->rho_inv_vec[i] *
                                        work->y[i];
  }
}

void update_xz_tilde(OSQPWorkspace *work) {
  // Compute right-hand side
  compute_rhs(work);

  // Solve linear system
  work->linsys_solver->solve(work->linsys_solver, work->xz_tilde);
}

void update_x(OSQPWorkspace *work) {
  c_int i;

  // update x
  for (i = 0; i < work->data->n; i++) {
    work->x[i] = work->settings->alpha * work->xz_tilde[i] +
                 ((c_float)1.0 - work->settings->alpha) * work->x_prev[i];
  }

  // update delta_x
  for (i = 0; i < work->data->n; i++) {
    work->delta_x[i] = work->x[i] - work->x_prev[i];
  }
}

void update_z(OSQPWorkspace *work) {
  c_int i;

  // update z
  for (i = 0; i < work->data->m; i++) {
    work->z[i] = work->settings->alpha * work->xz_tilde[i + work->data->n] +
                 ((c_float)1.0 - work->settings->alpha) * work->z_prev[i] +
                 work->rho_inv_vec[i] * work->y[i];
  }

  // project z
  project(work, work->z);
}

void update_y(OSQPWorkspace *work) {
  c_int i; // Index

  for (i = 0; i < work->data->m; i++) {
    work->delta_y[i] = work->rho_vec[i] *
                       (work->settings->alpha *
                        work->xz_tilde[i + work->data->n] +
                        ((c_float)1.0 - work->settings->alpha) * work->z_prev[i] -
                        work->z[i]);
    work->y[i] += work->delta_y[i];
  }
}

c_float compute_obj_val(OSQPWorkspace *work, c_float *x) {
  c_float obj_val;

  obj_val = quad_form(work->data->P, x) +
            vec_prod(work->data->q, x, work->data->n);

  if (work->settings->scaling) {
    obj_val *= work->scaling->cinv;
  }

  return obj_val;
}

c_float compute_pri_res(OSQPWorkspace *work, c_float *x, c_float *z) {
  // NB: Use z_prev as working vector
  // pr = Ax - z

  mat_vec(work->data->A, x, work->Ax, 0); // Ax
  vec_add_scaled(work->z_prev, work->Ax, z, work->data->m, -1);

  // If scaling active -> rescale residual
  if (work->settings->scaling && !work->settings->scaled_termination) {
    return vec_scaled_norm_inf(work->scaling->Einv, work->z_prev, work->data->m);
  }

  // Return norm of the residual
  return vec_norm_inf(work->z_prev, work->data->m);
}

c_float compute_pri_tol(OSQPWorkspace *work, c_float eps_abs, c_float eps_rel) {
  c_float max_rel_eps, temp_rel_eps;

  // max_rel_eps = max(||z||, ||A x||)
  if (work->settings->scaling && !work->settings->scaled_termination) {
    // ||Einv * z||
    max_rel_eps =
      vec_scaled_norm_inf(work->scaling->Einv, work->z, work->data->m);

    // ||Einv * A * x||
    temp_rel_eps = vec_scaled_norm_inf(work->scaling->Einv,
                                       work->Ax,
                                       work->data->m);

    // Choose maximum
    max_rel_eps = c_max(max_rel_eps, temp_rel_eps);
  } else { // No unscaling required
    // ||z||
    max_rel_eps = vec_norm_inf(work->z, work->data->m);

    // ||A * x||
    temp_rel_eps = vec_norm_inf(work->Ax, work->data->m);

    // Choose maximum
    max_rel_eps = c_max(max_rel_eps, temp_rel_eps);
  }

  // eps_prim
  return eps_abs + eps_rel * max_rel_eps;
}

c_float compute_dua_res(OSQPWorkspace *work, c_float *x, c_float *y) {
  // NB: Use x_prev as temporary vector
  // NB: Only upper triangular part of P is stored.
  // dr = q + A'*y + P*x

  // dr = q
  prea_vec_copy(work->data->q, work->x_prev, work->data->n);

  // P * x (upper triangular part)
  mat_vec(work->data->P, x, work->Px, 0);

  // P' * x (lower triangular part with no diagonal)
  mat_tpose_vec(work->data->P, x, work->Px, 1, 1);

  // dr += P * x (full P matrix)
  vec_add_scaled(work->x_prev, work->x_prev, work->Px, work->data->n, 1);

  // dr += A' * y
  if (work->data->m > 0) {
    mat_tpose_vec(work->data->A, y, work->Aty, 0, 0);
    vec_add_scaled(work->x_prev, work->x_prev, work->Aty, work->data->n, 1);
  }

  // If scaling active -> rescale residual
  if (work->settings->scaling && !work->settings->scaled_termination) {
    return work->scaling->cinv * vec_scaled_norm_inf(work->scaling->Dinv,
                                                     work->x_prev,
                                                     work->data->n);
  }

  return vec_norm_inf(work->x_prev, work->data->n);
}

c_float compute_dua_tol(OSQPWorkspace *work, c_float eps_abs, c_float eps_rel) {
  c_float max_rel_eps, temp_rel_eps;

  // max_rel_eps = max(||q||, ||A' y|, ||P x||)
  if (work->settings->scaling && !work->settings->scaled_termination) {
    // || Dinv q||
    max_rel_eps = vec_scaled_norm_inf(work->scaling->Dinv,
                                      work->data->q,
                                      work->data->n);

    // || Dinv A' y ||
    temp_rel_eps = vec_scaled_norm_inf(work->scaling->Dinv,
                                       work->Aty,
                                       work->data->n);
    max_rel_eps = c_max(max_rel_eps, temp_rel_eps);

    // || Dinv P x||
    temp_rel_eps = vec_scaled_norm_inf(work->scaling->Dinv,
                                       work->Px,
                                       work->data->n);
    max_rel_eps = c_max(max_rel_eps, temp_rel_eps);

    // Multiply by cinv
    max_rel_eps *= work->scaling->cinv;
  } else { // No scaling required
    // ||q||
    max_rel_eps = vec_norm_inf(work->data->q, work->data->n);

    // ||A'*y||
    temp_rel_eps = vec_norm_inf(work->Aty, work->data->n);
    max_rel_eps  = c_max(max_rel_eps, temp_rel_eps);

    // ||P*x||
    temp_rel_eps = vec_norm_inf(work->Px, work->data->n);
    max_rel_eps  = c_max(max_rel_eps, temp_rel_eps);
  }

  // eps_dual
  return eps_abs + eps_rel * max_rel_eps;
}

c_int is_primal_infeasible(OSQPWorkspace *work, c_float eps_prim_inf) {
  // This function checks for the primal infeasibility termination criteria.
  //
  // 1) A' * delta_y < eps * ||delta_y||
  //
  // 2) u'*max(delta_y, 0) + l'*min(delta_y, 0) < eps * ||delta_y||
  //

  c_int i; // Index for loops
  c_float norm_delta_y;
  c_float ineq_lhs = 0.0;

  // Project delta_y onto the polar of the recession cone of [l,u]
  for (i = 0; i < work->data->m; i++) {
    if (work->data->u[i] > OSQP_INFTY * MIN_SCALING) {          // Infinite upper bound
      if (work->data->l[i] < -OSQP_INFTY * MIN_SCALING) {       // Infinite lower bound
        // Both bounds infinite
        work->delta_y[i] = 0.0;
      } else {
        // Only upper bound infinite
        work->delta_y[i] = c_min(work->delta_y[i], 0.0);
      }
    } else if (work->data->l[i] < -OSQP_INFTY * MIN_SCALING) {  // Infinite lower bound
      // Only lower bound infinite
      work->delta_y[i] = c_max(work->delta_y[i], 0.0);
    }
  }

  // Compute infinity norm of delta_y (unscale if necessary)
  if (work->settings->scaling && !work->settings->scaled_termination) {
    // Use work->Adelta_x as temporary vector
    vec_ew_prod(work->scaling->E, work->delta_y, work->Adelta_x, work->data->m);
    norm_delta_y = vec_norm_inf(work->Adelta_x, work->data->m);
  } else {
    norm_delta_y = vec_norm_inf(work->delta_y, work->data->m);
  }

  if (norm_delta_y > OSQP_DIVISION_TOL) {

    for (i = 0; i < work->data->m; i++) {
      ineq_lhs += work->data->u[i] * c_max(work->delta_y[i], 0) + \
                  work->data->l[i] * c_min(work->delta_y[i], 0);
    }

    // Check if the condition is satisfied: ineq_lhs < -eps
    if (ineq_lhs < eps_prim_inf * norm_delta_y) {
      // Compute and return ||A'delta_y|| < eps_prim_inf
      mat_tpose_vec(work->data->A, work->delta_y, work->Atdelta_y, 0, 0);

      // Unscale if necessary
      if (work->settings->scaling && !work->settings->scaled_termination) {
        vec_ew_prod(work->scaling->Dinv,
                    work->Atdelta_y,
                    work->Atdelta_y,
                    work->data->n);
      }

      return vec_norm_inf(work->Atdelta_y, work->data->n) < eps_prim_inf * norm_delta_y;
    }
  }

  // Conditions not satisfied -> not primal infeasible
  return 0;
}

c_int is_dual_infeasible(OSQPWorkspace *work, c_float eps_dual_inf) {
  // This function checks for the scaled dual infeasibility termination
  // criteria.
  //
  // 1) q * delta_x < eps * || delta_x ||
  //
  // 2) ||P * delta_x || < eps * || delta_x ||
  //
  // 3) -> (A * delta_x)_i > -eps * || delta_x ||,    l_i != -inf
  //    -> (A * delta_x)_i <  eps * || delta_x ||,    u_i != inf
  //


  c_int   i; // Index for loops
  c_float norm_delta_x;
  c_float cost_scaling;

  // Compute norm of delta_x
  if (work->settings->scaling && !work->settings->scaled_termination) { // Unscale
                                                                        // if
                                                                        // necessary
    norm_delta_x = vec_scaled_norm_inf(work->scaling->D,
                                       work->delta_x,
                                       work->data->n);
    cost_scaling = work->scaling->c;
  } else {
    norm_delta_x = vec_norm_inf(work->delta_x, work->data->n);
    cost_scaling = 1.0;
  }

  // Prevent 0 division || delta_x || > 0
  if (norm_delta_x > OSQP_DIVISION_TOL) {
    // Normalize delta_x by its norm

    /* vec_mult_scalar(work->delta_x, 1./norm_delta_x, work->data->n); */

    // Check first if q'*delta_x < 0
    if (vec_prod(work->data->q, work->delta_x, work->data->n) <
        cost_scaling * eps_dual_inf * norm_delta_x) {
      // Compute product P * delta_x (NB: P is store in upper triangular form)
      mat_vec(work->data->P, work->delta_x, work->Pdelta_x, 0);
      mat_tpose_vec(work->data->P, work->delta_x, work->Pdelta_x, 1, 1);

      // Scale if necessary
      if (work->settings->scaling && !work->settings->scaled_termination) {
        vec_ew_prod(work->scaling->Dinv,
                    work->Pdelta_x,
                    work->Pdelta_x,
                    work->data->n);
      }

      // Check if || P * delta_x || = 0
      if (vec_norm_inf(work->Pdelta_x, work->data->n) <
          cost_scaling * eps_dual_inf * norm_delta_x) {
        // Compute A * delta_x
        mat_vec(work->data->A, work->delta_x, work->Adelta_x, 0);

        // Scale if necessary
        if (work->settings->scaling && !work->settings->scaled_termination) {
          vec_ew_prod(work->scaling->Einv,
                      work->Adelta_x,
                      work->Adelta_x,
                      work->data->m);
        }

        // De Morgan Law Applied to dual infeasibility conditions for A * x
        // NB: Note that MIN_SCALING is used to adjust the infinity value
        //     in case the problem is scaled.
        for (i = 0; i < work->data->m; i++) {
          if (((work->data->u[i] < OSQP_INFTY * MIN_SCALING) &&
               (work->Adelta_x[i] >  eps_dual_inf * norm_delta_x)) ||
              ((work->data->l[i] > -OSQP_INFTY * MIN_SCALING) &&
               (work->Adelta_x[i] < -eps_dual_inf * norm_delta_x))) {
            // At least one condition not satisfied -> not dual infeasible
            return 0;
          }
        }

        // All conditions passed -> dual infeasible
        return 1;
      }
    }
  }

  // Conditions not satisfied -> not dual infeasible
  return 0;
}

c_int has_solution(OSQPInfo * info){

  return ((info->status_val != OSQP_PRIMAL_INFEASIBLE) &&
      (info->status_val != OSQP_PRIMAL_INFEASIBLE_INACCURATE) &&
      (info->status_val != OSQP_DUAL_INFEASIBLE) &&
      (info->status_val != OSQP_DUAL_INFEASIBLE_INACCURATE) &&
      (info->status_val != OSQP_NON_CVX));

}

void store_solution(OSQPWorkspace *work) {

  if (has_solution(work->info)) {
    prea_vec_copy(work->x, work->solution->x, work->data->n); // primal
    prea_vec_copy(work->y, work->solution->y, work->data->m); // dual

    // Unscale solution if scaling has been performed
    if (work->settings->scaling)
      unscale_solution(work);
  } else {
    // No solution present. Solution is NaN
    vec_set_scalar(work->solution->x, OSQP_NAN, work->data->n);
    vec_set_scalar(work->solution->y, OSQP_NAN, work->data->m);


    // Cold start iterates to 0 for next runs (they cannot start from NaN)
    cold_start(work);
  }
}

void update_info(OSQPWorkspace *work,
                 c_int          iter,
                 c_int          compute_objective,
                 c_int          polish) {
  c_float *x, *z, *y;                   // Allocate pointers to variables
  c_float *obj_val, *pri_res, *dua_res; // objective value, residuals


  x                = work->x;
  y                = work->y;
  z                = work->z;
  obj_val          = &work->info->obj_val;
  pri_res          = &work->info->pri_res;
  dua_res          = &work->info->dua_res;
  work->info->iter = iter; // Update iteration number


  // Compute the objective if needed
  if (compute_objective) {
    *obj_val = compute_obj_val(work, x);
  }

  // Compute primal residual
  if (work->data->m == 0) {
    // No constraints -> Always primal feasible
    *pri_res = 0.;
  } else {
    *pri_res = compute_pri_res(work, x, z);
  }

  // Compute dual residual
  *dua_res = compute_dua_res(work, x, y);

  // Update timing

}


void reset_info(OSQPInfo *info) {

  update_status(info, OSQP_UNSOLVED); // Problem is unsolved

}

void update_status(OSQPInfo *info, c_int status_val) {
  // Update status value
  info->status_val = status_val;

  // Update status string depending on status val
  if (status_val == OSQP_SOLVED) c_strcpy(info->status, "solved");

  if (status_val == OSQP_SOLVED_INACCURATE) c_strcpy(info->status,
                                                     "solved inaccurate");
  else if (status_val == OSQP_PRIMAL_INFEASIBLE) c_strcpy(info->status,
                                                          "primal infeasible");
  else if (status_val == OSQP_PRIMAL_INFEASIBLE_INACCURATE) c_strcpy(info->status,
                                                                     "primal infeasible inaccurate");
  else if (status_val == OSQP_UNSOLVED) c_strcpy(info->status, "unsolved");
  else if (status_val == OSQP_DUAL_INFEASIBLE) c_strcpy(info->status,
                                                        "dual infeasible");
  else if (status_val == OSQP_DUAL_INFEASIBLE_INACCURATE) c_strcpy(info->status,
                                                                   "dual infeasible inaccurate");
  else if (status_val == OSQP_MAX_ITER_REACHED) c_strcpy(info->status,
                                                         "maximum iterations reached");
  else if (status_val == OSQP_SIGINT) c_strcpy(info->status, "interrupted");

  else if (status_val == OSQP_NON_CVX) c_strcpy(info->status, "problem non convex");

}

c_int check_termination(OSQPWorkspace *work, c_int approximate) {
  c_float eps_prim, eps_dual, eps_prim_inf, eps_dual_inf;
  c_int   exitflag;
  c_int   prim_res_check, dual_res_check, prim_inf_check, dual_inf_check;
  c_float eps_abs, eps_rel;

  // Initialize variables to 0
  exitflag       = 0;
  prim_res_check = 0; dual_res_check = 0;
  prim_inf_check = 0; dual_inf_check = 0;

  // Initialize tolerances
  eps_abs      = work->settings->eps_abs;
  eps_rel      = work->settings->eps_rel;
  eps_prim_inf = work->settings->eps_prim_inf;
  eps_dual_inf = work->settings->eps_dual_inf;

  // If residuals are too large, the problem is probably non convex
  if ((work->info->pri_res > OSQP_INFTY) ||
      (work->info->dua_res > OSQP_INFTY)){
    // Looks like residuals are diverging. Probably the problem is non convex!
    // Terminate and report it
    update_status(work->info, OSQP_NON_CVX);
    work->info->obj_val = OSQP_NAN;
    return 1;
  }

  // If approximate solution required, increase tolerances by 10
  if (approximate) {
    eps_abs      *= 10;
    eps_rel      *= 10;
    eps_prim_inf *= 10;
    eps_dual_inf *= 10;
  }

  // Check residuals
  if (work->data->m == 0) {
    prim_res_check = 1; // No constraints -> Primal feasibility always satisfied
  }
  else {
    // Compute primal tolerance
    eps_prim = compute_pri_tol(work, eps_abs, eps_rel);

    // Primal feasibility check
    if (work->info->pri_res < eps_prim) {
      prim_res_check = 1;
    } else {
      // Primal infeasibility check
      prim_inf_check = is_primal_infeasible(work, eps_prim_inf);
    }
  } // End check if m == 0

  // Compute dual tolerance
  eps_dual = compute_dua_tol(work, eps_abs, eps_rel);

  // Dual feasibility check
  if (work->info->dua_res < eps_dual) {
    dual_res_check = 1;
  } else {
    // Check dual infeasibility
    dual_inf_check = is_dual_infeasible(work, eps_dual_inf);
  }

  // Compare checks to determine solver status
  if (prim_res_check && dual_res_check) {
    // Update final information
    if (approximate) {
      update_status(work->info, OSQP_SOLVED_INACCURATE);
    } else {
      update_status(work->info, OSQP_SOLVED);
    }
    exitflag = 1;
  }
  else if (prim_inf_check) {
    // Update final information
    if (approximate) {
      update_status(work->info, OSQP_PRIMAL_INFEASIBLE_INACCURATE);
    } else {
      update_status(work->info, OSQP_PRIMAL_INFEASIBLE);
    }

    if (work->settings->scaling && !work->settings->scaled_termination) {
      // Update infeasibility certificate
      vec_ew_prod(work->scaling->E, work->delta_y, work->delta_y, work->data->m);
    }
    work->info->obj_val = OSQP_INFTY;
    exitflag            = 1;
  }
  else if (dual_inf_check) {
    // Update final information
    if (approximate) {
      update_status(work->info, OSQP_DUAL_INFEASIBLE_INACCURATE);
    } else {
      update_status(work->info, OSQP_DUAL_INFEASIBLE);
    }

    if (work->settings->scaling && !work->settings->scaled_termination) {
      // Update infeasibility certificate
      vec_ew_prod(work->scaling->D, work->delta_x, work->delta_x, work->data->n);
    }
    work->info->obj_val = -OSQP_INFTY;
    exitflag            = 1;
  }

  return exitflag;
}


