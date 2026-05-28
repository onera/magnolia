#include "scaling.h"


c_int unscale_data(OSQPWorkspace *work) {
  // Unscale cost
  mat_mult_scalar(work->data->P, work->scaling->cinv);
  mat_premult_diag(work->data->P, work->scaling->Dinv);
  mat_postmult_diag(work->data->P, work->scaling->Dinv);
  vec_mult_scalar(work->data->q, work->scaling->cinv, work->data->n);
  vec_ew_prod(work->scaling->Dinv, work->data->q, work->data->q, work->data->n);

  // Unscale constraints
  mat_premult_diag(work->data->A, work->scaling->Einv);
  mat_postmult_diag(work->data->A, work->scaling->Dinv);
  vec_ew_prod(work->scaling->Einv, work->data->l, work->data->l, work->data->m);
  vec_ew_prod(work->scaling->Einv, work->data->u, work->data->u, work->data->m);

  return 0;
}

c_int unscale_solution(OSQPWorkspace *work) {
  // primal
  vec_ew_prod(work->scaling->D,
              work->solution->x,
              work->solution->x,
              work->data->n);

  // dual
  vec_ew_prod(work->scaling->E,
              work->solution->y,
              work->solution->y,
              work->data->m);
  vec_mult_scalar(work->solution->y, work->scaling->cinv, work->data->m);

  return 0;
}
