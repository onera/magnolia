#include "../../include/glob_opts.h"

#include "../../include/qdldl.h"
#include "../../include/qdldl_interface.h"


// Permute x = P*b using P
void permute_x(c_int n, c_float * x, const c_float * b, const c_int * P) {
    c_int j;
    for (j = 0 ; j < n ; j++) x[j] = b[P[j]];
}

// Permute x = P'*b using P
void permutet_x(c_int n, c_float * x, const c_float * b, const c_int * P) {
    c_int j;
    for (j = 0 ; j < n ; j++) x[P[j]] = b[j];
}


static void LDLSolve(c_float *x, c_float *b, const csc *L, const c_float *Dinv, const c_int *P, c_float *bp) {
    /* solves P'LDL'P x = b for x */
    permute_x(L->n, bp, b, P);
    QDLDL_solve(L->n, L->p, L->i, L->x, Dinv, bp);
    permutet_x(L->n, x, bp, P);

}


c_int solve_linsys_qdldl(qdldl_solver * s, c_float * b) {
    c_int j;

        /* stores solution to the KKT system in s->sol */
        LDLSolve(s->sol, b, s->L, s->Dinv, s->P, s->bp);

        /* copy x_tilde from s->sol */
        for (j = 0 ; j < s->n ; j++) {
            b[j] = s->sol[j];
        }

        /* compute z_tilde from b and s->sol */
        for (j = 0 ; j < s->m ; j++) {
            b[j + s->n] += s->rho_inv_vec[j] * s->sol[j + s->n];
        }

    return 0;
}


