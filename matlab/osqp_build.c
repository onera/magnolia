#define S_FUNCTION_NAME  osqp_build
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "osqp.h"
#include "workspace.h"
#include "config_mpc.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    if (!ssSetNumInputPorts(S, 3)) return;
    
    ssSetInputPortWidth(S, 0, workspace.data->n);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    ssSetInputPortWidth(S, 1, workspace.data->m);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 1, 1);

    ssSetInputPortWidth(S, 2, workspace.data->m);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortRequiredContiguous(S, 2, 1);

    if (!ssSetNumOutputPorts(S, 2)) return;
    
    ssSetOutputPortWidth(S, 0, 3);
    
    ssSetOutputPortWidth(S, 1, 1);
    
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    const real_T *q = (const real_T*) ssGetInputPortSignal(S,0);
    const real_T *l = (const real_T*) ssGetInputPortSignal(S,1);
    const real_T *u = (const real_T*) ssGetInputPortSignal(S,2);
    
    real_T *mv     = ssGetOutputPortSignal(S,0);
    real_T *status = ssGetOutputPortSignal(S,1);

    osqp_update_lin_cost(&workspace, (c_float*)q);
    osqp_update_bounds(&workspace, (c_float*)l, (c_float*)u);

    osqp_solve(&workspace);

    status[0] = (real_T)workspace.info->status_val;

    if (workspace.info->status_val == 1 || workspace.info->status_val == 2) {
        mv[0] = (real_T)workspace.solution->x[N_STATES*(NP+1)];
        mv[1] = (real_T)workspace.solution->x[N_STATES*(NP+1)+1];
        mv[2] = (real_T)workspace.solution->x[N_STATES*(NP+1)+2];
    } else {
        mv[0] = 0.0; 
        mv[1] = 0.0; 
        mv[2] = 0.0;
    }
}

static void mdlTerminate(SimStruct *S) {}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif