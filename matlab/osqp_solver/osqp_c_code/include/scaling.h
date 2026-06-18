#ifndef SCALING_H
#define SCALING_H


// Functions to scale problem data
#include "types.h"
#include "lin_alg.h"
#include "constants.h"

// Enable data scaling if EMBEDDED is disabled or if EMBEDDED == 2


/**
 * Unscale problem matrices
 * @param  work Workspace
 * @return      exitflag
 */
c_int unscale_data(OSQPWorkspace *work);


/**
 * Unscale solution
 * @param  work Workspace
 * @return      exitflag
 */
c_int unscale_solution(OSQPWorkspace *work);


#endif // ifndef SCALING_H
