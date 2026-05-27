#ifndef GLOB_OPTS_H
#define GLOB_OPTS_H


/*
   Define OSQP compiler flags
 */

// cmake generated compiler flags
#include "osqp_configure.h"

/* DATA CUSTOMIZATIONS (depending on memory manager)-----------------------   */

// We do not need memory allocation functions if EMBEDDED is enabled


/* Use customized number representation -----------------------------------   */
typedef long long c_int; /* for indices */


#ifndef DFLOAT         // Doubles
typedef double c_float; /* for numerical values  */
#else                  // Floats
typedef float c_float;  /* for numerical values  */
#endif /* ifndef DFLOAT */


/* Use customized operations */

#ifndef c_absval
#define c_absval(x) (((x) < 0) ? -(x) : (x))
#endif /* ifndef c_absval */

#ifndef c_max
#define c_max(a, b) (((a) > (b)) ? (a) : (b))
#endif /* ifndef c_max */

#ifndef c_min
#define c_min(a, b) (((a) < (b)) ? (a) : (b))
#endif /* ifndef c_min */

// Round x to the nearest multiple of N
#ifndef c_roundmultiple
#define c_roundmultiple(x, N) ((x) + .5 * (N)-c_fmod((x) + .5 * (N), (N)))
#endif /* ifndef c_roundmultiple */


/* Use customized functions -----------------------------------------------   */


#endif /* ifndef GLOB_OPTS_H */
