#ifndef UTIL_H
#define UTIL_H


#include "types.h"
#include "constants.h"

/******************
* Versioning     *
******************/

/**
 * Return OSQP version
 * @return  OSQP version
 */
const char* osqp_version(void);


/**********************
* Utility Functions  *
**********************/


/**
 * Custom string copy to avoid string.h library
 * @param dest   destination string
 * @param source source string
 */
void c_strcpy(char       dest[],
              const char source[]);


/*********************************
* Timer Structs and Functions * *
*********************************/

/*! \cond PRIVATE */


/* ================================= DEBUG FUNCTIONS ======================= */

/*! \cond PRIVATE */


/*! \endcond */


#endif // ifndef UTIL_H
