#include "util.h"

/***************
* Versioning  *
***************/
const char* osqp_version(void) {
  return OSQP_VERSION;
}

/************************************
* Printing Constants to set Layout *
************************************/

/**********************
* Utility Functions  *
**********************/
void c_strcpy(char dest[], const char source[]) {
  int i = 0;

  while (1) {
    dest[i] = source[i];

    if (dest[i] == '\0') break;
    i++;
  }
}


/*******************
* Timer Functions *
*******************/


/* ==================== DEBUG FUNCTIONS ======================= */


// If debug mode enabled
#ifdef DDEBUG


#endif // DEBUG MODE
