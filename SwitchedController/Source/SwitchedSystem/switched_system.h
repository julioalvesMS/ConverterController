#ifndef _switched_system_h_
#define _switched_system_h_

#include "Source/Math/matrix.h"

typedef struct subsystem {

    Matrix *A;
    Matrix *B;

} SubSystem;

typedef struct switched_system {
    int N;  // Number of SubSystems

    SubSystem *subSystems;

} SwitchedSystem;

#endif
