#ifndef SRC_CORE_SWITCHED_SYSTEM_H_
#define SRC_CORE_SWITCHED_SYSTEM_H_

#include <src/Common/constants.h>

namespace SwitchedSystem
{

    class SubSystem
    {
    public:
        double A[SYSTEM_ORDER][SYSTEM_ORDER];
        double Ar[SYSTEM_ORDER][SYSTEM_ORDER];
        double Ar0[SYSTEM_ORDER][SYSTEM_ORDER];
        double Ard[SYSTEM_ORDER][SYSTEM_ORDER];
        double B[SYSTEM_ORDER];
        double L[SYSTEM_ORDER][SYSTEM_ORDER+1];
        double l[SYSTEM_ORDER];

        double E[SYSTEM_ORDER][SYSTEM_ORDER];
        double H[SYSTEM_ORDER];
        double G[SYSTEM_ORDER];

        double Q[SYSTEM_ORDER][SYSTEM_ORDER];

    };

    class System
    {
    public:
        int N;
        SubSystem subSystems[SUBSYSTEMS_COUNT];
    };

}

#endif /* SRC_CORE_SWITCHED_SYSTEM_H_ */
