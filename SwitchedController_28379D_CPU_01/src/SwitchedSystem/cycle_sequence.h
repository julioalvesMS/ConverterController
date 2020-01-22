#ifndef SRC_CYCLE_SEQUENCE_H_
#define SRC_CYCLE_SEQUENCE_H_

#include <src/Common/constants.h>

namespace CycleSequence
{

    class CycleStep
    {
    public:
        double P[SYSTEM_ORDER][SYSTEM_ORDER];
        double Xe[SYSTEM_ORDER];
        double ell[SUBSYSTEMS_COUNT][SYSTEM_ORDER];
    };

    class Cycle
    {
    public:
        int kappa;
        double rho;
        CycleStep cycleSteps[LIMIT_CYCLE_STEPS];
    };

}

#endif /* SRC_CYCLE_SEQUENCE_H_ */
