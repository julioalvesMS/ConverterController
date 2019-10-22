#include <src/Converter/buck_boost.h>

using namespace SwitchedSystem;
using namespace BaseConverter;

namespace ConverterBuckBoost
{
    static System system;
    static System discreteSystem;


    System* BuckBoost::GetSys()
    {
        DefineSystem();

        return &(system);
    }


    System* BuckBoost::GetDiscreteSys()
    {
        DefineDiscreteSystem();

        return &(discreteSystem);
    }


    void BuckBoost::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
#if SWITCHING_RULE == CONTINUOUS_RULE_1
        //
        // Buck Converter - Rule 1
        //
        P[0][0] = 4.4977e-05;
        P[0][1] = 1.9091e-05;
        P[1][0] = 1.9091e-05;
        P[1][1] = 6.9922e-05;
#elif SWITCHING_RULE == CONTINUOUS_RULE_2
        //
        // Buck Converter - Rule 2
        //
        P[0][0] = 9.7304e-04;
        P[0][1] = 5.3437e-07;
        P[1][0] = 5.3437e-07;
        P[1][1] = 0.0011;
#elif SWITCHING_RULE == DISCRETE_RULE_1
        //
        // Buck Converter - Rule 2
        //
        P[0][0] = 3.7319e-06;
        P[0][1] = 2.4064e-06;
        P[1][0] = 2.4064e-06;
        P[1][1] = 1.0160e-05;
#endif
    }

    void BuckBoost::GetH(double h[SYSTEM_ORDER])
    {
#if SWITCHING_RULE == DISCRETE_RULE_1
        //
        // Buck Converter - Rule 2
        //
        h[0] = 5.9279e-06;
        h[1] = 9.3472e-06;
#endif
    }


    double BuckBoost::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

#if SWITCHING_RULE == DISCRETE_RULE_1
        d = 1.2962e-05;
#endif

        return d;
    }


    int BuckBoost::SubSystem2SwitchState(int SubSystem)
    {
        int switchState;

        switch(SubSystem)
        {
        case 0:
            switchState = 2;
            break;
        case 1:
            switchState = 1;
            break;
        default:
            switchState = -1;
            break;
        }

        return switchState;
    }


    void DefineSystem()
    {
        SubSystem* subSys;

        //
        // =============== Subsystem 1 ===============
        //
        subSys = &(system.subSystems[0]);

        //
        // Subsystem 1 -- Matrix A
        //
        subSys->A[0][0] = -R/L;
        subSys->A[0][1] =  0;
        subSys->A[1][0] =  0;
        subSys->A[1][1] = -1/(Ro*Co);
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->B[0] = 1/L;
        subSys->B[1] = 0;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->Q[0][0] = 0;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 1/Ro;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(system.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A[0][0] = -R/L;
        subSys->A[0][1] = -1/L;
        subSys->A[1][0] =  1/Co;
        subSys->A[1][1] = -1/(Ro*Co);
        //
        // Subsystem 2 -- Matrix B
        //
        subSys->B[0] = 0;
        subSys->B[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 1/Ro;
    }

    void DefineDiscreteSystem()
    {
        SubSystem* subSys;

        //
        // =============== Subsystem 1 ===============
        //
        subSys = &(discreteSystem.subSystems[0]);

        //
        // Subsystem 1 -- Matrix A
        //
        subSys->A[0][0] =  0.9786;
        subSys->A[0][1] = -0.0506;
        subSys->A[1][0] =  0.0440;
        subSys->A[1][1] =  0.9984;
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->L[0] = 0.0193;
        subSys->L[1] = 4.3142e-04;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->Q[0][0] = 0;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 1/Ro;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(discreteSystem.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A[0][0] =  0.9786;
        subSys->A[0][1] = -0.0506;
        subSys->A[1][0] =  0.0440;
        subSys->A[1][1] =  0.9984;
        //
        // Subsystem 2 -- Matrix B
        //
        subSys->L[0] = -0.0313;
        subSys->L[1] = -6.9774e-04;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 1/Ro;
    }
}
