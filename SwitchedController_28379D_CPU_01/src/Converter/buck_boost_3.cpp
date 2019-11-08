#include <src/Converter/buck_boost_3.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuckBoost3
{
    static System system;
    static System discreteSystem;


    System* BuckBoost3::GetSys()
    {
        DefineSystem();

        system.N = 3;

        return &(system);
    }


    System* BuckBoost3::GetDiscreteSys()
    {
        DefineDiscreteSystem();

        discreteSystem.N = 3;

        return &(discreteSystem);
    }


    void BuckBoost3::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // BuckBoost3 Converter - Rule 1
            //
            P[0][0] = 4.50848e-05;
            P[0][1] = 1.58594e-05;
            P[1][0] = 1.58594e-05;
            P[1][1] = 6.43618e-05;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.000974906;
            P[0][1] = 7.86545e-07;
            P[1][0] = 7.86545e-07;
            P[1][1] = 0.00112501;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 4.68983e-06;
            P[0][1] = 1.79798e-06;
            P[1][0] = 1.79798e-06;
            P[1][1] = 1.08704e-05;
            break;

        default:
            break;
        }
    }


    void BuckBoost3::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            h[0] = 4.95923e-06;
            h[1] = 5.62314e-06;
            break;

        default:
            break;
        }
    }


    double BuckBoost3::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            d = 6.60471e-06;
            break;

        default:
            break;
        }

        return d;
    }


    void BuckBoost3::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 0.5;
        num[1] = -0.49;

        den[0] = 1;
        den[1] = -1;
    }


    int BuckBoost3::SubSystem2SwitchState(int SubSystem)
    {
        return SubSystem;
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
        subSys->A[0][1] = -1/L;
        subSys->A[1][0] =  1/Co;
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


        //
        // =============== Subsystem 3 ===============
        //
        subSys = &(system.subSystems[2]);

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
        subSys->A[0][0] = 0.989534;
        subSys->A[0][1] = -0.0254527;
        subSys->A[1][0] = 0.0221042;
        subSys->A[1][1] = 0.999487;
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->L[0] = -0.0082764;
        subSys->L[1] = 0.000295293;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(discreteSystem.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A[0][0] = 0.989534;
        subSys->A[0][1] = -0.0254527;
        subSys->A[1][0] = 0.0221042;
        subSys->A[1][1] = 0.999487;
        //
        // Subsystem 2 -- Matrix B
        //
        subSys->L[0] = -0.033732;
        subSys->L[1] = 1.1979e-05;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;


        //
        // =============== Subsystem 3 ===============
        //
        subSys = &(discreteSystem.subSystems[2]);

        //
        // Subsystem 3 -- Matrix A
        //
        subSys->A[0][0] = 0.989817;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.99977;
        //
        // Subsystem 3 -- Matrix B
        //
        subSys->L[0] = 0.0251424;
        subSys->L[1] = -0.000301283;
        //
        // Subsystem 3 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }
}
