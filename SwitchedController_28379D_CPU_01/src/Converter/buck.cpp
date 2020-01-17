#include <src/Converter/buck.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuck
{
    static System system;
    static System discreteSystem;


    System* Buck::GetSys()
    {
        DefineSystem();

        system.N = 2;

        return &(system);
    }


    System* Buck::GetDiscreteSys()
    {
        DefineDiscreteSystem();

        discreteSystem.N = 2;

        return &(discreteSystem);
    }


    void Buck::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // Buck Converter - Rule 1
            //
            P[0][0] = 4.52359e-05;
            P[0][1] = 9.58526e-06;
            P[1][0] = 9.58526e-06;
            P[1][1] = 5.66023e-05;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Buck Converter - Rule 2
            //
            P[0][0] = 4.52355e-05;
            P[0][1] = 9.58518e-06;
            P[1][0] = 9.58518e-06;
            P[1][1] = 5.66019e-05;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            P[0][0] = 1.54003e-05;
            P[0][1] = 3.3615e-06;
            P[1][0] = 3.3615e-06;
            P[1][1] = 2.71727e-05;
            break;

        default:
            break;
        }
    }


    void Buck::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            h[0] = -9.77065e-21;
            h[1] = 7.20138e-20;
            break;

        default:
            break;
        }
    }


    double Buck::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            d = 2.14138e-34;
            break;

        default:
            break;
        }

        return d;
    }


    void Buck::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.01;
        num[1] = -0.00995;

        den[0] = 1;
        den[1] = -1;
    }


    void Buck::GetClassicVoltageCurrnetController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
    {
        vNum[0] = 0.3161;
        vNum[1] = -0.3159;

        vDen[0] = 1;
        vDen[1] = -1;

        iNum[0] = 0.02042;
        iNum[1] = -0.02018;

        iDen[0] = 1;
        iDen[1] = -1;
    }


    void Buck::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9;

        den[0] = 1;
        den[1] = -1;
    }


    int Buck::SubSystem2SwitchState(int SubSystem)
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
        subSys->L[0] = 0.00972589;
        subSys->L[1] = 0.000108247;
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
        subSys->L[0] = -0.0157297;
        subSys->L[1] = -0.000175068;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }
}
