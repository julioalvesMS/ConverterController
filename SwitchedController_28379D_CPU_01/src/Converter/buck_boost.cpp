#include <src/Converter/buck_boost.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuckBoost
{
    static System system;
    static System discreteSystem;


    System* BuckBoost::GetSys()
    {
        DefineSystem();

        system.N = 2;

        return &(system);
    }


    System* BuckBoost::GetDiscreteSys()
    {
        DefineDiscreteSystem();

        discreteSystem.N = 2;

        return &(discreteSystem);
    }


    void BuckBoost::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // BuckBoost Converter - Rule 1
            //
            P[0][0] = 4.48917e-05;
            P[0][1] = 2.12726e-05;
            P[1][0] = 2.12726e-05;
            P[1][1] = 7.42982e-05;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Rule 2
            //
            P[0][0] = 0.000972027;
            P[0][1] = 5.07287e-07;
            P[1][0] = 5.07287e-07;
            P[1][1] = 0.001125;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            P[0][0] = 3.04753e-06;
            P[0][1] = 1.59905e-06;
            P[1][0] = 1.59905e-06;
            P[1][1] = 8.8219e-06;
            break;

        default:
            break;
        }
    }


    void BuckBoost::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            h[0] = 5.93261e-06;
            h[1] = 9.22972e-06;
            break;

        default:
            break;
        }
    }


    double BuckBoost::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            d = 1.6236e-05;
            break;

        default:
            break;
        }

        return d;
    }


    void BuckBoost::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.002;
        num[1] = -0.00199;

        den[0] = 1;
        den[1] = -1;
    }


    void BuckBoost::GetClassicVoltageCurrnetController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
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


    void BuckBoost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 3;
        num[1] = -2.9847;

        den[0] = 1;
        den[1] = -1;
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
        subSys->A[0][0] = 0.989817;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.99977;
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->L[0] = 0.0250089;
        subSys->L[1] = -0.000279266;
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
        subSys->L[0] = -0.0314279;
        subSys->L[1] = 0.000350945;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }
}
