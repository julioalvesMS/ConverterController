#include <src/Converter/boost.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBoost
{
    static System system;
    static System discreteSystem;


    System* Boost::GetSys()
    {
        DefineSystem();

        system.N = 2;

        return &(system);
    }


    System* Boost::GetDiscreteSys()
    {
        DefineDiscreteSystem();

        discreteSystem.N = 2;

        return &(discreteSystem);
    }


    void Boost::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // Boost Converter - Rule 1
            //
            P[0][0] = 4.48515e-05;
            P[0][1] = 2.22111e-05;
            P[1][0] = 2.22111e-05;
            P[1][1] = 7.63392e-05;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.000971581;
            P[0][1] = 4.96527e-07;
            P[1][0] = 4.96527e-07;
            P[1][1] = 0.001125;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 2.91485e-06;
            P[0][1] = 1.6548e-06;
            P[1][0] = 1.6548e-06;
            P[1][1] = 8.81002e-06;
            break;

        default:
            break;
        }
    }


    void Boost::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            h[0] = 6.0156e-06;
            h[1] = 9.74712e-06;
            break;

        default:
            break;
        }
    }


    double Boost::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            d = 1.7509e-05;
            break;

        default:
            break;
        }

        return d;
    }


    void Boost::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.0203;
        num[1] = -0.0200615;

        den[0] = 1;
        den[1] = -1;
    }


    void Boost::GetClassicVoltageCurrnetController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
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


    void Boost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 3;
        num[1] = -2.9847;

        den[0] = 1;
        den[1] = -1;
    }


    int Boost::SubSystem2SwitchState(int SubSystem)
    {
        int switchState;

        switch(SubSystem)
        {
        case 0:
            switchState = 2;
            break;
        case 1:
            switchState = 0;
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
        subSys->B[0] = 1/L;
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
        subSys->L[0] = 0.0247173;
        subSys->L[1] = -0.000526048;
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
        subSys->L[0] = -0.0336363;
        subSys->L[1] = 0.000715866;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }
}
