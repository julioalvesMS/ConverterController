#include <src/Converter/buck.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuck
{
    static System system;
    static System discreteSystem;
    static Cycle limitCycle;


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


    Cycle* Buck::GetLimitCycle()
    {
        switch(controlStrategy)
        {
        case CS_LIMIT_CYCLE_COST:
            DefineLimitCycleCost();
            break;
        case CS_LIMIT_CYCLE_H2:
            DefineLimitCycleH2();
            break;
        case CS_LIMIT_CYCLE_Hinf:
            DefineLimitCycleHinf();
        default:
            break;
        }

        return &(limitCycle);
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
            P[0][0] = 8.41785e-07;
            P[0][1] = 1.86619e-07;
            P[1][0] = 1.86619e-07;
            P[1][1] = 1.48723e-06;
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
            h[0] = 1.64885e-21;
            h[1] = -6.39568e-21;
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
            d = 3.4847e-35;
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



    void Buck::GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
    {
        vNum[0] = 0.316;
        vNum[1] = -0.315839;

        vDen[0] = 1;
        vDen[1] = -1;

        iNum[0] = 0.0203;
        iNum[1] = -0.0200615;

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
        subSys->A[0][0] = 0.994825;
        subSys->A[0][1] = -0.0127605;
        subSys->A[1][0] = 0.0110818;
        subSys->A[1][1] = 0.999814;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.00487573;
        subSys->L[1] = 2.71098e-05;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->E[0][0] = 0.1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.0103306;
        //
        // Subsystem 1 -- Matrix H
        //
        subSys->H[0] = 0;
        subSys->H[1] = 0;
        //
        // Subsystem 1 -- Matrix G
        //
        subSys->G[0] = 0;
        subSys->G[1] = 0;
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
        subSys->A[0][0] = 0.994825;
        subSys->A[0][1] = -0.0127605;
        subSys->A[1][0] = 0.0110818;
        subSys->A[1][1] = 0.999814;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0] = -0.00788555;
        subSys->L[1] = -4.38448e-05;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->E[0][0] = 0.1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.0103306;
        //
        // Subsystem 2 -- Matrix H
        //
        subSys->H[0] = 0;
        subSys->H[1] = 0;
        //
        // Subsystem 2 -- Matrix G
        //
        subSys->G[0] = 0;
        subSys->G[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.01;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }



    void DefineLimitCycleCost()
    {
        CycleStep* step;

        limitCycle.kappa = 13;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.871677;
        step->Xe[1] = 39.8423;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 7.61891e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = -0.546091;
        step->Xe[1] = 39.8298;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = -0.222032;
        step->Xe[1] = 39.821;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.100463;
        step->Xe[1] = 39.8157;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.421356;
        step->Xe[1] = 39.8141;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 5.55112e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.74061;
        step->Xe[1] = 39.8159;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.05819;
        step->Xe[1] = 39.8214;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -7.77156e-16;
        step->ell[0][1] = 7.61891e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.37405;
        step->Xe[1] = 39.8303;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.68817;
        step->Xe[1] = 39.8428;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = -8.88178e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 1.17101;
        step->Xe[1] = 39.8541;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = -7.10543e-15;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.656393;
        step->Xe[1] = 39.8596;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 1.94289e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = 0.144365;
        step->Xe[1] = 39.8595;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 1.16573e-15;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 1.81444;
        step->P[0][1] = 0.383412;
        step->P[1][0] = 0.383412;
        step->P[1][1] = 2.26926;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = -0.365011;
        step->Xe[1] = 39.8537;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 0;
        step->ell[1][1] = -7.10543e-15;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 13;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.15674e+16;
        step->P[0][1] = 2.47067e+15;
        step->P[1][0] = 2.47067e+15;
        step->P[1][1] = 1.34457e+16;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.10783;
        step->Xe[1] = 39.8523;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 7.10543e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.15696e+16;
        step->P[0][1] = 2.47135e+15;
        step->P[1][0] = 2.47135e+15;
        step->P[1][1] = 1.34473e+16;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.593563;
        step->Xe[1] = 39.8571;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 8.32667e-17;
        step->ell[1][1] = -7.10543e-15;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.15713e+16;
        step->P[0][1] = 2.47193e+15;
        step->P[1][0] = 2.47193e+15;
        step->P[1][1] = 1.34487e+16;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.0818923;
        step->Xe[1] = 39.8563;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = -1.66533e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.15727e+16;
        step->P[0][1] = 2.47241e+15;
        step->P[1][0] = 2.47241e+15;
        step->P[1][1] = 1.34499e+16;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = -0.42712;
        step->Xe[1] = 39.8498;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -5.55112e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.15738e+16;
        step->P[0][1] = 2.47278e+15;
        step->P[1][0] = 2.47278e+15;
        step->P[1][1] = 1.34507e+16;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = -0.103932;
        step->Xe[1] = 39.8423;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.15745e+16;
        step->P[0][1] = 2.47304e+15;
        step->P[1][0] = 2.47304e+15;
        step->P[1][1] = 1.34513e+16;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = -0.611804;
        step->Xe[1] = 39.8337;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.15749e+16;
        step->P[0][1] = 2.47318e+15;
        step->P[1][0] = 2.47318e+15;
        step->P[1][1] = 1.34517e+16;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = -0.287454;
        step->Xe[1] = 39.8242;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 7.61891e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.15749e+16;
        step->P[0][1] = 2.47321e+15;
        step->P[1][0] = 2.47321e+15;
        step->P[1][1] = 1.34517e+16;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.0353383;
        step->Xe[1] = 39.8182;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.15746e+16;
        step->P[0][1] = 2.47311e+15;
        step->P[1][0] = 2.47311e+15;
        step->P[1][1] = 1.34515e+16;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.356537;
        step->Xe[1] = 39.8158;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.15739e+16;
        step->P[0][1] = 2.47289e+15;
        step->P[1][0] = 2.47289e+15;
        step->P[1][1] = 1.34509e+16;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.676103;
        step->Xe[1] = 39.817;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-15;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.15728e+16;
        step->P[0][1] = 2.47254e+15;
        step->P[1][0] = 2.47254e+15;
        step->P[1][1] = 1.34501e+16;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.994001;
        step->Xe[1] = 39.8217;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 1.15714e+16;
        step->P[0][1] = 2.47205e+15;
        step->P[1][0] = 2.47205e+15;
        step->P[1][1] = 1.34489e+16;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = 1.31019;
        step->Xe[1] = 39.8299;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 9.99201e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 1.15696e+16;
        step->P[0][1] = 2.47143e+15;
        step->P[1][0] = 2.47143e+15;
        step->P[1][1] = 1.34475e+16;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = 1.62464;
        step->Xe[1] = 39.8417;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 13;
        limitCycle.rho = 1.45557e-21;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.68817;
        step->Xe[1] = 39.8428;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.17101;
        step->Xe[1] = 39.8541;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.656393;
        step->Xe[1] = 39.8596;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = -1.94289e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.144365;
        step->Xe[1] = 39.8595;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = -2.77556e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = -0.365011;
        step->Xe[1] = 39.8537;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.829483;
        step->ell[0][1] = 0.00461205;
        step->ell[1][0] = 3.33067e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = -0.871677;
        step->Xe[1] = 39.8423;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -3.33067e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = -0.546091;
        step->Xe[1] = 39.8298;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -4.44089e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = -0.222032;
        step->Xe[1] = 39.821;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.100463;
        step->Xe[1] = 39.8157;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.421356;
        step->Xe[1] = 39.8141;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = -4.44089e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.74061;
        step->Xe[1] = 39.8159;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 5.55112e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = 1.05819;
        step->Xe[1] = 39.8214;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 5.13478e-16;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 1.34514e+10;
        step->P[0][1] = 2.7464e+09;
        step->P[1][0] = 2.7464e+09;
        step->P[1][1] = 1.52993e+10;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = 1.37405;
        step->Xe[1] = 39.8303;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;

    }

}
