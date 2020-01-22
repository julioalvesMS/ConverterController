#include <src/Converter/buck_boost_3.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuckBoost3
{
    static System system;
    static System discreteSystem;
    static Cycle limitCycle;


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


    Cycle* BuckBoost3::GetLimitCycle()
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


    void BuckBoost3::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // BuckBoost3 Converter - Rule 1
            //
            P[0][0] = 0.00241074;
            P[0][1] = -2.43921e-05;
            P[1][0] = -2.43921e-05;
            P[1][1] = 0.00275677;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.00244308;
            P[0][1] = -8.78619e-06;
            P[1][0] = -8.78619e-06;
            P[1][1] = 0.00279305;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 1.19242e-06;
            P[0][1] = 4.59165e-07;
            P[1][0] = 4.59165e-07;
            P[1][1] = 2.76253e-06;
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
            h[0] = 6.35795e-07;
            h[1] = 7.21143e-07;
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
            d = 4.26748e-07;
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
        subSys->A[0][0] = 0.994825;
        subSys->A[0][1] = -0.0127605;
        subSys->A[1][0] = 0.0110818;
        subSys->A[1][1] = 0.999814;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = -0.00416256;
        subSys->L[1] = 0.000124301;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->E[0][0] = 1;
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
        subSys->Q[0][0] = 1;
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
        subSys->L[0] = -0.0169238;
        subSys->L[1] = 5.33469e-05;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->E[0][0] = 1;
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
        subSys->Q[0][0] = 1;
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
        subSys->A[0][0] = 0.994895;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 3 -- Matrix L
        //
        subSys->L[0] = 0.0126245;
        subSys->L[1] = -0.000150975;
        //
        // Subsystem 3 -- Matrix Q
        //
        subSys->E[0][0] = 1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.0103306;
        //
        // Subsystem 3 -- Matrix H
        //
        subSys->H[0] = 0;
        subSys->H[1] = 0;
        //
        // Subsystem 3 -- Matrix G
        //
        subSys->G[0] = 0;
        subSys->G[1] = 0;
        //
        // Subsystem 3 -- Matrix Q
        //
        subSys->Q[0][0] = 1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }



    void DefineLimitCycleCost()
    {
        CycleStep* step;

        limitCycle.kappa = 5;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 96.9313;
        step->P[0][1] = -0.732131;
        step->P[1][0] = -0.732131;
        step->P[1][1] = 110.595;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.630687;
        step->Xe[1] = 80.7302;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.03023;
        step->ell[0][1] = 0.00587325;
        step->ell[1][0] = -1.85971;
        step->ell[1][1] = 0.0012612;
        step->ell[2][0] = 0;
        step->ell[2][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 96.9182;
        step->P[0][1] = -0.735976;
        step->P[1][0] = -0.735976;
        step->P[1][1] = 110.61;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.45697;
        step->Xe[1] = 80.7209;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03017;
        step->ell[2][1] = -0.0150306;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 96.9215;
        step->P[0][1] = -0.735148;
        step->P[1][0] = -0.735148;
        step->P[1][1] = 110.606;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.24887;
        step->Xe[1] = 80.7267;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03022;
        step->ell[2][1] = -0.0127241;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 96.9248;
        step->P[0][1] = -0.734231;
        step->P[1][0] = -0.734231;
        step->P[1][1] = 110.603;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.04177;
        step->Xe[1] = 80.7301;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.0104288;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 96.928;
        step->P[0][1] = -0.733226;
        step->P[1][0] = -0.733226;
        step->P[1][1] = 110.599;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.835704;
        step->Xe[1] = 80.7313;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.00814514;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 5;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.50503e+26;
        step->P[0][1] = 3.92772e+25;
        step->P[1][0] = 3.92772e+25;
        step->P[1][1] = 1.82814e+26;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.45697;
        step->Xe[1] = 80.7209;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03017;
        step->ell[2][1] = -0.0150306;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.50341e+26;
        step->P[0][1] = 3.92638e+25;
        step->P[1][0] = 3.92638e+25;
        step->P[1][1] = 1.8302e+26;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.24887;
        step->Xe[1] = 80.7267;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03022;
        step->ell[2][1] = -0.0127241;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.50173e+26;
        step->P[0][1] = 3.92446e+25;
        step->P[1][0] = 3.92446e+25;
        step->P[1][1] = 1.83223e+26;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.04177;
        step->Xe[1] = 80.7301;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.0104288;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.49998e+26;
        step->P[0][1] = 3.92193e+25;
        step->P[1][0] = 3.92193e+25;
        step->P[1][1] = 1.83424e+26;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.835704;
        step->Xe[1] = 80.7313;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 7.61891e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.00814514;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.49817e+26;
        step->P[0][1] = 3.9188e+25;
        step->P[1][0] = 3.9188e+25;
        step->P[1][1] = 1.83621e+26;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.630687;
        step->Xe[1] = 80.7302;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.03023;
        step->ell[0][1] = 0.00587325;
        step->ell[1][0] = -1.85971;
        step->ell[1][1] = 0.0012612;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 5;
        limitCycle.rho = 2.02444e-22;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.73831e+14;
        step->P[0][1] = 4.20697e+13;
        step->P[1][0] = 4.20697e+13;
        step->P[1][1] = 1.95685e+14;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.45697;
        step->Xe[1] = 80.7209;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03017;
        step->ell[2][1] = -0.0150306;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.73646e+14;
        step->P[0][1] = 4.20857e+13;
        step->P[1][0] = 4.20857e+13;
        step->P[1][1] = 1.95902e+14;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.24887;
        step->Xe[1] = 80.7267;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03022;
        step->ell[2][1] = -0.0127241;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.73459e+14;
        step->P[0][1] = 4.20965e+13;
        step->P[1][0] = 4.20965e+13;
        step->P[1][1] = 1.96118e+14;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.04177;
        step->Xe[1] = 80.7301;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.59195e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.0104288;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.73272e+14;
        step->P[0][1] = 4.21018e+13;
        step->P[1][0] = 4.21018e+13;
        step->P[1][1] = 1.96333e+14;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.835704;
        step->Xe[1] = 80.7313;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 7.61891e-15;
        step->ell[1][0] = -0.829483;
        step->ell[1][1] = -0.00461205;
        step->ell[2][0] = 1.03025;
        step->ell[2][1] = -0.00814514;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.73083e+14;
        step->P[0][1] = 4.21018e+13;
        step->P[1][0] = 4.21018e+13;
        step->P[1][1] = 1.96546e+14;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.630687;
        step->Xe[1] = 80.7302;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.03023;
        step->ell[0][1] = 0.00587325;
        step->ell[1][0] = -1.85971;
        step->ell[1][1] = 0.0012612;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

    }

}
