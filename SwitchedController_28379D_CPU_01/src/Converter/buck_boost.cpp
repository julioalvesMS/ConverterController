#include <src/Converter/buck_boost.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBuckBoost
{
    static System system;
    static System discreteSystem;
    static Cycle limitCycle;


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


    Cycle* BuckBoost::GetLimitCycle()
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


    void BuckBoost::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // BuckBoost Converter - Rule 1
            //
            P[0][0] = 0.000259993;
            P[0][1] = 1.63634e-05;
            P[1][0] = 1.63634e-05;
            P[1][1] = 0.000316768;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Rule 2
            //
            P[0][0] = 0.000972799;
            P[0][1] = 3.01178e-07;
            P[1][0] = 3.01178e-07;
            P[1][1] = 0.001125;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            P[0][0] = 2.0018e-07;
            P[0][1] = 1.05306e-07;
            P[1][0] = 1.05306e-07;
            P[1][1] = 5.78926e-07;
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
            h[0] = 1.97064e-07;
            h[1] = 3.0633e-07;
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
            d = 2.72449e-07;
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



    void BuckBoost::GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
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



    void BuckBoost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9847;

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
        subSys->A[0][0] = 0.994895;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0125756;
        subSys->L[1] = -0.000140451;
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
        subSys->Q[0][0] = 0.1;
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
        subSys->L[0] = -0.0158033;
        subSys->L[1] = 0.0001765;
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
        subSys->Q[0][0] = 0.1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.0103306;

    }



    void DefineLimitCycleCost()
    {
        CycleStep* step;

        limitCycle.kappa = 9;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 10.434;
        step->P[0][1] = 0.643996;
        step->P[1][0] = 0.643996;
        step->P[1][1] = 12.6904;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.198898;
        step->Xe[1] = 79.5853;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84504;
        step->ell[1][1] = -0.00785088;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 10.4403;
        step->P[0][1] = 0.647374;
        step->P[1][0] = 0.647374;
        step->P[1][1] = 12.683;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.631621;
        step->Xe[1] = 79.5761;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84498;
        step->ell[1][1] = 0.00135343;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 10.4467;
        step->P[0][1] = 0.650769;
        step->P[1][0] = 0.650769;
        step->P[1][1] = 12.6756;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.4579;
        step->Xe[1] = 79.567;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84493;
        step->ell[1][1] = 0.0105108;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 10.4532;
        step->P[0][1] = 0.654183;
        step->P[1][0] = 0.654183;
        step->P[1][1] = 12.6681;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.27996;
        step->Xe[1] = 79.5579;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84487;
        step->ell[1][1] = 0.0196213;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 10.4597;
        step->P[0][1] = 0.657614;
        step->P[1][0] = 0.657614;
        step->P[1][1] = 12.6607;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 3.09783;
        step->Xe[1] = 79.5487;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -5.55112e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84481;
        step->ell[1][1] = 0.0286854;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 10.4662;
        step->P[0][1] = 0.661064;
        step->P[1][0] = 0.661064;
        step->P[1][1] = 12.6533;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 3.91151;
        step->Xe[1] = 79.5396;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.84475;
        step->ell[0][1] = -0.0377033;
        step->ell[1][0] = 8.88178e-16;
        step->ell[1][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 10.4582;
        step->P[0][1] = 0.65714;
        step->P[1][0] = 0.65714;
        step->P[1][1] = 12.6627;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.8763;
        step->Xe[1] = 79.5682;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.84504;
        step->ell[0][1] = -0.0262292;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 10.4501;
        step->P[0][1] = 0.652987;
        step->P[1][0] = 0.652987;
        step->P[1][1] = 12.6721;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.84608;
        step->Xe[1] = 79.5853;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.84519;
        step->ell[0][1] = -0.0148112;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 10.4421;
        step->P[0][1] = 0.648606;
        step->P[1][0] = 0.648606;
        step->P[1][1] = 12.6813;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.820976;
        step->Xe[1] = 79.591;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.84519;
        step->ell[0][1] = -0.00345078;
        step->ell[1][0] = 8.32667e-17;
        step->ell[1][1] = 0;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 9;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 6.32789e+19;
        step->P[0][1] = 3.2005e+19;
        step->P[1][0] = 3.2005e+19;
        step->P[1][1] = 1.13824e+20;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 2.06378;
        step->Xe[1] = 79.5735;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84505;
        step->ell[1][1] = 0.0172245;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 6.36119e+19;
        step->P[0][1] = 3.22077e+19;
        step->P[1][0] = 3.22077e+19;
        step->P[1][1] = 1.13459e+20;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.88274;
        step->Xe[1] = 79.5644;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.84499;
        step->ell[0][1] = -0.0263008;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 6.32279e+19;
        step->P[0][1] = 3.19588e+19;
        step->P[1][0] = 3.19588e+19;
        step->P[1][1] = 1.13915e+20;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.85254;
        step->Xe[1] = 79.5815;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84514;
        step->ell[1][1] = 0.0148831;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 6.3553e+19;
        step->P[0][1] = 3.21619e+19;
        step->P[1][0] = 3.21619e+19;
        step->P[1][1] = 1.13551e+20;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.67259;
        step->Xe[1] = 79.5724;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.84508;
        step->ell[0][1] = -0.0239713;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 6.3162e+19;
        step->P[0][1] = 3.19115e+19;
        step->P[1][0] = 3.19115e+19;
        step->P[1][1] = 1.14007e+20;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.64337;
        step->Xe[1] = 79.5872;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.8452;
        step->ell[1][1] = 0.0125646;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 6.34789e+19;
        step->P[0][1] = 3.21149e+19;
        step->P[1][0] = 3.21149e+19;
        step->P[1][1] = 1.13643e+20;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.46448;
        step->Xe[1] = 79.5781;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.84514;
        step->ell[0][1] = -0.0216647;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 6.30806e+19;
        step->P[0][1] = 3.18627e+19;
        step->P[1][0] = 3.18627e+19;
        step->P[1][1] = 1.14098e+20;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.43627;
        step->Xe[1] = 79.5906;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.84523;
        step->ell[0][1] = -0.0102694;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 6.26797e+19;
        step->P[0][1] = 3.15992e+19;
        step->P[1][0] = 3.15992e+19;
        step->P[1][1] = 1.14548e+20;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.413213;
        step->Xe[1] = 79.5918;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84517;
        step->ell[1][1] = -0.00106803;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 6.29796e+19;
        step->P[0][1] = 3.18015e+19;
        step->P[1][0] = 3.18015e+19;
        step->P[1][1] = 1.14186e+20;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.24061;
        step->Xe[1] = 79.5826;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84511;
        step->ell[1][1] = 0.00810165;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 9;
        limitCycle.rho = 1.32674e-22;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 7.69991e+22;
        step->P[0][1] = 2.60338e+22;
        step->P[1][0] = 2.60338e+22;
        step->P[1][1] = 8.39107e+22;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 3.08568;
        step->Xe[1] = 79.5632;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.84499;
        step->ell[0][1] = -0.0285498;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 7.66785e+22;
        step->P[0][1] = 2.6065e+22;
        step->P[1][0] = 2.6065e+22;
        step->P[1][1] = 8.42831e+22;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.05444;
        step->Xe[1] = 79.5827;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.84517;
        step->ell[0][1] = -0.0171204;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = -1.42109e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 7.63562e+22;
        step->P[0][1] = 2.60862e+22;
        step->P[1][0] = 2.60862e+22;
        step->P[1][1] = 8.46541e+22;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.02829;
        step->Xe[1] = 79.5906;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.8452;
        step->ell[1][1] = 0.00574821;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 7.66135e+22;
        step->P[0][1] = 2.60668e+22;
        step->P[1][0] = 2.60668e+22;
        step->P[1][1] = 8.43574e+22;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.85254;
        step->Xe[1] = 79.5815;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84514;
        step->ell[1][1] = 0.0148831;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 7.68723e+22;
        step->P[0][1] = 2.60486e+22;
        step->P[1][0] = 2.60486e+22;
        step->P[1][1] = 8.40621e+22;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.67259;
        step->Xe[1] = 79.5724;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.84508;
        step->ell[0][1] = -0.0239714;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 7.6549e+22;
        step->P[0][1] = 2.60728e+22;
        step->P[1][0] = 2.60728e+22;
        step->P[1][1] = 8.44283e+22;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.64337;
        step->Xe[1] = 79.5872;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.8452;
        step->ell[0][1] = -0.0125647;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 7.62257e+22;
        step->P[0][1] = 2.60903e+22;
        step->P[1][0] = 2.60903e+22;
        step->P[1][1] = 8.47987e+22;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.619288;
        step->Xe[1] = 79.5906;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84517;
        step->ell[1][1] = 0.00121574;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 7.64826e+22;
        step->P[0][1] = 2.60703e+22;
        step->P[1][0] = 2.60703e+22;
        step->P[1][1] = 8.45013e+22;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.44563;
        step->Xe[1] = 79.5815;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84511;
        step->ell[1][1] = 0.0103738;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 7.67398e+22;
        step->P[0][1] = 2.60512e+22;
        step->P[1][0] = 2.60512e+22;
        step->P[1][1] = 8.42052e+22;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 2.26775;
        step->Xe[1] = 79.5724;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84505;
        step->ell[1][1] = 0.019485;

    }

}
