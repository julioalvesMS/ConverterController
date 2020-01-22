#include <src/Converter/boost.h>

using namespace SwitchedSystem;
using namespace BaseConverter;
using namespace Controller;

extern ControlStrategy controlStrategy;

namespace ConverterBoost
{
    static System system;
    static System discreteSystem;
    static Cycle limitCycle;


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


    Cycle* Boost::GetLimitCycle()
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


    void Boost::GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
            //
            // Boost Converter - Rule 1
            //
            P[0][0] = 0.00241068;
            P[0][1] = -2.27767e-05;
            P[1][0] = -2.27767e-05;
            P[1][1] = 0.00275919;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.00244307;
            P[0][1] = -8.78488e-06;
            P[1][0] = -8.78488e-06;
            P[1][1] = 0.00279299;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 3.14522e-07;
            P[0][1] = 1.14556e-07;
            P[1][0] = 1.14556e-07;
            P[1][1] = 6.98378e-07;
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
            h[0] = 1.39517e-07;
            h[1] = 1.4744e-07;
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
            d = 7.61054e-08;
            break;

        default:
            break;
        }

        return d;
    }



    void Boost::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.002;
        num[1] = -0.00199;

        den[0] = 1;
        den[1] = -1;
    }



    void Boost::GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
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



    void Boost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9847;

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
        subSys->A[0][0] = 0.994895;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0126179;
        subSys->L[1] = -0.000176328;
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
        subSys->L[0] = -0.00698789;
        subSys->L[1] = 9.76518e-05;
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

    }



    void DefineLimitCycleCost()
    {
        CycleStep* step;

        limitCycle.kappa = 11;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 96.9294;
        step->P[0][1] = -0.919039;
        step->P[1][0] = -0.919039;
        step->P[1][1] = 110.359;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.65914;
        step->Xe[1] = 101.086;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29006;
        step->ell[1][1] = 0.0158261;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 96.9164;
        step->P[0][1] = -0.923864;
        step->P[1][0] = -0.923864;
        step->P[1][1] = 110.374;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.48018;
        step->Xe[1] = 101.075;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.28997;
        step->ell[1][1] = 0.0249255;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 96.9031;
        step->P[0][1] = -0.928714;
        step->P[1][0] = -0.928714;
        step->P[1][1] = 110.389;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 3.29702;
        step->Xe[1] = 101.063;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.28988;
        step->ell[0][1] = -0.0339784;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 96.9105;
        step->P[0][1] = -0.926577;
        step->P[1][0] = -0.926577;
        step->P[1][1] = 110.381;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.81982;
        step->Xe[1] = 101.086;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.29013;
        step->ell[0][1] = -0.0286886;
        step->ell[1][0] = -4.44089e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 96.9179;
        step->P[0][1] = -0.924237;
        step->P[1][0] = -0.924237;
        step->P[1][1] = 110.372;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.3448;
        step->Xe[1] = 101.103;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.29031;
        step->ell[0][1] = -0.0234233;
        step->ell[1][0] = -6.66134e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 96.9254;
        step->P[0][1] = -0.921692;
        step->P[1][0] = -0.921692;
        step->P[1][1] = 110.363;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.87202;
        step->Xe[1] = 101.115;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.29043;
        step->ell[0][1] = -0.0181832;
        step->ell[1][0] = -3.33067e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 96.9328;
        step->P[0][1] = -0.918943;
        step->P[1][0] = -0.918943;
        step->P[1][1] = 110.355;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.40154;
        step->Xe[1] = 101.121;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.29048;
        step->ell[0][1] = -0.0129689;
        step->ell[1][0] = 1.11022e-15;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 96.9403;
        step->P[0][1] = -0.915989;
        step->P[1][0] = -0.915989;
        step->P[1][1] = 110.346;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.933409;
        step->Xe[1] = 101.122;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.29046;
        step->ell[0][1] = -0.0077811;
        step->ell[1][0] = -4.44089e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 96.9477;
        step->P[0][1] = -0.912829;
        step->P[1][0] = -0.912829;
        step->P[1][1] = 110.338;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.467684;
        step->Xe[1] = 101.119;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.29038;
        step->ell[0][1] = -0.00262028;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 96.9552;
        step->P[0][1] = -0.909463;
        step->P[1][0] = -0.909463;
        step->P[1][1] = 110.329;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.00441752;
        step->Xe[1] = 101.11;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29023;
        step->ell[1][1] = -0.00251292;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 96.9424;
        step->P[0][1] = -0.914238;
        step->P[1][0] = -0.914238;
        step->P[1][1] = 110.344;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.833898;
        step->Xe[1] = 101.098;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29015;
        step->ell[1][1] = 0.00668007;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 11;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.06193e+10;
        step->P[0][1] = 3.39978e+09;
        step->P[1][0] = 3.39978e+09;
        step->P[1][1] = 1.30487e+10;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.816492;
        step->Xe[1] = 101.122;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.29045;
        step->ell[0][1] = -0.00648547;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = -2.08028e-14;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.05935e+10;
        step->P[0][1] = 3.39585e+09;
        step->P[1][0] = 3.39585e+09;
        step->P[1][1] = 1.3081e+10;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.351376;
        step->Xe[1] = 101.117;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.29035;
        step->ell[1][1] = 0.0013315;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.0643e+10;
        step->P[0][1] = 3.40128e+09;
        step->P[1][0] = 3.40128e+09;
        step->P[1][1] = 1.30261e+10;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.17909;
        step->Xe[1] = 101.105;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = -1.42109e-14;
        step->ell[1][0] = -1.29026;
        step->ell[1][1] = 0.0105049;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.06924e+10;
        step->P[0][1] = 3.40669e+09;
        step->P[1][0] = 3.40669e+09;
        step->P[1][1] = 1.29713e+10;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.00257;
        step->Xe[1] = 101.094;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.29017;
        step->ell[0][1] = -0.0196314;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.06657e+10;
        step->P[0][1] = 3.40432e+09;
        step->P[1][0] = 3.40432e+09;
        step->P[1][1] = 1.30034e+10;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.53168;
        step->Xe[1] = 101.102;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29024;
        step->ell[1][1] = 0.0144125;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.07145e+10;
        step->P[0][1] = 3.40951e+09;
        step->P[1][0] = 3.40951e+09;
        step->P[1][1] = 1.29482e+10;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.35336;
        step->Xe[1] = 101.09;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.29015;
        step->ell[0][1] = -0.0235191;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.06871e+10;
        step->P[0][1] = 3.40745e+09;
        step->P[1][0] = 3.40745e+09;
        step->P[1][1] = 1.29801e+10;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.8807;
        step->Xe[1] = 101.102;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 2.84217e-14;
        step->ell[1][0] = -1.29027;
        step->ell[1][1] = 0.0182802;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.07351e+10;
        step->P[0][1] = 3.41241e+09;
        step->P[1][0] = 3.41241e+09;
        step->P[1][1] = 1.29246e+10;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 2.7006;
        step->Xe[1] = 101.091;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.29018;
        step->ell[0][1] = -0.0273671;
        step->ell[1][0] = -6.66134e-16;
        step->ell[1][1] = -2.08028e-14;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.0707e+10;
        step->P[0][1] = 3.41065e+09;
        step->P[1][0] = 3.41065e+09;
        step->P[1][1] = 1.29562e+10;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 2.22614;
        step->Xe[1] = 101.106;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.29035;
        step->ell[0][1] = -0.022108;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.06783e+10;
        step->P[0][1] = 3.40796e+09;
        step->P[1][0] = 3.40796e+09;
        step->P[1][1] = 1.29875e+10;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 1.75393;
        step->Xe[1] = 101.117;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 1.29045;
        step->ell[0][1] = -0.0168743;
        step->ell[1][0] = -8.88178e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.0649e+10;
        step->P[0][1] = 3.40434e+09;
        step->P[1][0] = 3.40434e+09;
        step->P[1][1] = 1.30183e+10;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 1.28403;
        step->Xe[1] = 101.122;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 1.29048;
        step->ell[0][1] = -0.0116666;
        step->ell[1][0] = 0;
        step->ell[1][1] = 7.61891e-15;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 11;
        limitCycle.rho = 1.03803e-28;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 7.32602e+15;
        step->P[0][1] = 2.07112e+15;
        step->P[1][0] = 2.07112e+15;
        step->P[1][1] = 8.20433e+15;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.76753;
        step->Xe[1] = 101.097;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.2902;
        step->ell[1][1] = 0.0170265;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 7.35541e+15;
        step->P[0][1] = 2.06962e+15;
        step->P[1][0] = 2.06962e+15;
        step->P[1][1] = 8.17071e+15;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.58801;
        step->Xe[1] = 101.086;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.29011;
        step->ell[0][1] = -0.0261197;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 7.33881e+15;
        step->P[0][1] = 2.07126e+15;
        step->P[1][0] = 2.07126e+15;
        step->P[1][1] = 8.19011e+15;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.11419;
        step->Xe[1] = 101.1;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.29027;
        step->ell[0][1] = -0.0208679;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 7.32212e+15;
        step->P[0][1] = 2.0724e+15;
        step->P[1][0] = 2.0724e+15;
        step->P[1][1] = 8.20944e+15;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.64264;
        step->Xe[1] = 101.11;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29035;
        step->ell[1][1] = 0.0156415;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 7.35149e+15;
        step->P[0][1] = 2.07089e+15;
        step->P[1][0] = 2.07089e+15;
        step->P[1][1] = 8.17579e+15;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.46375;
        step->Xe[1] = 101.098;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.29026;
        step->ell[0][1] = -0.0247419;
        step->ell[1][0] = -4.44089e-16;
        step->ell[1][1] = 2.18298e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 7.33485e+15;
        step->P[0][1] = 2.0724e+15;
        step->P[1][0] = 2.0724e+15;
        step->P[1][1] = 8.19519e+15;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.99042;
        step->Xe[1] = 101.111;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.29039;
        step->ell[0][1] = -0.0194955;
        step->ell[1][0] = -6.66134e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 7.31811e+15;
        step->P[0][1] = 2.07342e+15;
        step->P[1][0] = 2.07342e+15;
        step->P[1][1] = 8.21451e+15;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.51937;
        step->Xe[1] = 101.119;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.29046;
        step->ell[0][1] = -0.0142748;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = -6.59195e-15;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 7.30128e+15;
        step->P[0][1] = 2.07394e+15;
        step->P[1][0] = 2.07394e+15;
        step->P[1][1] = 8.23374e+15;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.05065;
        step->Xe[1] = 101.122;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.29046;
        step->ell[0][1] = -0.00908042;
        step->ell[1][0] = -8.88178e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 7.28437e+15;
        step->P[0][1] = 2.07398e+15;
        step->P[1][0] = 2.07398e+15;
        step->P[1][1] = 8.25287e+15;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.584329;
        step->Xe[1] = 101.119;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.2904;
        step->ell[1][1] = 0.00391289;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 7.31358e+15;
        step->P[0][1] = 2.07232e+15;
        step->P[1][0] = 2.07232e+15;
        step->P[1][1] = 8.21897e+15;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 1.41085;
        step->Xe[1] = 101.108;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 1.29031;
        step->ell[0][1] = -0.0130731;
        step->ell[1][0] = -9.99201e-16;
        step->ell[1][1] = 7.61891e-15;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 7.29676e+15;
        step->P[0][1] = 2.07273e+15;
        step->P[1][0] = 2.07273e+15;
        step->P[1][1] = 8.23815e+15;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.942842;
        step->Xe[1] = 101.109;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.29029;
        step->ell[1][1] = 0.00788659;

    }

}
