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
            P[0][0] = 0.00200287;
            P[0][1] = -1.60846e-05;
            P[1][0] = -1.60846e-05;
            P[1][1] = 0.00226064;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.00202192;
            P[0][1] = -6.34993e-06;
            P[1][0] = -6.34993e-06;
            P[1][1] = 0.00228168;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 4.6766e-07;
            P[0][1] = 2.07556e-07;
            P[1][0] = 2.07556e-07;
            P[1][1] = 1.16448e-06;
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
            h[0] = 2.04164e-07;
            h[1] = 2.62795e-07;
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
            d = 1.16777e-07;
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



    void Boost::GetStateFeedbackH2Controller(double K[2], double C[2], double* M)
    {
        K[0] = 1.21908;
        K[1] = 0.661907;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.677367;
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
        subSys->A[0][0] = 0.993835;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0124073;
        subSys->L[1] = -0.000176275;
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
        subSys->A[0][0] = 0.993766;
        subSys->A[0][1] = -0.0125799;
        subSys->A[1][0] = 0.0110759;
        subSys->A[1][1] = 0.999815;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0] = -0.00691519;
        subSys->L[1] = 9.82467e-05;
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
        step->P[0][0] = 80.6162;
        step->P[0][1] = -0.647411;
        step->P[1][0] = -0.647411;
        step->P[1][1] = 90.4214;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.65886;
        step->Xe[1] = 100.856;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = -1.42109e-14;
        step->ell[1][0] = -1.2689;
        step->ell[1][1] = 0.0158651;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 80.607;
        step->P[0][1] = -0.651502;
        step->P[1][0] = -0.651502;
        step->P[1][1] = 90.4318;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.46639;
        step->Xe[1] = 100.844;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26881;
        step->ell[1][1] = 0.0248101;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 80.5976;
        step->P[0][1] = -0.655618;
        step->P[1][0] = -0.655618;
        step->P[1][1] = 90.4423;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 3.26895;
        step->Xe[1] = 100.833;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.26872;
        step->ell[0][1] = -0.0337;
        step->ell[1][0] = 3.33067e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 80.6028;
        step->P[0][1] = -0.653728;
        step->P[1][0] = -0.653728;
        step->P[1][1] = 90.4361;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.79784;
        step->Xe[1] = 100.855;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.26896;
        step->ell[0][1] = -0.0284805;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 80.608;
        step->P[0][1] = -0.651692;
        step->P[1][0] = -0.651692;
        step->P[1][1] = 90.4301;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.32939;
        step->Xe[1] = 100.872;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.26914;
        step->ell[0][1] = -0.0232908;
        step->ell[1][0] = -7.77156e-16;
        step->ell[1][1] = 1.77514e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 80.6133;
        step->P[0][1] = -0.64951;
        step->P[1][0] = -0.64951;
        step->P[1][1] = 90.424;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.86365;
        step->Xe[1] = 100.884;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.26926;
        step->ell[0][1] = -0.0181315;
        step->ell[1][0] = 4.44089e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 80.6185;
        step->P[0][1] = -0.64718;
        step->P[1][0] = -0.64718;
        step->P[1][1] = 90.4181;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.40067;
        step->Xe[1] = 100.89;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.26931;
        step->ell[0][1] = -0.013003;
        step->ell[1][0] = -1.22125e-15;
        step->ell[1][1] = 1.77514e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 80.6238;
        step->P[0][1] = -0.644703;
        step->P[1][0] = -0.644703;
        step->P[1][1] = 90.4122;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.940488;
        step->Xe[1] = 100.892;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.26929;
        step->ell[0][1] = -0.00790599;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 80.6291;
        step->P[0][1] = -0.642079;
        step->P[1][0] = -0.642079;
        step->P[1][1] = 90.4063;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.483159;
        step->Xe[1] = 100.888;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.26922;
        step->ell[0][1] = -0.0028409;
        step->ell[1][0] = -3.33067e-16;
        step->ell[1][1] = 1.77514e-14;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 80.6343;
        step->P[0][1] = -0.639306;
        step->P[1][0] = -0.639306;
        step->P[1][1] = 90.4006;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.0287272;
        step->Xe[1] = 100.879;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-15;
        step->ell[0][1] = -1.42109e-14;
        step->ell[1][0] = -1.26907;
        step->ell[1][1] = -0.00219175;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 80.6253;
        step->P[0][1] = -0.643346;
        step->P[1][0] = -0.643346;
        step->P[1][1] = 90.411;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.846312;
        step->Xe[1] = 100.868;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26898;
        step->ell[1][1] = 0.00686458;

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
        step->P[0][0] = 1.86496e+12;
        step->P[0][1] = 5.07643e+11;
        step->P[1][0] = 5.07643e+11;
        step->P[1][1] = 1.20915e+12;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 2.22157;
        step->Xe[1] = 100.865;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.26905;
        step->ell[0][1] = -0.022097;
        step->ell[1][0] = 5.55112e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.86119e+12;
        step->P[0][1] = 5.11992e+11;
        step->P[1][0] = 5.11992e+11;
        step->P[1][1] = 1.21459e+12;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.75658;
        step->Xe[1] = 100.876;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -4.44089e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26915;
        step->ell[1][1] = 0.0169461;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.86867e+12;
        step->P[0][1] = 5.06176e+11;
        step->P[1][0] = 5.06176e+11;
        step->P[1][1] = 1.20705e+12;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.56351;
        step->Xe[1] = 100.864;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.26906;
        step->ell[0][1] = -0.0258844;
        step->ell[1][0] = 5.55112e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.86479e+12;
        step->P[0][1] = 5.10362e+11;
        step->P[1][0] = 5.10362e+11;
        step->P[1][1] = 1.21222e+12;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.09641;
        step->Xe[1] = 100.879;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.26921;
        step->ell[0][1] = -0.0207098;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.86078e+12;
        step->P[0][1] = 5.14299e+11;
        step->P[1][0] = 5.14299e+11;
        step->P[1][1] = 1.21726e+12;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.63203;
        step->Xe[1] = 100.888;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.26929;
        step->ell[0][1] = -0.0155658;
        step->ell[1][0] = -5.55112e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.85664e+12;
        step->P[0][1] = 5.17988e+11;
        step->P[1][0] = 5.17988e+11;
        step->P[1][1] = 1.22216e+12;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.17044;
        step->Xe[1] = 100.892;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.26931;
        step->ell[0][1] = -0.0104529;
        step->ell[1][0] = -3.33067e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.85239e+12;
        step->P[0][1] = 5.21429e+11;
        step->P[1][0] = 5.21429e+11;
        step->P[1][1] = 1.22691e+12;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.711674;
        step->Xe[1] = 100.891;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.26927;
        step->ell[0][1] = -0.00537171;
        step->ell[1][0] = -3.33067e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.84804e+12;
        step->P[0][1] = 5.24624e+11;
        step->P[1][0] = 5.24624e+11;
        step->P[1][1] = 1.23149e+12;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.255782;
        step->Xe[1] = 100.884;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-15;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26916;
        step->ell[1][1] = 0.000322724;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.85524e+12;
        step->P[0][1] = 5.18005e+11;
        step->P[1][0] = 5.18005e+11;
        step->P[1][1] = 1.22275e+12;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.07197;
        step->Xe[1] = 100.873;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.26907;
        step->ell[1][1] = 0.00936354;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.86233e+12;
        step->P[0][1] = 5.11281e+11;
        step->P[1][0] = 5.11281e+11;
        step->P[1][1] = 1.214e+12;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 1.88312;
        step->Xe[1] = 100.861;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = -6.66134e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26898;
        step->ell[1][1] = 0.0183486;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.86931e+12;
        step->P[0][1] = 5.04451e+11;
        step->P[1][0] = 5.04451e+11;
        step->P[1][1] = 1.20525e+12;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 2.68927;
        step->Xe[1] = 100.85;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 1.26889;
        step->ell[0][1] = -0.0272783;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 11;
        limitCycle.rho = 3.73486e-29;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 4.1578e+16;
        step->P[0][1] = 1.31397e+16;
        step->P[1][0] = 1.31397e+16;
        step->P[1][1] = 4.57921e+16;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.5206;
        step->Xe[1] = 100.885;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.26925;
        step->ell[0][1] = -0.0143317;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 4.14713e+16;
        step->P[0][1] = 1.31433e+16;
        step->P[1][0] = 1.31433e+16;
        step->P[1][1] = 4.59122e+16;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.05974;
        step->Xe[1] = 100.888;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 2.84217e-14;
        step->ell[1][0] = -1.26925;
        step->ell[1][1] = 0.00922704;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 4.16569e+16;
        step->P[0][1] = 1.31325e+16;
        step->P[1][0] = 1.31325e+16;
        step->P[1][1] = 4.57009e+16;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.87096;
        step->Xe[1] = 100.876;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.26916;
        step->ell[0][1] = -0.018213;
        step->ell[1][0] = 4.44089e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 4.15508e+16;
        step->P[0][1] = 1.31386e+16;
        step->P[1][0] = 1.31386e+16;
        step->P[1][1] = 4.58214e+16;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.40803;
        step->Xe[1] = 100.883;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.26921;
        step->ell[0][1] = -0.0130851;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 4.1444e+16;
        step->P[0][1] = 1.31413e+16;
        step->P[1][0] = 1.31413e+16;
        step->P[1][1] = 4.59413e+16;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.947899;
        step->Xe[1] = 100.884;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -3.33067e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.2692;
        step->ell[1][1] = 0.00798858;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 4.16294e+16;
        step->P[0][1] = 1.31304e+16;
        step->P[1][0] = 1.31304e+16;
        step->P[1][1] = 4.57298e+16;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.75982;
        step->Xe[1] = 100.872;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26911;
        step->ell[1][1] = 0.0169821;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 4.18158e+16;
        step->P[0][1] = 1.31205e+16;
        step->P[1][0] = 1.31205e+16;
        step->P[1][1] = 4.55197e+16;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.56673;
        step->Xe[1] = 100.861;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.26902;
        step->ell[0][1] = -0.0259203;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = -1.06703e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 4.17107e+16;
        step->P[0][1] = 1.31314e+16;
        step->P[1][0] = 1.31314e+16;
        step->P[1][1] = 4.56412e+16;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 2.09965;
        step->Xe[1] = 100.875;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.26917;
        step->ell[0][1] = -0.0207459;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 4.16049e+16;
        step->P[0][1] = 1.31391e+16;
        step->P[1][0] = 1.31391e+16;
        step->P[1][1] = 4.57622e+16;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.6353;
        step->Xe[1] = 100.884;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.26925;
        step->ell[0][1] = -0.0156021;
        step->ell[1][0] = -3.33067e-16;
        step->ell[1][1] = 3.54057e-15;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 4.14983e+16;
        step->P[0][1] = 1.31435e+16;
        step->P[1][0] = 1.31435e+16;
        step->P[1][1] = 4.58825e+16;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 1.17372;
        step->Xe[1] = 100.888;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 4.44089e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.26927;
        step->ell[1][1] = 0.0104895;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 4.1684e+16;
        step->P[0][1] = 1.31329e+16;
        step->P[1][0] = 1.31329e+16;
        step->P[1][1] = 4.56714e+16;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 1.98425;
        step->Xe[1] = 100.877;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 1.26918;
        step->ell[0][1] = -0.0194676;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = 3.54057e-15;

    }

}
