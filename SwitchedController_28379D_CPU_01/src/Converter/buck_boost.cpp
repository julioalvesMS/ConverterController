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
            P[0][0] = 0.00021615;
            P[0][1] = 1.76357e-05;
            P[1][0] = 1.76357e-05;
            P[1][1] = 0.000268118;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Rule 2
            //
            P[0][0] = 0.000988005;
            P[0][1] = 6.20027e-07;
            P[1][0] = 6.20027e-07;
            P[1][1] = 0.00112501;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            P[0][0] = 7.25177e-07;
            P[0][1] = 4.6514e-07;
            P[1][0] = 4.6514e-07;
            P[1][1] = 2.44934e-06;
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
            h[0] = 6.97902e-07;
            h[1] = 1.32223e-06;
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
            d = 1.02725e-06;
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



    void BuckBoost::GetStateFeedbackH2Controller(double K[2], double C[2], double* M)
    {
        K[0] = 1.21908;
        K[1] = 0.661907;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.677367;
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
        subSys->A[0][0] = 0.993835;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0123567;
        subSys->L[1] = -0.000140285;
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
        subSys->A[0][0] = 0.993766;
        subSys->A[0][1] = -0.0125799;
        subSys->A[1][0] = 0.0110759;
        subSys->A[1][1] = 0.999815;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0] = -0.0156024;
        subSys->L[1] = 0.000177132;
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
        step->P[0][0] = 8.67903;
        step->P[0][1] = 0.690395;
        step->P[1][0] = 0.690395;
        step->P[1][1] = 10.7396;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.172915;
        step->Xe[1] = 79.2168;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81429;
        step->ell[1][1] = -0.00745721;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 8.68579;
        step->P[0][1] = 0.694757;
        step->P[1][0] = 0.694757;
        step->P[1][1] = 10.7317;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.645912;
        step->Xe[1] = 79.2077;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81423;
        step->ell[1][1] = 0.00161271;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 8.69263;
        step->P[0][1] = 0.699146;
        step->P[1][0] = 0.699146;
        step->P[1][1] = 10.7239;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.45969;
        step->Xe[1] = 79.1986;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.81418;
        step->ell[1][1] = 0.0106267;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 8.69956;
        step->P[0][1] = 0.703563;
        step->P[1][0] = 0.703563;
        step->P[1][1] = 10.716;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.26846;
        step->Xe[1] = 79.1895;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -6.66134e-16;
        step->ell[0][1] = -1.42109e-14;
        step->ell[1][0] = -1.81412;
        step->ell[1][1] = 0.0195852;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 8.70658;
        step->P[0][1] = 0.708008;
        step->P[1][0] = 0.708008;
        step->P[1][1] = 10.7081;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 3.07223;
        step->Xe[1] = 79.1804;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.81406;
        step->ell[1][1] = 0.0284884;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 8.71368;
        step->P[0][1] = 0.712481;
        step->P[1][0] = 0.712481;
        step->P[1][1] = 10.7003;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 3.87106;
        step->Xe[1] = 79.1713;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.814;
        step->ell[0][1] = -0.0373367;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.42109e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 8.705;
        step->P[0][1] = 0.70734;
        step->P[1][0] = 0.70734;
        step->P[1][1] = 10.7103;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.85095;
        step->Xe[1] = 79.1996;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.81429;
        step->ell[0][1] = -0.0260361;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 8.69632;
        step->P[0][1] = 0.701945;
        step->P[1][0] = 0.701945;
        step->P[1][1] = 10.7202;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.83685;
        step->Xe[1] = 79.2165;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.81443;
        step->ell[0][1] = -0.0148029;
        step->ell[1][0] = -1.11022e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 8.68767;
        step->P[0][1] = 0.696297;
        step->P[1][0] = 0.696297;
        step->P[1][1] = 10.73;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.828862;
        step->Xe[1] = 79.2222;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.81443;
        step->ell[0][1] = -0.00363803;
        step->ell[1][0] = 1.11022e-16;
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
        step->P[0][0] = 1.33172e+12;
        step->P[0][1] = 6.46612e+11;
        step->P[1][0] = 6.46612e+11;
        step->P[1][1] = 1.85545e+12;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.44508;
        step->Xe[1] = 79.2154;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81439;
        step->ell[1][1] = 0.0104637;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.33845e+12;
        step->P[0][1] = 6.48366e+11;
        step->P[1][0] = 6.48366e+11;
        step->P[1][1] = 1.84826e+12;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.25393;
        step->Xe[1] = 79.2063;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81433;
        step->ell[1][1] = 0.0194231;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.34515e+12;
        step->P[0][1] = 6.50124e+11;
        step->P[1][0] = 6.50124e+11;
        step->P[1][1] = 1.84107e+12;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 3.0578;
        step->Xe[1] = 79.1972;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.81427;
        step->ell[0][1] = -0.0283273;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.33735e+12;
        step->P[0][1] = 6.48274e+11;
        step->P[1][0] = 6.48274e+11;
        step->P[1][1] = 1.85024e+12;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.04244;
        step->Xe[1] = 79.2164;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.81444;
        step->ell[0][1] = -0.0170799;
        step->ell[1][0] = 0;
        step->ell[1][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.32943e+12;
        step->P[0][1] = 6.46163e+11;
        step->P[1][0] = 6.46163e+11;
        step->P[1][1] = 1.85932e+12;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.03317;
        step->Xe[1] = 79.2244;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81447;
        step->ell[1][1] = 0.00590077;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.3358e+12;
        step->P[0][1] = 6.47805e+11;
        step->P[1][0] = 6.47805e+11;
        step->P[1][1] = 1.85204e+12;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.84456;
        step->Xe[1] = 79.2153;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 6.66134e-16;
        step->ell[0][1] = -1.42109e-14;
        step->ell[1][0] = -1.81441;
        step->ell[1][1] = 0.0148883;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.34215e+12;
        step->P[0][1] = 6.49451e+11;
        step->P[1][0] = 6.49451e+11;
        step->P[1][1] = 1.84477e+12;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.65095;
        step->Xe[1] = 79.2063;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.81436;
        step->ell[0][1] = -0.0238205;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.33399e+12;
        step->P[0][1] = 6.47406e+11;
        step->P[1][0] = 6.47406e+11;
        step->P[1][1] = 1.85385e+12;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.63802;
        step->Xe[1] = 79.221;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.81447;
        step->ell[0][1] = -0.0126003;
        step->ell[1][0] = 1.11022e-16;
        step->ell[1][1] = -1.42109e-14;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.32572e+12;
        step->P[0][1] = 6.45092e+11;
        step->P[1][0] = 6.45092e+11;
        step->P[1][1] = 1.86282e+12;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.631209;
        step->Xe[1] = 79.2245;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81444;
        step->ell[1][1] = 0.00144868;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 9;
        limitCycle.rho = 1.81769e-34;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.45998e+30;
        step->P[0][1] = 5.23843e+29;
        step->P[1][0] = 5.23843e+29;
        step->P[1][1] = 1.56938e+30;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 2.25087;
        step->Xe[1] = 79.2092;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.81436;
        step->ell[0][1] = -0.0193891;
        step->ell[1][0] = 2.22045e-16;
        step->ell[1][1] = -1.42109e-14;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.45351e+30;
        step->P[0][1] = 5.24334e+29;
        step->P[1][0] = 5.24334e+29;
        step->P[1][1] = 1.57673e+30;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.2404;
        step->Xe[1] = 79.2195;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81442;
        step->ell[1][1] = 0.00819634;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.45868e+30;
        step->P[0][1] = 5.23915e+29;
        step->P[1][0] = 5.23915e+29;
        step->P[1][1] = 1.57084e+30;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.05051;
        step->Xe[1] = 79.2104;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -6.66134e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81437;
        step->ell[1][1] = 0.0171698;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.46387e+30;
        step->P[0][1] = 5.23534e+29;
        step->P[1][0] = 5.23534e+29;
        step->P[1][1] = 1.56499e+30;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.85563;
        step->Xe[1] = 79.2013;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.81431;
        step->ell[0][1] = -0.0260879;
        step->ell[1][0] = 4.44089e-16;
        step->ell[1][1] = 1.42109e-14;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.45743e+30;
        step->P[0][1] = 5.24152e+29;
        step->P[1][0] = 5.24152e+29;
        step->P[1][1] = 1.57236e+30;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.84148;
        step->Xe[1] = 79.2183;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.81445;
        step->ell[1][1] = 0.014854;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.46262e+30;
        step->P[0][1] = 5.23762e+29;
        step->P[1][0] = 5.23762e+29;
        step->P[1][1] = 1.5665e+30;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.64789;
        step->Xe[1] = 79.2092;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.81439;
        step->ell[0][1] = -0.0237864;
        step->ell[1][0] = -2.22045e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.45616e+30;
        step->P[0][1] = 5.24338e+29;
        step->P[1][0] = 5.24338e+29;
        step->P[1][1] = 1.57386e+30;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.63494;
        step->Xe[1] = 79.2239;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.81451;
        step->ell[0][1] = -0.0125659;
        step->ell[1][0] = -4.44089e-16;
        step->ell[1][1] = -1.42109e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.44965e+30;
        step->P[0][1] = 5.24701e+29;
        step->P[1][0] = 5.24701e+29;
        step->P[1][1] = 1.58118e+30;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.628112;
        step->Xe[1] = 79.2274;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = 1.42109e-14;
        step->ell[1][0] = -1.81448;
        step->ell[1][1] = 0.00141418;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.4548e+30;
        step->P[0][1] = 5.24253e+29;
        step->P[1][0] = 5.24253e+29;
        step->P[1][1] = 1.57526e+30;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.442;
        step->Xe[1] = 79.2183;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -2.22045e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81442;
        step->ell[1][1] = 0.0104294;

    }

}
