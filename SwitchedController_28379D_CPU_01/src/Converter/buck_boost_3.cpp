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
            P[0][0] = 0.00200292;
            P[0][1] = -1.71799e-05;
            P[1][0] = -1.71799e-05;
            P[1][1] = 0.00225865;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.00202192;
            P[0][1] = -6.35027e-06;
            P[1][0] = -6.35027e-06;
            P[1][1] = 0.00228168;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 1.77267e-06;
            P[0][1] = 8.29183e-07;
            P[1][0] = 8.29183e-07;
            P[1][1] = 4.62762e-06;
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
            h[0] = 9.24223e-07;
            h[1] = 1.27314e-06;
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
            d = 6.48616e-07;
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
        subSys->A[0][0] = 0.993766;
        subSys->A[0][1] = -0.0125799;
        subSys->A[1][0] = 0.0110759;
        subSys->A[1][1] = 0.999815;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = -0.00408392;
        subSys->L[1] = 0.000123671;
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
        subSys->L[0] = -0.0166646;
        subSys->L[1] = 5.37084e-05;
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
        subSys->A[0][0] = 0.993835;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.999885;
        //
        // Subsystem 3 -- Matrix L
        //
        subSys->L[0] = 0.0124162;
        subSys->L[1] = -0.000150525;
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
        step->P[0][0] = 80.6187;
        step->P[0][1] = -0.515512;
        step->P[1][0] = -0.515512;
        step->P[1][1] = 90.6239;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.635672;
        step->Xe[1] = 80.6138;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.01418;
        step->ell[0][1] = 0.0059485;
        step->ell[1][0] = -1.83192;
        step->ell[1][1] = 0.00140092;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 80.6095;
        step->P[0][1] = -0.518775;
        step->P[1][0] = -0.518775;
        step->P[1][1] = 90.6344;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.44952;
        step->Xe[1] = 80.6045;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01412;
        step->ell[2][1] = -0.0149632;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 80.6118;
        step->P[0][1] = -0.518054;
        step->P[1][0] = -0.518054;
        step->P[1][1] = 90.6317;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.24422;
        step->Xe[1] = 80.6102;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 3.33067e-16;
        step->ell[0][1] = -1.06703e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01418;
        step->ell[2][1] = -0.012689;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 80.6141;
        step->P[0][1] = -0.51727;
        step->P[1][0] = -0.51727;
        step->P[1][1] = 90.6291;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.04014;
        step->Xe[1] = 80.6137;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 8.88178e-16;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0104283;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 80.6164;
        step->P[0][1] = -0.516423;
        step->P[1][0] = -0.516423;
        step->P[1][1] = 90.6265;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.837279;
        step->Xe[1] = 80.6148;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -1.06703e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0081814;

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
        step->P[0][0] = 2.8821e+17;
        step->P[0][1] = 8.8555e+16;
        step->P[1][0] = 8.8555e+16;
        step->P[1][1] = 3.63486e+17;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.44952;
        step->Xe[1] = 80.6045;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = -1.06703e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01412;
        step->ell[2][1] = -0.0149632;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2.8786e+17;
        step->P[0][1] = 8.84967e+16;
        step->P[1][0] = 8.84967e+16;
        step->P[1][1] = 3.63946e+17;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.24422;
        step->Xe[1] = 80.6102;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.11022e-16;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01418;
        step->ell[2][1] = -0.012689;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2.87489e+17;
        step->P[0][1] = 8.8423e+16;
        step->P[1][0] = 8.8423e+16;
        step->P[1][1] = 3.64399e+17;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.04014;
        step->Xe[1] = 80.6137;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0104283;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 2.87095e+17;
        step->P[0][1] = 8.83336e+16;
        step->P[1][0] = 8.83336e+16;
        step->P[1][1] = 3.64844e+17;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.837279;
        step->Xe[1] = 80.6148;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.22125e-15;
        step->ell[0][1] = 1.77514e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0081814;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 2.86679e+17;
        step->P[0][1] = 8.82279e+16;
        step->P[1][0] = 8.82279e+16;
        step->P[1][1] = 3.65281e+17;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.635672;
        step->Xe[1] = 80.6138;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.01418;
        step->ell[0][1] = 0.0059485;
        step->ell[1][0] = -1.83192;
        step->ell[1][1] = 0.00140092;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 5;
        limitCycle.rho = 5.10319e-23;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 8.43342e+22;
        step->P[0][1] = 2.33483e+22;
        step->P[1][0] = 2.33483e+22;
        step->P[1][1] = 9.3401e+22;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.24422;
        step->Xe[1] = 80.6102;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = -1.06703e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01418;
        step->ell[2][1] = -0.012689;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 8.42307e+22;
        step->P[0][1] = 2.33547e+22;
        step->P[1][0] = 2.33547e+22;
        step->P[1][1] = 9.35193e+22;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.04014;
        step->Xe[1] = 80.6137;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -4.44089e-16;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0104283;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 8.41265e+22;
        step->P[0][1] = 2.33579e+22;
        step->P[1][0] = 2.33579e+22;
        step->P[1][1] = 9.36369e+22;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.837279;
        step->Xe[1] = 80.6148;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -3.33067e-16;
        step->ell[0][1] = -1.06703e-14;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01421;
        step->ell[2][1] = -0.0081814;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 8.40218e+22;
        step->P[0][1] = 2.33581e+22;
        step->P[1][0] = 2.33581e+22;
        step->P[1][1] = 9.37537e+22;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.635672;
        step->Xe[1] = 80.6138;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.01418;
        step->ell[0][1] = 0.0059485;
        step->ell[1][0] = -1.83192;
        step->ell[1][1] = 0.00140092;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 8.4437e+22;
        step->P[0][1] = 2.3339e+22;
        step->P[1][0] = 2.3339e+22;
        step->P[1][1] = 9.32822e+22;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.44952;
        step->Xe[1] = 80.6045;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.11022e-16;
        step->ell[0][1] = 3.54057e-15;
        step->ell[1][0] = -0.817743;
        step->ell[1][1] = -0.00454758;
        step->ell[2][0] = 1.01412;
        step->ell[2][1] = -0.0149632;

    }

}
