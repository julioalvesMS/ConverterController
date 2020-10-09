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
            P[0][0] = 0.002002772606;
            P[0][1] = -1.050206365e-05;
            P[1][0] = -1.050206365e-05;
            P[1][1] = 0.002268707902;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.002021901498;
            P[0][1] = -6.215347112e-06;
            P[1][0] = -6.215347112e-06;
            P[1][1] = 0.002281699668;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 154.1619982;
            P[0][1] = 60.17778348;
            P[1][0] = 60.17778348;
            P[1][1] = 210.2827552;
            break;

        default:
            break;
        }
    }



    void BuckBoost3::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 0.5;
        num[1] = -0.49;

        den[0] = 1;
        den[1] = -1;
    }



    void BuckBoost3::GetCurrentCorrectionController(double num[2], double den[2], double *designVoltage)
    {
        (*designVoltage) = 100;

        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter
            //
            num[0] = 1.5;
            num[1] = -1.4;

            den[0] = 1;
            den[1] = -1;
            break;

        default:
            break;
        }
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
        subSys->A[0][0] = -247.3498233;
        subSys->A[0][1] = -504.7955578;
        subSys->A[1][0] = 444.4444444;
        subSys->A[1][1] = -4.591368228;
        //
        // Subsystem 1 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = -247.3498233;
        subSys->Ar0[0][1] = -504.7955578;
        subSys->Ar0[1][0] = 444.4444444;
        subSys->Ar0[1][1] = 0;
        //
        // Subsystem 1 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -444.4444444;
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->B[0] = 504.7955578;
        subSys->B[1] = 0;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->Q[0][0] = 1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.01033057851;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(system.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A[0][0] = -247.3498233;
        subSys->A[0][1] = -504.7955578;
        subSys->A[1][0] = 444.4444444;
        subSys->A[1][1] = -4.591368228;
        //
        // Subsystem 2 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = -247.3498233;
        subSys->Ar0[0][1] = -504.7955578;
        subSys->Ar0[1][0] = 444.4444444;
        subSys->Ar0[1][1] = 0;
        //
        // Subsystem 2 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -444.4444444;
        //
        // Subsystem 2 -- Matrix B
        //
        subSys->B[0] = 0;
        subSys->B[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.01033057851;


        //
        // =============== Subsystem 3 ===============
        //
        subSys = &(system.subSystems[2]);

        //
        // Subsystem 3 -- Matrix A
        //
        subSys->A[0][0] = -247.3498233;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = -4.591368228;
        //
        // Subsystem 3 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = -247.3498233;
        subSys->Ar0[0][1] = 0;
        subSys->Ar0[1][0] = 0;
        subSys->Ar0[1][1] = 0;
        //
        // Subsystem 3 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -444.4444444;
        //
        // Subsystem 3 -- Matrix B
        //
        subSys->B[0] = 504.7955578;
        subSys->B[1] = 0;
        //
        // Subsystem 3 -- Matrix Q
        //
        subSys->Q[0][0] = 1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.01033057851;

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
        subSys->A[0][0] = 0.9937655158;
        subSys->A[0][1] = -0.01257993339;
        subSys->A[1][0] = 0.01107593247;
        subSys->A[1][1] = 0.9998152624;
        //
        // Subsystem 1 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = 0.9937655131;
        subSys->Ar0[0][1] = -0.01258065614;
        subSys->Ar0[1][0] = 0.01107656881;
        subSys->Ar0[1][1] = 0.9999300346;
        //
        // Subsystem 1 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -0.01111111111;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0][0] = -0.00623448422;
        subSys->L[0][1] = -0.01257993339;
        subSys->L[0][2] = 0.01258065615;
        subSys->L[1][0] = 0.01107593247;
        subSys->L[1][1] = -0.0001847376467;
        subSys->L[1][2] = 6.996270663e-05;
        //
        // Subsystem 1 -- Matrix Q
        //
        subSys->E[0][0] = 1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.01033057851;
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
        subSys->Q[1][1] = 0.01033057851;


        //
        // =============== Subsystem 2 ===============
        //
        subSys = &(discreteSystem.subSystems[1]);

        //
        // Subsystem 2 -- Matrix A
        //
        subSys->A[0][0] = 0.9937655158;
        subSys->A[0][1] = -0.01257993339;
        subSys->A[1][0] = 0.01107593247;
        subSys->A[1][1] = 0.9998152624;
        //
        // Subsystem 2 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = 0.9937655131;
        subSys->Ar0[0][1] = -0.01258065614;
        subSys->Ar0[1][0] = 0.01107656881;
        subSys->Ar0[1][1] = 0.9999300346;
        //
        // Subsystem 2 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -0.01111111111;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0][0] = -0.00623448422;
        subSys->L[0][1] = -0.01257993339;
        subSys->L[0][2] = 0;
        subSys->L[1][0] = 0.01107593247;
        subSys->L[1][1] = -0.0001847376467;
        subSys->L[1][2] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->E[0][0] = 1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.01033057851;
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
        subSys->Q[1][1] = 0.01033057851;


        //
        // =============== Subsystem 3 ===============
        //
        subSys = &(discreteSystem.subSystems[2]);

        //
        // Subsystem 3 -- Matrix A
        //
        subSys->A[0][0] = 0.9938353344;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.9998852224;
        //
        // Subsystem 3 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = 0.9938353344;
        subSys->Ar0[0][1] = 0;
        subSys->Ar0[1][0] = 0;
        subSys->Ar0[1][1] = 1;
        //
        // Subsystem 3 -- Matrix Ard
        //
        subSys->Ard[0][0] = 0;
        subSys->Ard[0][1] = 0;
        subSys->Ard[1][0] = 0;
        subSys->Ard[1][1] = -0.01111111111;
        //
        // Subsystem 3 -- Matrix L
        //
        subSys->L[0][0] = -0.006164665577;
        subSys->L[0][1] = 0;
        subSys->L[0][2] = 0.01258095016;
        subSys->L[1][0] = 0;
        subSys->L[1][1] = -0.0001147776182;
        subSys->L[1][2] = 0;
        //
        // Subsystem 3 -- Matrix Q
        //
        subSys->E[0][0] = 1;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.01033057851;
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
        subSys->Q[1][1] = 0.01033057851;

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
        step->P[0][0] = 80.61872466;
        step->P[0][1] = -0.5155124367;
        step->P[1][0] = -0.5155124367;
        step->P[1][1] = 90.62389114;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.6356722535;
        step->Xe[1] = 80.61377457;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.014179407;
        step->ell[0][1] = 0.005948496923;
        step->ell[1][0] = -1.831922057;
        step->ell[1][1] = 0.001400920992;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 80.60951128;
        step->P[0][1] = -0.5187750251;
        step->P[1][0] = -0.5187750251;
        step->P[1][1] = 90.63435619;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.449515307;
        step->Xe[1] = 80.60452191;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014119831;
        step->ell[2][1] = -0.01496321494;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 80.61179847;
        step->P[0][1] = -0.5180540294;
        step->P[1][0] = -0.5180540294;
        step->P[1][1] = 90.63171096;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.244221459;
        step->Xe[1] = 80.61023353;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = -1.06702841e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014177349;
        step->ell[2][1] = -0.01268899456;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 80.61409727;
        step->P[0][1] = -0.5172701442;
        step->P[1][0] = -0.5172701442;
        step->P[1][1] = 90.62908398;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.040135661;
        step->Xe[1] = 80.61367027;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 8.881784197e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206334;
        step->ell[2][1] = -0.01042831361;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 80.61640643;
        step->P[0][1] = -0.5164230443;
        step->P[1][0] = -0.5164230443;
        step->P[1][1] = 90.62647684;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.837278999;
        step->Xe[1] = 80.61484594;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -1.06702841e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206961;
        step->ell[2][1] = -0.008181404668;

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
        step->P[0][0] = 2.882098515e+17;
        step->P[0][1] = 8.855501101e+16;
        step->P[1][0] = 8.855501101e+16;
        step->P[1][1] = 3.634857225e+17;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.449515307;
        step->Xe[1] = 80.60452191;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -1.06702841e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014119831;
        step->ell[2][1] = -0.01496321494;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2.878604697e+17;
        step->P[0][1] = 8.849667409e+16;
        step->P[1][0] = 8.849667409e+16;
        step->P[1][1] = 3.639459178e+17;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.244221459;
        step->Xe[1] = 80.61023353;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014177349;
        step->ell[2][1] = -0.01268899456;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2.874889264e+17;
        step->P[0][1] = 8.842304503e+16;
        step->P[1][0] = 8.842304503e+16;
        step->P[1][1] = 3.643988228e+17;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.040135661;
        step->Xe[1] = 80.61367027;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206334;
        step->ell[2][1] = -0.01042831361;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 2.870951797e+17;
        step->P[0][1] = 8.83336196e+16;
        step->P[1][0] = 8.83336196e+16;
        step->P[1][1] = 3.648439076e+17;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.837278999;
        step->Xe[1] = 80.61484594;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.221245327e-15;
        step->ell[0][1] = 1.775142533e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206961;
        step->ell[2][1] = -0.008181404668;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 2.866792018e+17;
        step->P[0][1] = 8.822789484e+16;
        step->P[1][0] = 8.822789484e+16;
        step->P[1][1] = 3.652806241e+17;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.6356722535;
        step->Xe[1] = 80.61377457;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.014179407;
        step->ell[0][1] = 0.005948496923;
        step->ell[1][0] = -1.831922057;
        step->ell[1][1] = 0.001400920992;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 5;
        limitCycle.rho = 5.103188686e-23;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 8.433419599e+22;
        step->P[0][1] = 2.334834912e+22;
        step->P[1][0] = 2.334834912e+22;
        step->P[1][1] = 9.340104304e+22;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.244221459;
        step->Xe[1] = 80.61023353;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -1.06702841e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014177349;
        step->ell[2][1] = -0.01268899456;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 8.423069827e+22;
        step->P[0][1] = 2.335466041e+22;
        step->P[1][0] = 2.335466041e+22;
        step->P[1][1] = 9.351930653e+22;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.040135661;
        step->Xe[1] = 80.61367027;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -4.440892099e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206334;
        step->ell[2][1] = -0.01042831361;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 8.412654981e+22;
        step->P[0][1] = 2.335789579e+22;
        step->P[1][0] = 2.335789579e+22;
        step->P[1][1] = 9.363687864e+22;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.837278999;
        step->Xe[1] = 80.61484594;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -3.330669074e-16;
        step->ell[0][1] = -1.06702841e-14;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014206961;
        step->ell[2][1] = -0.008181404668;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 8.402181844e+22;
        step->P[0][1] = 2.335805795e+22;
        step->P[1][0] = 2.335805795e+22;
        step->P[1][1] = 9.375368528e+22;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.6356722535;
        step->Xe[1] = 80.61377457;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.014179407;
        step->ell[0][1] = 0.005948496923;
        step->ell[1][0] = -1.831922057;
        step->ell[1][1] = 0.001400920992;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 8.44369743e+22;
        step->P[0][1] = 2.333896007e+22;
        step->P[1][0] = 2.333896007e+22;
        step->P[1][1] = 9.328216228e+22;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.449515307;
        step->Xe[1] = 80.60452191;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;
        step->ell[2][0] = 1.014119831;
        step->ell[2][1] = -0.01496321494;

    }

}
