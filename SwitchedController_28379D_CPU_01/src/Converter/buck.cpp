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
            P[0][0] = 0.006403021076;
            P[0][1] = 0.002988408223;
            P[1][0] = 0.002988408223;
            P[1][1] = 0.008962378313;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Buck Converter - Rule 2
            //
            P[0][0] = 0.006403073046;
            P[0][1] = 0.002988426322;
            P[1][0] = 0.002988426322;
            P[1][1] = 0.008962468963;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            P[0][0] = 5.986936969;
            P[0][1] = 2.327900964;
            P[1][0] = 2.327900964;
            P[1][1] = 8.142682586;
            break;

        default:
            break;
        }
    }



    void Buck::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.00676;
        num[1] = -0.0066035;

        den[0] = 1;
        den[1] = -1;
    }



    void Buck::GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
    {
        vNum[0] = 0.316;
        vNum[1] = -0.3158385;

        vDen[0] = 1;
        vDen[1] = -1;

        iNum[0] = 0.0203;
        iNum[1] = -0.0200615;

        iDen[0] = 1;
        iDen[1] = -1;
    }



    void Buck::GetStateFeedbackH2Controller(double K[2], double* M)
    {
        K[0] = 0.6090444634;
        K[1] = 0.2895357471;

        (*M) = 0.3112900208;
    }



    void Buck::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9;

        den[0] = 1;
        den[1] = -1;
    }



    void Buck::GetCurrentCorrectionController(double num[2], double den[2], double *designVoltage)
    {
        (*designVoltage) = 40;

        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter
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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 3.099173554;


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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 3.099173554;

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
        subSys->E[0][0] = 0;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.001033057851;
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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 3.099173554;


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
        subSys->E[0][0] = 0;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 0.001033057851;
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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 3.099173554;

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
        step->P[0][0] = 254.6682236;
        step->P[0][1] = 119.5428527;
        step->P[1][0] = 119.5428527;
        step->P[1][1] = 358.2866392;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -1.015837635;
        step->Xe[1] = 29.84612534;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 254.6682229;
        step->P[0][1] = 119.542852;
        step->P[1][0] = 119.542852;
        step->P[1][1] = 358.2866389;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = -0.5672240304;
        step->Xe[1] = 29.83390787;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 254.6682225;
        step->P[0][1] = 119.5428514;
        step->P[1][0] = 119.5428514;
        step->P[1][1] = 358.2866385;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = -0.1212536052;
        step->Xe[1] = 29.82666146;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -5.551115123e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 254.6682221;
        step->P[0][1] = 119.5428509;
        step->P[1][0] = 119.5428509;
        step->P[1][1] = 358.2866382;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.3220275836;
        step->Xe[1] = 29.82435593;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -3.330669074e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 254.6682218;
        step->P[0][1] = 119.5428505;
        step->P[1][0] = 119.5428505;
        step->P[1][1] = 358.2866379;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.7625741462;
        step->Xe[1] = 29.82696058;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -3.330669074e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 254.6682216;
        step->P[0][1] = 119.5428503;
        step->P[1][0] = 119.5428503;
        step->P[1][1] = 358.2866377;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.200341362;
        step->Xe[1] = 29.83444422;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 5.551115123e-16;
        step->ell[0][1] = -3.564856743e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 254.6682215;
        step->P[0][1] = 119.5428501;
        step->P[1][0] = 119.5428501;
        step->P[1][1] = 358.2866376;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.635285181;
        step->Xe[1] = 29.84677515;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 254.6682215;
        step->P[0][1] = 119.5428501;
        step->P[1][0] = 119.5428501;
        step->P[1][1] = 358.2866375;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.249619578;
        step->Xe[1] = 29.85937363;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -3.330669074e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 254.6682216;
        step->P[0][1] = 119.5428502;
        step->P[1][0] = 119.5428502;
        step->P[1][1] = 358.2866375;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.8661999132;
        step->Xe[1] = 29.86769818;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 254.6682218;
        step->P[0][1] = 119.5428504;
        step->P[1][0] = 119.5428504;
        step->P[1][1] = 358.2866376;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.4850659497;
        step->Xe[1] = 29.87177447;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 1.665334537e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 254.6682221;
        step->P[0][1] = 119.5428508;
        step->P[1][0] = 119.5428508;
        step->P[1][1] = 358.2866379;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.1062568805;
        step->Xe[1] = 29.87162858;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 3.885780586e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 254.6682225;
        step->P[0][1] = 119.5428513;
        step->P[1][0] = 119.5428513;
        step->P[1][1] = 358.2866382;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = -0.2701886743;
        step->Xe[1] = 29.86728706;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -6.661338148e-16;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 254.668223;
        step->P[0][1] = 119.5428519;
        step->P[1][0] = 119.5428519;
        step->P[1][1] = 358.2866386;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = -0.6442326692;
        step->Xe[1] = 29.85877686;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

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
        step->P[0][0] = 6.76101121e+15;
        step->P[0][1] = 1.639636072e+15;
        step->P[1][0] = 1.639636072e+15;
        step->P[1][1] = 9.098361184e+15;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.001048064686;
        step->Xe[1] = 29.83805867;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 6.761152047e+15;
        step->P[0][1] = 1.639487737e+15;
        step->P[1][0] = 1.639487737e+15;
        step->P[1][1] = 9.09145574e+15;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.4413403285;
        step->Xe[1] = 29.83708243;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -3.564856743e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 6.761210916e+15;
        step->P[0][1] = 1.639460254e+15;
        step->P[1][0] = 1.639460254e+15;
        step->P[1][1] = 9.085700985e+15;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.8809829392;
        step->Xe[1] = 29.84100623;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -4.440892099e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 6.761199214e+15;
        step->P[0][1] = 1.639526278e+15;
        step->P[1][0] = 1.639526278e+15;
        step->P[1][1] = 9.081099653e+15;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.317835244;
        step->Xe[1] = 29.84979875;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 6.761130107e+15;
        step->P[0][1] = 1.63965879e+15;
        step->P[1][0] = 1.63965879e+15;
        step->P[1][1] = 9.077653354e+15;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.9341107408;
        step->Xe[1] = 29.85888063;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -1.110223025e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 6.76101857e+15;
        step->P[0][1] = 1.639831166e+15;
        step->P[1][0] = 1.639831166e+15;
        step->P[1][1] = 9.075362599e+15;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.5526643126;
        step->Xe[1] = 29.86371071;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -2.775557562e-17;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 6.760881407e+15;
        step->P[0][1] = 1.640017244e+15;
        step->P[1][0] = 1.640017244e+15;
        step->P[1][1] = 9.074226828e+15;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.173535244;
        step->Xe[1] = 29.86431503;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 1.942890293e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 6.760737279e+15;
        step->P[0][1] = 1.640191399e+15;
        step->P[1][0] = 1.640191399e+15;
        step->P[1][1] = 9.074244447e+15;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = -0.2032377528;
        step->Xe[1] = 29.86072004;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 6.760606732e+15;
        step->P[0][1] = 1.640328608e+15;
        step->P[1][0] = 1.640328608e+15;
        step->P[1][1] = 9.075412861e+15;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = -0.5776165393;
        step->Xe[1] = 29.85295259;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 6.760512214e+15;
        step->P[0][1] = 1.640404526e+15;
        step->P[1][0] = 1.640404526e+15;
        step->P[1][1] = 9.077728515e+15;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = -0.1318209036;
        step->Xe[1] = 29.84558756;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 6.760478101e+15;
        step->P[0][1] = 1.640395554e+15;
        step->P[1][0] = 1.640395554e+15;
        step->P[1][1] = 9.081186932e+15;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.3112880778;
        step->Xe[1] = 29.84316149;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 4.30211422e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 6.760530711e+15;
        step->P[0][1] = 1.640278916e+15;
        step->P[1][0] = 1.640278916e+15;
        step->P[1][1] = 9.085782763e+15;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = -0.06607762664;
        step->Xe[1] = 29.84109614;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 6.760698328e+15;
        step->P[0][1] = 1.640032727e+15;
        step->P[1][0] = 1.640032727e+15;
        step->P[1][1] = 9.091509831e+15;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = 0.3766779811;
        step->Xe[1] = 29.83939907;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 2.623769257e-17;
        step->ell[1][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 13;
        limitCycle.rho = 1.782529606e-22;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.635285181;
        step->Xe[1] = 29.84677515;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.249619578;
        step->Xe[1] = 29.85937363;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.8661999132;
        step->Xe[1] = 29.86769818;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -1.665334537e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.4850659497;
        step->Xe[1] = 29.87177447;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = -6.938893904e-17;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.1062568805;
        step->Xe[1] = 29.87162858;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 1.665334537e-16;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = -0.2701886743;
        step->Xe[1] = 29.86728706;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = -0.6442326692;
        step->Xe[1] = 29.85877686;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8177426497;
        step->ell[0][1] = 0.004547575931;
        step->ell[1][0] = 0;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = -1.015837635;
        step->Xe[1] = 29.84612534;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = -0.5672240304;
        step->Xe[1] = 29.83390787;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = -0.1212536052;
        step->Xe[1] = 29.82666146;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.3220275836;
        step->Xe[1] = 29.82435593;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 6.661338148e-16;
        step->ell[0][1] = 3.540570614e-15;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 12 ===============
        //
        step = &(limitCycle.cycleSteps[11]);

        //
        // Cycle Step 12 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 12 -- Vector Xe
        //
        step->Xe[0] = 0.7625741462;
        step->Xe[1] = 29.82696058;
        //
        // Cycle Step 12 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

        //
        // =============== Limit Cycle Step 13 ===============
        //
        step = &(limitCycle.cycleSteps[12]);

        //
        // Cycle Step 13 -- Matrix P
        //
        step->P[0][0] = 5.101913818e+11;
        step->P[0][1] = 1.212113367e+11;
        step->P[1][0] = 1.212113367e+11;
        step->P[1][1] = 5.704468332e+11;
        //
        // Cycle Step 13 -- Vector Xe
        //
        step->Xe[0] = 1.200341362;
        step->Xe[1] = 29.83444422;
        //
        // Cycle Step 13 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -1.214306433e-17;
        step->ell[1][0] = -0.8177426497;
        step->ell[1][1] = -0.004547575931;

    }

}
