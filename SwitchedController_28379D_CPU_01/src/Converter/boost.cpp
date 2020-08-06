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
            P[0][0] = 0.001505080018;
            P[0][1] = 0.0005297029297;
            P[1][0] = 0.0005297029297;
            P[1][1] = 0.002264849911;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.02310283858;
            P[0][1] = 0.0011663968;
            P[1][0] = 0.0011663968;
            P[1][1] = 0.03461006052;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 5.932438395e-07;
            P[0][1] = 3.159763018e-07;
            P[1][0] = 3.159763018e-07;
            P[1][1] = 1.686928764e-06;
            break;

        default:
            break;
        }
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
        vNum[1] = -0.3158385;

        vDen[0] = 1;
        vDen[1] = -1;

        iNum[0] = 0.0203;
        iNum[1] = -0.0200615;

        iDen[0] = 1;
        iDen[1] = -1;
    }



    void Boost::GetStateFeedbackH2Controller(double K[2], double C[2], double* M)
    {
        K[0] = 1.200606059;
        K[1] = 0.6518776693;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.6671040097;
    }



    void Boost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9847;

        den[0] = 1;
        den[1] = -1;
    }



    void Boost::GetCurrentCorrectionController(double num[2], double den[2])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Continuous
            //
            num[0] = 0.5;
            num[1] = -0.475;

            den[0] = 1;
            den[1] = -1;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete
            //
            num[0] = 0.5;
            num[1] = -0.475;

            den[0] = 1;
            den[1] = -1;
            break;

        default:
            break;
        }
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
        subSys->A[0][0] = -247.3498233;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = -4.591368228;
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
        subSys->Q[1][1] = 0.3099173554;


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
        // Subsystem 2 -- Matrix B
        //
        subSys->B[0] = 504.7955578;
        subSys->B[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.3099173554;

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
        subSys->A[0][0] = 0.9938353344;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.9998852224;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0][0] = -0.006164665577;
        subSys->L[0][1] = 0;
        subSys->L[0][2] = 0.01258095016;
        subSys->L[1][0] = 0;
        subSys->L[1][1] = -0.0001147776182;
        subSys->L[1][2] = 0;
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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.3099173554;


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
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0][0] = -0.00623448422;
        subSys->L[0][1] = -0.01257993339;
        subSys->L[0][2] = 0.01258065615;
        subSys->L[1][0] = 0.01107593247;
        subSys->L[1][1] = -0.0001847376467;
        subSys->L[1][2] = 6.996270663e-05;
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
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.3099173554;

    }



    void DefineLimitCycleCost()
    {
        CycleStep* step;

        limitCycle.kappa = 3;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 60.44296673;
        step->P[0][1] = 17.22845641;
        step->P[1][0] = 17.22845641;
        step->P[1][1] = 83.51212509;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.105865135;
        step->Xe[1] = 97.88893633;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -1.231532914;
        step->ell[1][1] = 0.01001771342;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 60.69902055;
        step->P[0][1] = 17.33730384;
        step->P[1][0] = 17.33730384;
        step->P[1][1] = 83.22129826;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.929390556;
        step->Xe[1] = 97.87770087;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.23144907;
        step->ell[0][1] = -0.01913981141;
        step->ell[1][0] = -1.110223025e-16;
        step->ell[1][1] = 2.938621568e-15;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 60.57118068;
        step->P[0][1] = 17.28466737;
        step->P[1][0] = 17.28466737;
        step->P[1][1] = 83.36738191;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.516390149;
        step->Xe[1] = 97.88560651;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.231519687;
        step->ell[0][1] = -0.01456489371;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = 2.938621568e-15;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 3;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 6.519021402e+17;
        step->P[0][1] = 2.240819605e+17;
        step->P[1][0] = 2.240819605e+17;
        step->P[1][1] = 8.076803411e+17;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.516390149;
        step->Xe[1] = 97.88560651;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.231519687;
        step->ell[0][1] = -0.01456489371;
        step->ell[1][0] = 3.330669074e-16;
        step->ell[1][1] = 1.714947628e-14;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 6.502821391e+17;
        step->P[0][1] = 2.238245071e+17;
        step->P[1][0] = 2.238245071e+17;
        step->P[1][1] = 8.095747415e+17;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.105865135;
        step->Xe[1] = 97.88893633;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 4.440892099e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.231532914;
        step->ell[1][1] = 0.01001771342;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 6.536021702e+17;
        step->P[0][1] = 2.243226981e+17;
        step->P[1][0] = 2.243226981e+17;
        step->P[1][1] = 8.058042938e+17;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.929390556;
        step->Xe[1] = 97.87770087;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.23144907;
        step->ell[0][1] = -0.01913981141;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.127223315e-14;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 3;
        limitCycle.rho = 2.00517534e-20;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 2.860891906e+29;
        step->P[0][1] = 8.807952696e+28;
        step->P[1][0] = 8.807952696e+28;
        step->P[1][1] = 3.13857956e+29;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.929390556;
        step->Xe[1] = 97.87770087;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.23144907;
        step->ell[0][1] = -0.01913981141;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 2.938621568e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2.854362558e+29;
        step->P[0][1] = 8.812388659e+28;
        step->P[1][0] = 8.812388659e+28;
        step->P[1][1] = 3.146014336e+29;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.516390149;
        step->Xe[1] = 97.88560651;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.231519687;
        step->ell[0][1] = -0.01456489371;
        step->ell[1][0] = 5.551115123e-16;
        step->ell[1][1] = -1.127223315e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2.847825019e+29;
        step->P[0][1] = 8.814943886e+28;
        step->P[1][0] = 8.814943886e+28;
        step->P[1][1] = 3.153400399e+29;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.105865135;
        step->Xe[1] = 97.88893633;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.231532914;
        step->ell[1][1] = 0.01001771342;

    }

}
