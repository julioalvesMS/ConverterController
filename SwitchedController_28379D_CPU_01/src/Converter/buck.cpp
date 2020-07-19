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
            P[0][0] = 0.0002164400013;
            P[0][1] = 7.956092697e-06;
            P[1][0] = 7.956092697e-06;
            P[1][1] = 0.0002503397528;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Buck Converter - Rule 2
            //
            P[0][0] = 0.0002164377228;
            P[0][1] = 7.955550053e-06;
            P[1][0] = 7.955550053e-06;
            P[1][1] = 0.0002503374837;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            P[0][0] = 0.0004083260881;
            P[0][1] = 0.000111443485;
            P[1][0] = 0.000111443485;
            P[1][1] = 0.0007762778706;
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
            h[0] = 2.086901436e-19;
            h[1] = 1.161967496e-19;
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
            d = 1.113638546e-34;
            break;

        default:
            break;
        }

        return d;
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



    void Buck::GetStateFeedbackH2Controller(double K[2], double C[2], double* M)
    {
        K[0] = 1.200606059;
        K[1] = 0.6518776693;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.6671040097;
    }



    void Buck::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9;

        den[0] = 1;
        den[1] = -1;
    }



    void Buck::GetCurrentCorrectionController(double num[2], double den[2])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
            //
            // Buck Converter - Continuous
            //
            num[0] = 0.5;
            num[1] = -0.495;

            den[0] = 1;
            den[1] = -1;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete
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
        subSys->A[0][0] = 0.9997525686;
        subSys->A[0][1] = -0.0005047319548;
        subSys->A[1][0] = 0.0004443884455;
        subSys->A[1][1] = 0.9999952965;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0002741476298;
        subSys->L[1] = 6.092411492e-08;
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
        subSys->Q[0][0] = 0.1;
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
        subSys->A[0][0] = 0.9997525686;
        subSys->A[0][1] = -0.0005047319548;
        subSys->A[1][0] = 0.0004443884455;
        subSys->A[1][1] = 0.9999952965;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0] = -0.0002305854837;
        subSys->L[1] = -5.124325357e-08;
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
        subSys->Q[0][0] = 0.1;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 0.01033057851;

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
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742945;
        step->P[1][0] = 7.955742945;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.2629267143;
        step->Xe[1] = 29.84890196;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 4.857225733e-17;
        step->ell[0][1] = 2.515752228e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046318e-06;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742945;
        step->P[1][0] = 7.955742945;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.2811083488;
        step->Xe[1] = 29.84888581;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -6.245004514e-17;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742945;
        step->P[1][0] = 7.955742945;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.2992854928;
        step->Xe[1] = 29.84887774;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.3174581432;
        step->Xe[1] = 29.84887774;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.3356262972;
        step->Xe[1] = 29.84888583;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.3537899517;
        step->Xe[1] = 29.84890198;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046324e-06;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.3386367183;
        step->Xe[1] = 29.84891881;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.3234872258;
        step->Xe[1] = 29.8489289;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742946;
        step->P[1][0] = 7.955742946;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.3083414767;
        step->Xe[1] = 29.84893226;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046324e-06;
        step->ell[1][0] = 5.551115123e-17;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742945;
        step->P[1][0] = 7.955742945;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.2931994734;
        step->Xe[1] = 29.84892889;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046317e-06;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 216.4885195;
        step->P[0][1] = 7.955742945;
        step->P[1][0] = 7.955742945;
        step->P[1][1] = 250.3434407;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.2780612185;
        step->Xe[1] = 29.84891878;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046317e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = -3.552713679e-15;

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
        step->P[0][0] = 5.79500941e+14;
        step->P[0][1] = 1.243188932e+14;
        step->P[1][0] = 1.243188932e+14;
        step->P[1][1] = 5.010958954e+14;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.3113871082;
        step->Xe[1] = 29.84890197;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 4.857225733e-17;
        step->ell[0][1] = 2.515752228e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046318e-06;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 5.795005929e+14;
        step->P[0][1] = 1.243189103e+14;
        step->P[1][0] = 1.243189103e+14;
        step->P[1][1] = 5.010961028e+14;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.3295567521;
        step->Xe[1] = 29.84890736;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046324e-06;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 5.795003141e+14;
        step->P[0][1] = 1.243189236e+14;
        step->P[1][0] = 1.243189236e+14;
        step->P[1][1] = 5.010962684e+14;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.3144095121;
        step->Xe[1] = 29.84891341;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046317e-06;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 5.795001047e+14;
        step->P[0][1] = 1.243189331e+14;
        step->P[1][0] = 1.243189331e+14;
        step->P[1][1] = 5.010963925e+14;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.2992660169;
        step->Xe[1] = 29.84891274;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046317e-06;
        step->ell[1][0] = 5.551115123e-17;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 5.794999648e+14;
        step->P[0][1] = 1.243189391e+14;
        step->P[1][0] = 1.243189391e+14;
        step->P[1][1] = 5.010964751e+14;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.284126269;
        step->Xe[1] = 29.84890533;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = 2.515752228e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046318e-06;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 5.794998946e+14;
        step->P[0][1] = 1.243189416e+14;
        step->P[1][0] = 1.243189416e+14;
        step->P[1][1] = 5.010965162e+14;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.3023026564;
        step->Xe[1] = 29.8488986;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -6.245004514e-17;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 5.794998941e+14;
        step->P[0][1] = 1.243189408e+14;
        step->P[1][0] = 1.243189408e+14;
        step->P[1][1] = 5.010965159e+14;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.3204745498;
        step->Xe[1] = 29.84889995;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 5.794999634e+14;
        step->P[0][1] = 1.243189369e+14;
        step->P[1][0] = 1.243189369e+14;
        step->P[1][1] = 5.010964743e+14;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.3053295607;
        step->Xe[1] = 29.84890197;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = 2.515752228e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046318e-06;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 5.795001026e+14;
        step->P[0][1] = 1.243189301e+14;
        step->P[1][0] = 1.243189301e+14;
        step->P[1][1] = 5.010963913e+14;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.3235007034;
        step->Xe[1] = 29.84890466;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 5.795003119e+14;
        step->P[0][1] = 1.243189204e+14;
        step->P[1][0] = 1.243189204e+14;
        step->P[1][1] = 5.010962672e+14;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.3083549632;
        step->Xe[1] = 29.84890803;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 5.551115123e-17;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 5.795005914e+14;
        step->P[0][1] = 1.243189081e+14;
        step->P[1][0] = 1.243189081e+14;
        step->P[1][1] = 5.010961019e+14;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.2932129688;
        step->Xe[1] = 29.84890466;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -4.58967513e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046325e-06;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 11;
        limitCycle.rho = 4.664508027e-20;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.3537899517;
        step->Xe[1] = 29.84890198;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.3386367183;
        step->Xe[1] = 29.84891881;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.3234872258;
        step->Xe[1] = 29.8489289;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046317e-06;
        step->ell[1][0] = -5.551115123e-17;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.3083414767;
        step->Xe[1] = 29.84893226;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046324e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.2931994734;
        step->Xe[1] = 29.84892889;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.2780612185;
        step->Xe[1] = 29.84891878;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.2629267143;
        step->Xe[1] = 29.84890196;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 4.857225733e-17;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.2811083488;
        step->Xe[1] = 29.84888581;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.2992854928;
        step->Xe[1] = 29.84887774;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.3174581432;
        step->Xe[1] = 29.84887774;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = 2.515752228e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046318e-06;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 2.840144385e+10;
        step->P[0][1] = 6748289177;
        step->P[1][0] = 6748289177;
        step->P[1][1] = 3.175641255e+10;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.3356262972;
        step->Xe[1] = 29.84888583;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-18;
        step->ell[0][1] = -1.036961451e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046322e-06;

    }

}
