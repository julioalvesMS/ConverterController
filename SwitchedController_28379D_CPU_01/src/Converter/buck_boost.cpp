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
            P[0][0] = 0.001492673187;
            P[0][1] = 0.0008091722998;
            P[1][0] = 0.0008091722998;
            P[1][1] = 0.003023426152;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Rule 2
            //
            P[0][0] = 0.02310283858;
            P[0][1] = 0.0011663968;
            P[1][0] = 0.0011663968;
            P[1][1] = 0.03461006052;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            P[0][0] = 1.270076656e-07;
            P[0][1] = 1.068723768e-07;
            P[1][0] = 1.068723768e-07;
            P[1][1] = 5.531812659e-07;
            break;

        default:
            break;
        }
    }



    void BuckBoost::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.00283;
        num[1] = -0.0028144;

        den[0] = 1;
        den[1] = -1;
    }



    void BuckBoost::GetClassicVoltageCurrentController(double vNum[2], double vDen[2], double iNum[2], double iDen[2])
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



    void BuckBoost::GetStateFeedbackH2Controller(double K[2], double C[2], double* M)
    {
        K[0] = 1.200606059;
        K[1] = 0.6518776693;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.6671040097;
    }



    void BuckBoost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9847;

        den[0] = 1;
        den[1] = -1;
    }



    void BuckBoost::GetCurrentCorrectionController(double num[2], double den[2])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Continuous
            //
            num[0] = 0.5;
            num[1] = -0.475;

            den[0] = 1;
            den[1] = -1;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete
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
        subSys->B[0] = 0;
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
        subSys->E[0][0] = 0.49;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 10.33057851;
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
        subSys->L[0][2] = 0;
        subSys->L[1][0] = 0.01107593247;
        subSys->L[1][1] = -0.0001847376467;
        subSys->L[1][2] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->E[0][0] = 0.49;
        subSys->E[0][1] = 0;
        subSys->E[1][0] = 0;
        subSys->E[1][1] = 10.33057851;
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

        limitCycle.kappa = 9;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 59.65661508;
        step->P[0][1] = 25.15150976;
        step->P[1][0] = 25.15150976;
        step->P[1][1] = 101.6557497;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.1755756559;
        step->Xe[1] = 80.43547454;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.842203364;
        step->ell[1][1] = -0.007571932192;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 59.90288104;
        step->P[0][1] = 25.31041691;
        step->P[1][0] = 25.31041691;
        step->P[1][1] = 101.3690907;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.6558494197;
        step->Xe[1] = 80.42624234;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842145273;
        step->ell[1][1] = 0.001637521682;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 60.15221139;
        step->P[0][1] = 25.47032806;
        step->P[1][0] = 25.47032806;
        step->P[1][1] = 101.0823659;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.482149038;
        step->Xe[1] = 80.41701121;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.842086837;
        step->ell[1][1] = 0.01079020626;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 60.40464426;
        step->P[0][1] = 25.63124955;
        step->P[1][0] = 25.63124955;
        step->P[1][1] = 100.7955753;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.303354795;
        step->Xe[1] = 80.40778114;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842028059;
        step->ell[1][1] = 0.01988647151;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 60.66021826;
        step->P[0][1] = 25.79318776;
        step->P[1][0] = 25.79318776;
        step->P[1][1] = 100.5087188;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 3.119498093;
        step->Xe[1] = 80.39855213;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84196894;
        step->ell[1][1] = 0.02892666523;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 60.91897247;
        step->P[0][1] = 25.95614911;
        step->P[1][0] = 25.95614911;
        step->P[1][1] = 100.2217964;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 3.930610141;
        step->Xe[1] = 80.38932417;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.841909484;
        step->ell[0][1] = -0.03791113308;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 60.6026394;
        step->P[0][1] = 25.76884317;
        step->P[1][0] = 25.76884317;
        step->P[1][1] = 100.5876605;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.894812471;
        step->Xe[1] = 80.41800841;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.842198012;
        step->ell[0][1] = -0.02643670128;
        step->ell[1][0] = 6.661338148e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 60.28666108;
        step->P[0][1] = 25.5722992;
        step->P[1][0] = 25.5722992;
        step->P[1][1] = 100.9487635;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.865111619;
        step->Xe[1] = 80.43521492;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.842342576;
        step->ell[0][1] = -0.01503060041;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 59.97124847;
        step->P[0][1] = 25.36651902;
        step->P[1][0] = 25.36651902;
        step->P[1][1] = 101.3048714;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.8416139634;
        step->Xe[1] = 80.44101336;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.842344061;
        step->ell[0][1] = -0.003694003842;
        step->ell[1][0] = -8.326672685e-17;
        step->ell[1][1] = -1.421085472e-14;

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
        step->P[0][0] = 1.505184535e+30;
        step->P[0][1] = 6.535212966e+29;
        step->P[1][0] = 6.535212966e+29;
        step->P[1][1] = 1.954350683e+30;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 2.488340046;
        step->Xe[1] = 80.42728019;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.842286271;
        step->ell[0][1] = -0.0219339915;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.497357216e+30;
        step->P[0][1] = 6.52027666e+29;
        step->P[1][0] = 6.52027666e+29;
        step->P[1][1] = 1.963498539e+30;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.461056702;
        step->Xe[1] = 80.43998293;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842374347;
        step->ell[1][1] = 0.01055498187;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.503974879e+30;
        step->P[0][1] = 6.533349746e+29;
        step->P[1][0] = 6.533349746e+29;
        step->P[1][1] = 1.956169192e+30;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.282392486;
        step->Xe[1] = 80.43075022;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.842315545;
        step->ell[0][1] = -0.01965268747;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.496028923e+30;
        step->P[0][1] = 6.517666068e+29;
        step->P[1][0] = 6.517666068e+29;
        step->P[1][1] = 1.965309465e+30;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.256349466;
        step->Xe[1] = 80.44117126;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.842375004;
        step->ell[1][1] = 0.008287575211;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.502521702e+30;
        step->P[0][1] = 6.530346052e+29;
        step->P[1][0] = 6.530346052e+29;
        step->P[1][1] = 1.95797787e+30;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.078947202;
        step->Xe[1] = 80.43193841;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842316288;
        step->ell[1][1] = 0.01739925812;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.509009409e+30;
        step->P[0][1] = 6.543245197e+29;
        step->P[1][0] = 6.543245197e+29;
        step->P[1][1] = 1.950678767e+30;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.896473898;
        step->Xe[1] = 80.42270663;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.842257231;
        step->ell[0][1] = -0.02645477445;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.50090858e+30;
        step->P[0][1] = 6.528612371e+29;
        step->P[1][0] = 6.528612371e+29;
        step->P[1][1] = 1.959875068e+30;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.866703584;
        step->Xe[1] = 80.43993067;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.842402011;
        step->ell[0][1] = -0.015047903;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.49272087e+30;
        step->P[0][1] = 6.511298513e+29;
        step->P[1][0] = 6.511298513e+29;
        step->P[1][1] = 1.968994979e+30;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.8431366803;
        step->Xe[1] = 80.44574587;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842403702;
        step->ell[1][1] = 0.003710538265;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.49895701e+30;
        step->P[0][1] = 6.523145977e+29;
        step->P[1][0] = 6.523145977e+29;
        step->P[1][1] = 1.961656401e+30;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.668281735;
        step->Xe[1] = 80.4365125;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842345157;
        step->ell[1][1] = 0.01285043513;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 9;
        limitCycle.rho = 4.543968568e-27;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.180314233e+26;
        step->P[0][1] = 4.275382929e+25;
        step->P[1][0] = 4.275382929e+25;
        step->P[1][1] = 1.277016978e+26;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.666345182;
        step->Xe[1] = 80.43678409;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.842348439;
        step->ell[0][1] = -0.01282896701;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.175012267e+26;
        step->P[0][1] = 4.278250551e+25;
        step->P[1][0] = 4.278250551e+25;
        step->P[1][1] = 1.282987972e+26;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.6440669934;
        step->Xe[1] = 80.44038072;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.84232231;
        step->ell[1][1] = 0.001506031203;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.179211832e+26;
        step->P[0][1] = 4.274683111e+25;
        step->P[1][0] = 4.274683111e+25;
        step->P[1][1] = 1.278160164e+26;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.470439246;
        step->Xe[1] = 80.43114796;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842263859;
        step->ell[1][1] = 0.01065952039;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.18343361e+26;
        step->P[0][1] = 4.271426655e+25;
        step->P[1][0] = 4.271426655e+25;
        step->P[1][1] = 1.273365402e+26;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.29171719;
        step->Xe[1] = 80.42191627;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842205065;
        step->ell[1][1] = 0.01975658529;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.187677955e+26;
        step->P[0][1] = 4.26848075e+25;
        step->P[1][0] = 4.26848075e+25;
        step->P[1][1] = 1.268603083e+26;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 3.107932231;
        step->Xe[1] = 80.41268563;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.842145931;
        step->ell[0][1] = -0.02879757373;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.182427829e+26;
        step->P[0][1] = 4.273757281e+25;
        step->P[1][0] = 4.273757281e+25;
        step->P[1][1] = 1.274617007e+26;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.076969647;
        step->Xe[1] = 80.43225363;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.842320115;
        step->ell[1][1] = 0.0173773328;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.186666635e+26;
        step->P[0][1] = 4.270732166e+25;
        step->P[1][0] = 4.270732166e+25;
        step->P[1][1] = 1.269846184e+26;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 2.894508534;
        step->Xe[1] = 80.42302181;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.842261059;
        step->ell[0][1] = -0.02643298416;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.181406349e+26;
        step->P[0][1] = 4.275663591e+25;
        step->P[1][0] = 4.275663591e+25;
        step->P[1][1] = 1.275856896e+26;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.864746509;
        step->Xe[1] = 80.44022403;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.842405565;
        step->ell[0][1] = -0.01502620604;
        step->ell[1][0] = -4.440892099e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.176109016e+26;
        step->P[0][1] = 4.278874099e+25;
        step->P[1][0] = 4.278874099e+25;
        step->P[1][1] = 1.281836933e+26;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.8411881155;
        step->Xe[1] = 80.4460175;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.842406983;
        step->ell[1][1] = 0.00368893709;

    }

}
