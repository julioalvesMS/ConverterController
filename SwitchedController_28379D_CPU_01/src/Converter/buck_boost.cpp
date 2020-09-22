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
            P[0][0] = 0.001492235547;
            P[0][1] = 0.0008170792702;
            P[1][0] = 0.0008170792702;
            P[1][1] = 0.003050191756;
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
            P[0][0] = 0.8371672588;
            P[0][1] = 0.419950784;
            P[1][0] = 0.419950784;
            P[1][1] = 1.355279634;
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
        K[0] = 1.219076922;
        K[1] = 0.6619065564;

        C[0] = 1.8;
        C[1] = 1;

        (*M) = 0.6773671482;
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
            num[0] = 1.5;
            num[1] = -1.4;

            den[0] = 1;
            den[1] = -1;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete
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
        step->Xe[0] = -0.1729154186;
        step->Xe[1] = 79.21675523;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814291192;
        step->ell[1][1] = -0.007457205947;

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
        step->Xe[0] = 0.6459123073;
        step->Xe[1] = 79.20766292;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814233981;
        step->ell[1][1] = 0.001612710748;

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
        step->Xe[0] = 1.459692234;
        step->Xe[1] = 79.19857165;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.81417643;
        step->ell[1][1] = 0.01062671829;

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
        step->Xe[0] = 2.26845548;
        step->Xe[1] = 79.18948143;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -6.661338148e-16;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -1.814118543;
        step->ell[1][1] = 0.01958516133;

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
        step->Xe[0] = 3.072232971;
        step->Xe[1] = 79.18039225;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 2.220446049e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.81406032;
        step->ell[1][1] = 0.02848838242;

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
        step->Xe[0] = 3.871055442;
        step->Xe[1] = 79.17130411;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.814001764;
        step->ell[0][1] = -0.03733672197;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.421085472e-14;

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
        step->Xe[0] = 2.850951676;
        step->Xe[1] = 79.19955374;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.814285921;
        step->ell[0][1] = -0.0260361452;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 1.421085472e-14;

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
        step->Xe[0] = 1.836852352;
        step->Xe[1] = 79.21649955;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.814428295;
        step->ell[0][1] = -0.01480286404;
        step->ell[1][0] = -1.110223025e-16;
        step->ell[1][1] = 0;

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
        step->Xe[0] = 0.8288622366;
        step->Xe[1] = 79.22221013;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.814429757;
        step->ell[0][1] = -0.003638034086;
        step->ell[1][0] = 1.110223025e-16;
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
        step->P[0][0] = 1.434413768e+19;
        step->P[0][1] = 8.817462676e+18;
        step->P[1][0] = 8.817462676e+18;
        step->P[1][1] = 2.973821425e+19;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.844544471;
        step->Xe[1] = 79.21536799;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -1.814414597;
        step->ell[1][1] = 0.01488814059;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.443656799e+19;
        step->P[0][1] = 8.883638632e+18;
        step->P[1][0] = 8.883638632e+18;
        step->P[1][1] = 2.963884826e+19;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 2.650935231;
        step->Xe[1] = 79.20627584;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.814356519;
        step->ell[0][1] = -0.02382030629;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.433123155e+19;
        step->P[0][1] = 8.801388808e+18;
        step->P[1][0] = 8.801388808e+18;
        step->P[1][1] = 2.9762783e+19;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.637998343;
        step->Xe[1] = 79.22100504;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -1.81447109;
        step->ell[1][1] = 0.01260005526;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.442110709e+19;
        step->P[0][1] = 8.867680801e+18;
        step->P[1][0] = 8.867680801e+18;
        step->P[1][1] = 2.966364e+19;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 2.445662391;
        step->Xe[1] = 79.21191224;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.814413093;
        step->ell[0][1] = -0.02154632385;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.431351297e+19;
        step->P[0][1] = 8.785047381e+18;
        step->P[1][0] = 8.785047381e+18;
        step->P[1][1] = 2.978739293e+19;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.433934367;
        step->Xe[1] = 79.22436681;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.814499133;
        step->ell[0][1] = -0.01033962126;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.420510364e+19;
        step->P[0][1] = 8.69926515e+18;
        step->P[1][0] = 8.69926515e+18;
        step->P[1][1] = 2.990929701e+19;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.4283572684;
        step->Xe[1] = 79.22561325;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 4.440892099e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814444605;
        step->ell[1][1] = -0.0007981699774;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.428957376e+19;
        step->P[0][1] = 8.765199498e+18;
        step->P[1][0] = 8.765199498e+18;
        step->P[1][1] = 2.981065992e+19;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.243478349;
        step->Xe[1] = 79.21651992;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.814387123;
        step->ell[1][1] = 0.008230692238;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.437394642e+19;
        step->P[0][1] = 8.831619751e+18;
        step->P[1][0] = 8.831619751e+18;
        step->P[1][1] = 2.971197596e+19;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 2.053574481;
        step->Xe[1] = 79.20742764;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -6.661338148e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814329302;
        step->ell[1][1] = 0.01720389839;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.44582061e+19;
        step->P[0][1] = 8.898529418e+18;
        step->P[1][0] = 8.898529418e+18;
        step->P[1][1] = 2.961324512e+19;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 2.858676642;
        step->Xe[1] = 79.1983364;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 1.814271146;
        step->ell[0][1] = -0.02612179157;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 0;

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
        step->Xe[0] = 1.641097528;
        step->Xe[1] = 79.21804494;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.814434068;
        step->ell[0][1] = -0.01263458872;
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
        step->Xe[0] = 0.6343084026;
        step->Xe[1] = 79.22158707;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814408336;
        step->ell[1][1] = 0.001483212548;

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
        step->Xe[0] = 1.448159864;
        step->Xe[1] = 79.21249421;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.81435077;
        step->ell[1][1] = 0.01049801251;

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
        step->Xe[0] = 2.256994203;
        step->Xe[1] = 79.20340238;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814292867;
        step->ell[1][1] = 0.01945724309;

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
        step->Xe[0] = 3.060842348;
        step->Xe[1] = 79.19431161;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.814234629;
        step->ell[0][1] = -0.02836124686;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = 0;

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
        step->Xe[0] = 2.04550041;
        step->Xe[1] = 79.21358312;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -6.661338148e-16;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -1.814406174;
        step->ell[1][1] = 0.01711403988;

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
        step->Xe[0] = 2.850652344;
        step->Xe[1] = 79.20449117;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.814348012;
        step->ell[0][1] = -0.0260324844;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = 1.421085472e-14;

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
        step->Xe[0] = 1.836492774;
        step->Xe[1] = 79.22143275;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.814490329;
        step->ell[0][1] = -0.01479853625;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.421085472e-14;

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
        step->Xe[0] = 0.828442841;
        step->Xe[1] = 79.22713844;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.814491726;
        step->ell[1][1] = 0.003633044104;

    }

}
