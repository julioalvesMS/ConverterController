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
            P[0][0] = 0.0002164385836;
            P[0][1] = 7.95577254e-06;
            P[1][0] = 7.95577254e-06;
            P[1][1] = 0.0002503384835;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Buck Converter - Rule 2
            //
            P[0][0] = 0.000216438814;
            P[0][1] = 7.955830817e-06;
            P[1][0] = 7.955830817e-06;
            P[1][1] = 0.0002503387422;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Buck Converter - Discrete Rule 1
            //
            P[0][0] = 5.044744814e-06;
            P[0][1] = 1.356239421e-06;
            P[1][0] = 1.356239421e-06;
            P[1][1] = 9.58204939e-06;
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
            num[1] = -0.475;

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
        subSys->A[0][0] = -247.3498233;
        subSys->A[0][1] = -504.7955578;
        subSys->A[1][0] = 444.4444444;
        subSys->A[1][1] = -4.591368228;
        //
        // Subsystem 1 -- Matrix B
        //
        subSys->B[0] = 504.7955578;
        subSys->B[1] = 0;
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
        subSys->Q[0][0] = 0.1;
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
        step->P[0][0] = 8.707719041;
        step->P[0][1] = 0.3183435623;
        step->P[1][0] = 0.3183435623;
        step->P[1][1] = 10.01877227;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = -0.3665434232;
        step->Xe[1] = 29.83655133;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 8.707719051;
        step->P[0][1] = 0.3183435827;
        step->P[1][0] = 0.3183435827;
        step->P[1][1] = 10.01877232;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.09072326343;
        step->Xe[1] = 29.83159712;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -3.330669074e-16;
        step->ell[0][1] = -4.166805789e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 8.707719058;
        step->P[0][1] = 0.3183435988;
        step->P[1][0] = 0.3183435988;
        step->P[1][1] = 10.01877235;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.5452014517;
        step->Xe[1] = 29.83170849;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 8.707719064;
        step->P[0][1] = 0.3183436106;
        step->P[1][0] = 0.3183436106;
        step->P[1][1] = 10.01877238;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.996844802;
        step->Xe[1] = 29.8368536;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -3.330669074e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 8.707719067;
        step->P[0][1] = 0.3183436182;
        step->P[1][0] = 0.3183436182;
        step->P[1][1] = 10.01877239;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.445607664;
        step->Xe[1] = 29.84700013;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 8.707719068;
        step->P[0][1] = 0.3183436217;
        step->P[1][0] = 0.3183436217;
        step->P[1][1] = 10.0187724;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.061121772;
        step->Xe[1] = 29.85749772;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 8.707719067;
        step->P[0][1] = 0.3183436212;
        step->P[1][0] = 0.3183436212;
        step->P[1][1] = 10.0187724;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.6789008922;
        step->Xe[1] = 29.86373483;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 8.707719065;
        step->P[0][1] = 0.3183436168;
        step->P[1][0] = 0.3183436168;
        step->P[1][1] = 10.0187724;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.2989845003;
        step->Xe[1] = 29.86573733;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -2.081668171e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 8.707719061;
        step->P[0][1] = 0.3183436086;
        step->P[1][0] = 0.3183436086;
        step->P[1][1] = 10.01877238;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = -0.07858850033;
        step->Xe[1] = 29.86353154;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 8.707719056;
        step->P[0][1] = 0.3183435967;
        step->P[1][0] = 0.3183435967;
        step->P[1][1] = 10.01877235;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = -0.4537797793;
        step->Xe[1] = 29.85714418;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.110223025e-15;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 8.707719049;
        step->P[0][1] = 0.3183435812;
        step->P[1][0] = 0.3183435812;
        step->P[1][1] = 10.01877232;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = -0.8265515815;
        step->Xe[1] = 29.84660241;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

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
        step->P[0][0] = 1.387908262e+16;
        step->P[0][1] = 3.509093591e+15;
        step->P[1][0] = 3.509093591e+15;
        step->P[1][1] = 1.592036476e+16;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 0.00348690739;
        step->Xe[1] = 29.85218998;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 5.551115123e-17;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.388218057e+16;
        step->P[0][1] = 3.510272418e+15;
        step->P[1][0] = 3.510272418e+15;
        step->P[1][1] = 1.59226567e+16;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = -0.3720733932;
        step->Xe[1] = 29.84671377;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.388467012e+16;
        step->P[0][1] = 3.51123892e+15;
        step->P[1][0] = 3.51123892e+15;
        step->P[1][1] = 1.592452454e+16;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.08509992702;
        step->Xe[1] = 29.84169644;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.388654715e+16;
        step->P[0][1] = 3.511985159e+15;
        step->P[1][0] = 3.511985159e+15;
        step->P[1][1] = 1.59259574e+16;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.5394861251;
        step->Xe[1] = 29.84174365;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -2.775557562e-17;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.388780772e+16;
        step->P[0][1] = 3.512503247e+15;
        step->P[1][0] = 3.512503247e+15;
        step->P[1][1] = 1.592694414e+16;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 0.1607155599;
        step->Xe[1] = 29.84220607;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.38884481e+16;
        step->P[0][1] = 3.512785356e+15;
        step->P[1][0] = 3.512785356e+15;
        step->P[1][1] = 1.592747332e+16;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 0.6146239224;
        step->Xe[1] = 29.84309071;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = -4.166805789e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.388846474e+16;
        step->P[0][1] = 3.512823727e+15;
        step->P[1][0] = 3.512823727e+15;
        step->P[1][1] = 1.592753324e+16;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.065691272;
        step->Xe[1] = 29.84900264;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.388785433e+16;
        step->P[0][1] = 3.512610672e+15;
        step->P[1][0] = 3.512610672e+15;
        step->P[1][1] = 1.59271119e+16;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.6835487712;
        step->Xe[1] = 29.85529193;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -1.665334537e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.388661373e+16;
        step->P[0][1] = 3.512138591e+15;
        step->P[1][0] = 3.512138591e+15;
        step->P[1][1] = 1.592619705e+16;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.3037096133;
        step->Xe[1] = 29.85734747;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 5.551115123e-17;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 1.388474003e+16;
        step->P[0][1] = 3.511399973e+15;
        step->P[1][0] = 3.511399973e+15;
        step->P[1][1] = 1.592477615e+16;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = -0.07378730201;
        step->Xe[1] = 29.85519556;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-15;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 1.38822305e+16;
        step->P[0][1] = 3.510387407e+15;
        step->P[1][0] = 3.510387407e+15;
        step->P[1][1] = 1.592283641e+16;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.381419658;
        step->Xe[1] = 29.85348046;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -1.864827737e-17;
        step->ell[1][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 11;
        limitCycle.rho = 1.041841321e-17;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.445607664;
        step->Xe[1] = 29.84700013;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.061121772;
        step->Xe[1] = 29.85749772;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 0.6789008922;
        step->Xe[1] = 29.86373483;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.665334537e-16;
        step->ell[1][1] = -3.552713679e-15;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.2989845003;
        step->Xe[1] = 29.86573733;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -2.775557562e-17;
        step->ell[1][1] = 3.552713679e-15;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = -0.07858850033;
        step->Xe[1] = 29.86353154;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.665334537e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = -0.4537797793;
        step->Xe[1] = 29.85714418;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = -0.8265515815;
        step->Xe[1] = 29.84660241;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = -0.3665434232;
        step->Xe[1] = 29.83655133;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -2.220446049e-16;
        step->ell[0][1] = -4.166805789e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 0.09072326343;
        step->Xe[1] = 29.83159712;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 3.330669074e-16;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 10 ===============
        //
        step = &(limitCycle.cycleSteps[9]);

        //
        // Cycle Step 10 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 10 -- Vector Xe
        //
        step->Xe[0] = 0.5452014517;
        step->Xe[1] = 29.83170849;
        //
        // Cycle Step 10 -- Matrix ell
        //
        step->ell[0][0] = 4.440892099e-16;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

        //
        // =============== Limit Cycle Step 11 ===============
        //
        step = &(limitCycle.cycleSteps[10]);

        //
        // Cycle Step 11 -- Matrix P
        //
        step->P[0][0] = 9.750744769e+17;
        step->P[0][1] = 2.31513849e+17;
        step->P[1][0] = 2.31513849e+17;
        step->P[1][1] = 1.090152626e+18;
        //
        // Cycle Step 11 -- Vector Xe
        //
        step->Xe[0] = 0.996844802;
        step->Xe[1] = 29.8368536;
        //
        // Cycle Step 11 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = -6.140921105e-16;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;

    }

}
