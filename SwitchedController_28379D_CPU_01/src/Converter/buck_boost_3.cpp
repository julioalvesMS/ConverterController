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
            P[0][0] = 0.002002745073;
            P[0][1] = -1.049968312e-05;
            P[1][0] = -1.049968312e-05;
            P[1][1] = 0.002268687893;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.002021884222;
            P[0][1] = -5.954169369e-06;
            P[1][0] = -5.954169369e-06;
            P[1][1] = 0.002281800609;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 4.865844279e-05;
            P[0][1] = 1.341775847e-05;
            P[1][0] = 1.341775847e-05;
            P[1][1] = 9.296591225e-05;
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



    void BuckBoost3::GetCurrentCorrectionController(double num[2], double den[2])
    {
        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Continuous
            //
            num[0] = 0.5;
            num[1] = -0.475;

            den[0] = 1;
            den[1] = -1;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete
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

        limitCycle.kappa = 8;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 80.63159058;
        step->P[0][1] = -0.8130126526;
        step->P[1][0] = -0.8130126526;
        step->P[1][1] = 90.06689029;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.173990899;
        step->Xe[1] = 80.70203274;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.015327568;
        step->ell[0][1] = 0.01197466605;
        step->ell[1][0] = -1.845650873;
        step->ell[1][1] = 0.007357127416;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 80.62254189;
        step->P[0][1] = -0.8181530737;
        step->P[1][0] = -0.8181530737;
        step->P[1][1] = 90.07723264;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.997096348;
        step->Xe[1] = 80.69276995;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.01526851;
        step->ell[0][1] = 0.02109197445;
        step->ell[1][0] = -1.845591816;
        step->ell[1][1] = 0.01647443581;
        step->ell[2][0] = 2.220446049e-16;
        step->ell[2][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 80.61338051;
        step->P[0][1] = -0.823325995;
        step->P[1][0] = -0.823325995;
        step->P[1][1] = 90.08757737;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.815127628;
        step->Xe[1] = 80.68350823;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -1.015209112;
        step->ell[0][1] = 0.0301530816;
        step->ell[1][0] = -1.845532418;
        step->ell[1][1] = 0.02553554297;
        step->ell[2][0] = -2.220446049e-16;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 80.60410504;
        step->P[0][1] = -0.8285316217;
        step->P[1][0] = -0.8285316217;
        step->P[1][1] = 90.09792447;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 3.628116018;
        step->Xe[1] = 80.67424757;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.845472682;
        step->ell[2][1] = -0.03454079535;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 80.61321088;
        step->P[0][1] = -0.8237680884;
        step->P[1][0] = -0.8237680884;
        step->P[1][1] = 90.08739282;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 2.590619925;
        step->Xe[1] = 80.69952876;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 1.421085472e-14;
        step->ell[2][0] = 1.845718281;
        step->ell[2][1] = -0.02304779001;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 80.62232059;
        step->P[0][1] = -0.8187424991;
        step->P[1][0] = -0.8187424991;
        step->P[1][1] = 90.0769822;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.559274049;
        step->Xe[1] = 80.71331405;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.845819692;
        step->ell[2][1] = -0.01162370832;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 80.63142837;
        step->P[0][1] = -0.8134545712;
        step->P[1][0] = -0.8134545712;
        step->P[1][1] = 90.06669927;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 0.5341846646;
        step->Xe[1] = 80.71567368;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = -1.127223315e-14;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;
        step->ell[2][0] = 1.015454499;
        step->ell[2][1] = -0.004887261084;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 80.64052831;
        step->P[0][1] = -0.8079041684;
        step->P[1][0] = -0.8079041684;
        step->P[1][1] = 90.05655068;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.3457798058;
        step->Xe[1] = 80.71129659;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -1.015386282;
        step->ell[0][1] = 0.002800807813;
        step->ell[1][0] = -1.845709588;
        step->ell[1][1] = -0.001816730825;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

    }



    void DefineLimitCycleH2()
    {
        CycleStep* step;

        limitCycle.kappa = 8;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.583882106e+18;
        step->P[0][1] = 8.628160236e+17;
        step->P[1][0] = 8.628160236e+17;
        step->P[1][1] = 3.066542093e+18;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.565649746;
        step->Xe[1] = 80.8332465;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -1.110223025e-16;
        step->ell[0][1] = -1.127223315e-14;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;
        step->ell[2][0] = 1.017005573;
        step->ell[2][1] = -0.01630347327;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.574804406e+18;
        step->P[0][1] = 8.55628419e+17;
        step->P[1][0] = 8.55628419e+17;
        step->P[1][1] = 3.07730599e+18;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.369335176;
        step->Xe[1] = 80.84027213;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.017080249;
        step->ell[0][1] = 0.01412861483;
        step->ell[1][0] = -1.847403554;
        step->ell[1][1] = 0.009511076197;
        step->ell[2][0] = -2.220446049e-16;
        step->ell[2][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.58468947e+18;
        step->P[0][1] = 8.626559628e+17;
        step->P[1][0] = 8.626559628e+17;
        step->P[1][1] = 3.066400529e+18;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 2.191236393;
        step->Xe[1] = 80.83099347;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.847344214;
        step->ell[2][1] = -0.01861504771;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.575348554e+18;
        step->P[0][1] = 8.555048349e+17;
        step->P[1][0] = 8.555048349e+17;
        step->P[1][1] = 3.077196682e+18;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.16072665;
        step->Xe[1] = 80.84033093;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.017066424;
        step->ell[0][1] = 0.01181807678;
        step->ell[1][0] = -1.847389729;
        step->ell[1][1] = 0.007200538139;
        step->ell[2][0] = 0;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.584960221e+18;
        step->P[0][1] = 8.625573403e+17;
        step->P[1][0] = 8.625573403e+17;
        step->P[1][1] = 3.066327395e+18;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.983913869;
        step->Xe[1] = 80.83105227;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.017007172;
        step->ell[0][1] = 0.02093629196;
        step->ell[1][0] = -1.847330478;
        step->ell[1][1] = 0.01631875332;
        step->ell[2][0] = -2.220446049e-16;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.594567122e+18;
        step->P[0][1] = 8.696647576e+17;
        step->P[1][0] = 8.696647576e+17;
        step->P[1][1] = 3.055453153e+18;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.802026414;
        step->Xe[1] = 80.82177468;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.847270886;
        step->ell[2][1] = -0.02538076168;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.584778839e+18;
        step->P[0][1] = 8.628345545e+17;
        step->P[1][0] = 8.628345545e+17;
        step->P[1][1] = 3.066461968e+18;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.767824682;
        step->Xe[1] = 80.83787891;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.421085472e-14;
        step->ell[2][0] = 1.84740127;
        step->ell[2][1] = -0.01392488649;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.574859359e+18;
        step->P[0][1] = 8.557234269e+17;
        step->P[1][0] = 8.557234269e+17;
        step->P[1][1] = 3.077336998e+18;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 0.7398680744;
        step->Xe[1] = 80.84252542;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -1.017064646;
        step->ell[0][1] = 0.007156522089;
        step->ell[1][0] = -1.847387952;
        step->ell[1][1] = 0.002538983452;
        step->ell[2][0] = -1.110223025e-16;
        step->ell[2][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 8;
        limitCycle.rho = 8.194020469e-28;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.230453949e+30;
        step->P[0][1] = 4.336866267e+29;
        step->P[1][0] = 4.336866267e+29;
        step->P[1][1] = 1.32734625e+30;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.98198897;
        step->Xe[1] = 80.9140853;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.848374894;
        step->ell[2][1] = -0.01629162428;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.225621353e+30;
        step->P[0][1] = 4.340377282e+29;
        step->P[1][0] = 4.340377282e+29;
        step->P[1][1] = 1.332794634e+30;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 0.9517384877;
        step->Xe[1] = 80.9210898;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -1.018067773;
        step->ell[0][1] = 0.009497688113;
        step->ell[1][0] = -1.848391079;
        step->ell[1][1] = 0.004880149476;
        step->ell[2][0] = -2.220446049e-16;
        step->ell[2][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.230437739e+30;
        step->P[0][1] = 4.33608195e+29;
        step->P[1][0] = 4.33608195e+29;
        step->P[1][1] = 1.327305438e+30;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.776214049;
        step->Xe[1] = 80.91180187;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 4.440892099e-16;
        step->ell[1][1] = -1.421085472e-14;
        step->ell[2][0] = 1.848331802;
        step->ell[2][1] = -0.01401263489;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.22555663e+30;
        step->P[0][1] = 4.339263576e+29;
        step->P[1][0] = 4.339263576e+29;
        step->P[1][1] = 1.33275593e+30;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 0.7472751919;
        step->Xe[1] = 80.91652764;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -1.017996106;
        step->ell[0][1] = 0.007233385625;
        step->ell[1][0] = -1.848319412;
        step->ell[1][1] = 0.002615846988;
        step->ell[2][0] = 3.330669074e-16;
        step->ell[2][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.230346333e+30;
        step->P[0][1] = 4.335326973e+29;
        step->P[1][0] = 4.335326973e+29;
        step->P[1][1] = 1.327291791e+30;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.573011201;
        step->Xe[1] = 80.90724023;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -1.017936923;
        step->ell[0][1] = 0.01637983164;
        step->ell[1][0] = -1.848260229;
        step->ell[1][1] = 0.011762293;
        step->ell[2][0] = -2.220446049e-16;
        step->ell[2][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.235177948e+30;
        step->P[0][1] = 4.331531292e+29;
        step->P[1][0] = 4.331531292e+29;
        step->P[1][1] = 1.321833405e+30;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 2.393656823;
        step->Xe[1] = 80.89795389;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.8303233059;
        step->ell[0][1] = 0.004617538638;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;
        step->ell[2][0] = 1.848200704;
        step->ell[2][1] = -0.02085235817;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.23039372e+30;
        step->P[0][1] = 4.336043483e+29;
        step->P[1][0] = 4.336043483e+29;
        step->P[1][1] = 1.327315764e+30;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.361042736;
        step->Xe[1] = 80.90952097;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -1.017950815;
        step->ell[0][1] = 0.01403192368;
        step->ell[1][0] = -1.848274121;
        step->ell[1][1] = 0.00941438504;
        step->ell[2][0] = -4.440892099e-16;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.235211761e+30;
        step->P[0][1] = 4.331979527e+29;
        step->P[1][0] = 4.331979527e+29;
        step->P[1][1] = 1.32187242e+30;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 2.182995073;
        step->Xe[1] = 80.90023437;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 1.110223025e-16;
        step->ell[0][1] = 2.938621568e-15;
        step->ell[1][0] = -0.8303233059;
        step->ell[1][1] = -0.004617538638;
        step->ell[2][0] = 1.017891378;
        step->ell[2][1] = -0.02313646195;

    }

}
