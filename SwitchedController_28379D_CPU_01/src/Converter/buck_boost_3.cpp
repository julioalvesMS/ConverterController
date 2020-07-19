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
            P[0][0] = 0.002002917413;
            P[0][1] = -1.717987664e-05;
            P[1][0] = -1.717987664e-05;
            P[1][1] = 0.002258652463;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost3 Converter - Rule 2
            //
            P[0][0] = 0.002021917803;
            P[0][1] = -6.350268894e-06;
            P[1][0] = -6.350268894e-06;
            P[1][1] = 0.002281676229;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost3 Converter - Discrete Rule 1
            //
            P[0][0] = 0.0001207486682;
            P[0][1] = 5.672538615e-05;
            P[1][0] = 5.672538615e-05;
            P[1][1] = 0.000314844971;
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
            h[0] = 2.582126728e-06;
            h[1] = 3.557148362e-06;
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
            d = 7.428341708e-08;
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
            num[1] = -0.495;

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
        subSys->A[0][0] = 0.9997525686;
        subSys->A[0][1] = -0.0005047319548;
        subSys->A[1][0] = 0.0004443884455;
        subSys->A[1][1] = 0.9999952965;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = -0.000164484711;
        subSys->L[1] = 4.061589333e-06;
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
        subSys->A[0][0] = 0.9997525686;
        subSys->A[0][1] = -0.0005047319548;
        subSys->A[1][0] = 0.0004443884455;
        subSys->A[1][1] = 0.9999952965;
        //
        // Subsystem 2 -- Matrix L
        //
        subSys->L[0] = -0.0006692178246;
        subSys->L[1] = 3.949421964e-06;
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
        subSys->A[0][0] = 0.9997526808;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.9999954086;
        //
        // Subsystem 3 -- Matrix L
        //
        subSys->L[0] = 0.0004990936233;
        subSys->L[1] = -6.036300315e-06;
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
        step->P[0][0] = 2003.52183;
        step->P[0][1] = -20.49407117;
        step->P[1][0] = -20.49407117;
        step->P[1][1] = 2251.776581;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.666421581;
        step->Xe[1] = 80.85669805;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046335e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 1.421085472e-14;
        step->ell[2][0] = 0.0741235329;
        step->ell[2][1] = -0.0007314690266;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2003.530938;
        step->P[0][1] = -20.48890132;
        step->P[1][0] = -20.48890132;
        step->P[1][1] = 2251.766238;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.625198296;
        step->Xe[1] = 80.85705828;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -6.245004514e-17;
        step->ell[0][1] = -1.880052984e-14;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.40304634e-06;
        step->ell[2][0] = 0.04081132461;
        step->ell[2][1] = -0.0007205528813;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2003.540046;
        step->P[0][1] = -20.48372098;
        step->P[1][0] = -20.48372098;
        step->P[1][1] = 2251.7559;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.617297416;
        step->Xe[1] = 80.85740759;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -0.04081150003;
        step->ell[0][1] = 0.000717041782;
        step->ell[1][0] = -0.07412388552;
        step->ell[1][1] = 0.0007096387357;
        step->ell[2][0] = 1.526556659e-16;
        step->ell[2][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 2003.530945;
        step->P[0][1] = -20.48888363;
        step->P[1][0] = -20.48888363;
        step->P[1][1] = 2251.766246;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.650209814;
        step->Xe[1] = 80.85703634;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -0.04081131634;
        step->ell[0][1] = 0.000731667713;
        step->ell[1][0] = -0.07412370183;
        step->ell[1][1] = 0.0007242646667;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 2003.521838;
        step->P[0][1] = -20.49404757;
        step->P[1][0] = -20.49404757;
        step->P[1][1] = 2251.776591;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.683114072;
        step->Xe[1] = 80.8566651;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -0.04081113265;
        step->ell[0][1] = 0.0007462900268;
        step->ell[1][0] = -0.07412351814;
        step->ell[1][1] = 0.0007388869805;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 2003.512728;
        step->P[0][1] = -20.49921282;
        step->P[1][0] = -20.49921282;
        step->P[1][1] = 2251.786936;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.716010192;
        step->Xe[1] = 80.85629386;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -0.04081094896;
        step->ell[0][1] = 0.0007609087242;
        step->ell[1][0] = -0.07412333446;
        step->ell[1][1] = 0.0007535056778;
        step->ell[2][0] = 1.526556659e-16;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 2003.503612;
        step->P[0][1] = -20.50437936;
        step->P[1][0] = -20.50437936;
        step->P[1][1] = 2251.797282;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.748898176;
        step->Xe[1] = 80.85592262;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;
        step->ell[2][0] = 0.07412315077;
        step->ell[2][1] = -0.0007681207597;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 2003.512721;
        step->P[0][1] = -20.49923051;
        step->P[1][0] = -20.49923051;
        step->P[1][1] = 2251.786929;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.707654876;
        step->Xe[1] = 80.8563195;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;
        step->ell[2][0] = 0.07412334646;
        step->ell[2][1] = -0.0007497926692;

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
        step->P[0][0] = 7.018229991e+15;
        step->P[0][1] = 2.272785686e+15;
        step->P[1][0] = 2.272785686e+15;
        step->P[1][1] = 4.630377828e+15;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.633771693;
        step->Xe[1] = 80.86415523;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -0.04081490763;
        step->ell[0][1] = 0.0007243620035;
        step->ell[1][0] = -0.07412729312;
        step->ell[1][1] = 0.0007169589572;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 7.019241986e+15;
        step->P[0][1] = 2.272044918e+15;
        step->P[1][0] = 2.272044918e+15;
        step->P[1][1] = 4.629232874e+15;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.666680016;
        step->Xe[1] = 80.86378395;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 0.07412710942;
        step->ell[2][1] = -0.0007315830776;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 7.018233423e+15;
        step->P[0][1] = 2.272788797e+15;
        step->P[1][0] = 2.272788797e+15;
        step->P[1][1] = 4.630381459e+15;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.625453092;
        step->Xe[1] = 80.86414426;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -0.04081490116;
        step->ell[0][1] = 0.0007206653145;
        step->ell[1][0] = -0.07412728665;
        step->ell[1][1] = 0.0007132622682;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 7.019244154e+15;
        step->P[0][1] = 2.272046653e+15;
        step->P[1][0] = 2.272046653e+15;
        step->P[1][1] = 4.629235157e+15;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.658363472;
        step->Xe[1] = 80.86377299;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -0.04081471745;
        step->ell[0][1] = 0.0007352903492;
        step->ell[1][0] = -0.07412710295;
        step->ell[1][1] = 0.0007278873029;
        step->ell[2][0] = 1.526556659e-16;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 7.020254166e+15;
        step->P[0][1] = 2.271303991e+15;
        step->P[1][0] = 2.271303991e+15;
        step->P[1][1] = 4.628088834e+15;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.691265714;
        step->Xe[1] = 80.86340171;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -0.04081453375;
        step->ell[0][1] = 0.0007499117668;
        step->ell[1][0] = -0.07412691924;
        step->ell[1][1] = 0.0007425087205;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 7.021263459e+15;
        step->P[0][1] = 2.27056081e+15;
        step->P[1][0] = 2.27056081e+15;
        step->P[1][1] = 4.626942489e+15;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.724159818;
        step->Xe[1] = 80.86303044;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;
        step->ell[2][0] = 0.07412673554;
        step->ell[2][1] = -0.000757126522;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 7.020253512e+15;
        step->P[0][1] = 2.271304315e+15;
        step->P[1][0] = 2.271304315e+15;
        step->P[1][1] = 4.628088183e+15;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.682919052;
        step->Xe[1] = 80.86341629;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -6.245004514e-17;
        step->ell[0][1] = 9.621179585e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046311e-06;
        step->ell[2][0] = 0.04081454017;
        step->ell[2][1] = -0.0007462026048;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 7.019242355e+15;
        step->P[0][1] = 2.27204594e+15;
        step->P[1][0] = 2.27204594e+15;
        step->P[1][1] = 4.629233297e+15;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.67500068;
        step->Xe[1] = 80.86379122;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046306e-06;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = -1.421085472e-14;
        step->ell[2][0] = 0.07412711402;
        step->ell[2][1] = -0.0007352806836;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 8;
        limitCycle.rho = 3.592047531e-30;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 3.230530809e+27;
        step->P[0][1] = 1.128399294e+27;
        step->P[1][0] = 1.128399294e+27;
        step->P[1][1] = 3.509379259e+27;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.748898176;
        step->Xe[1] = 80.85592262;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046306e-06;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = -1.421085472e-14;
        step->ell[2][0] = 0.07412315077;
        step->ell[2][1] = -0.0007681207598;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 3.230029448e+27;
        step->P[0][1] = 1.128435253e+27;
        step->P[1][0] = 1.128435253e+27;
        step->P[1][1] = 3.509949089e+27;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.707654876;
        step->Xe[1] = 80.8563195;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046321e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;
        step->ell[2][0] = 0.07412334646;
        step->ell[2][1] = -0.0007497926692;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 3.229527987e+27;
        step->P[0][1] = 1.128470574e+27;
        step->P[1][0] = 1.128470574e+27;
        step->P[1][1] = 3.510518766e+27;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.666421581;
        step->Xe[1] = 80.85669805;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.03331238549;
        step->ell[0][1] = 7.403046335e-06;
        step->ell[1][0] = 0;
        step->ell[1][1] = 1.421085472e-14;
        step->ell[2][0] = 0.0741235329;
        step->ell[2][1] = -0.0007314690266;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 3.229026396e+27;
        step->P[0][1] = 1.128505257e+27;
        step->P[1][0] = 1.128505257e+27;
        step->P[1][1] = 3.511088323e+27;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.625198296;
        step->Xe[1] = 80.85705828;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 1.595945598e-16;
        step->ell[0][1] = -4.58967513e-15;
        step->ell[1][0] = -0.03331238549;
        step->ell[1][1] = -7.403046325e-06;
        step->ell[2][0] = 0.04081132461;
        step->ell[2][1] = -0.0007205528813;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 3.228524862e+27;
        step->P[0][1] = 1.128539477e+27;
        step->P[1][0] = 1.128539477e+27;
        step->P[1][1] = 3.511657927e+27;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.617297416;
        step->Xe[1] = 80.85740759;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -0.04081150003;
        step->ell[0][1] = 0.000717041782;
        step->ell[1][0] = -0.07412388552;
        step->ell[1][1] = 0.0007096387357;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 3.229026172e+27;
        step->P[0][1] = 1.1285042e+27;
        step->P[1][0] = 1.1285042e+27;
        step->P[1][1] = 3.511087973e+27;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.650209814;
        step->Xe[1] = 80.85703634;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = -0.04081131634;
        step->ell[0][1] = 0.000731667713;
        step->ell[1][0] = -0.07412370183;
        step->ell[1][1] = 0.0007242646667;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 3.229527619e+27;
        step->P[0][1] = 1.128469113e+27;
        step->P[1][0] = 1.128469113e+27;
        step->P[1][1] = 3.510518276e+27;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.683114072;
        step->Xe[1] = 80.8566651;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -0.04081113265;
        step->ell[0][1] = 0.0007462900268;
        step->ell[1][0] = -0.07412351814;
        step->ell[1][1] = 0.0007388869805;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 3.230029179e+27;
        step->P[0][1] = 1.128434169e+27;
        step->P[1][0] = 1.128434169e+27;
        step->P[1][1] = 3.509948743e+27;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.716010192;
        step->Xe[1] = 80.85629386;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -0.04081094896;
        step->ell[0][1] = 0.0007609087242;
        step->ell[1][0] = -0.07412333446;
        step->ell[1][1] = 0.0007535056778;
        step->ell[2][0] = -6.938893904e-17;
        step->ell[2][1] = 0;

    }

}
