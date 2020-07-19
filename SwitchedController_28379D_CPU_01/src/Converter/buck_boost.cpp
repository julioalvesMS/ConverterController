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
            P[0][0] = 0.01857634665;
            P[0][1] = 0.02195172849;
            P[1][0] = 0.02195172849;
            P[1][1] = 0.04900915211;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // BuckBoost Converter - Rule 2
            //
            P[0][0] = 0.9841523422;
            P[0][1] = 0.0002825838797;
            P[1][0] = 0.0002825838797;
            P[1][1] = 1.12500585;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            P[0][0] = 0.001279605259;
            P[0][1] = 0.0008147720704;
            P[1][0] = 0.0008147720704;
            P[1][1] = 0.004249247422;
            break;

        default:
            break;
        }
    }



    void BuckBoost::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            h[0] = 4.995030366e-05;
            h[1] = 9.368135598e-05;
            break;

        default:
            break;
        }
    }



    double BuckBoost::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // BuckBoost Converter - Discrete Rule 1
            //
            d = 2.976171145e-06;
            break;

        default:
            break;
        }

        return d;
    }



    void BuckBoost::GetClassicVoltageController(double num[2], double den[2])
    {
        num[0] = 0.00483;
        num[1] = -0.00482265;

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
            num[1] = -0.495;

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
        subSys->A[0][0] = 0.9997526808;
        subSys->A[0][1] = 0;
        subSys->A[1][0] = 0;
        subSys->A[1][1] = 0.9999954086;
        //
        // Subsystem 1 -- Matrix L
        //
        subSys->L[0] = 0.0004977122204;
        subSys->L[1] = -5.56373786e-06;
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
        subSys->Q[1][1] = 10.33057851;


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
        subSys->L[0] = -0.0006186506511;
        subSys->L[1] = 6.915663125e-06;
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
        subSys->Q[1][1] = 10.33057851;

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
        step->P[0][0] = 18551.46596;
        step->P[0][1] = 22002.84134;
        step->P[1][0] = 22002.84134;
        step->P[1][1] = 49205.91566;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.78757759;
        step->Xe[1] = 80.43953687;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -0.07391299192;
        step->ell[1][1] = 0.000785356149;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 18560.15532;
        step->P[0][1] = 22008.38542;
        step->P[1][0] = 22008.38542;
        step->P[1][1] = 49196.03678;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.820447874;
        step->Xe[1] = 80.43916755;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -0.07391280919;
        step->ell[1][1] = 0.000799963365;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 18568.84898;
        step->P[0][1] = 22013.9309;
        step->P[1][0] = 22013.9309;
        step->P[1][1] = 49186.15781;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.853310029;
        step->Xe[1] = 80.43879822;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -0.07391262647;
        step->ell[1][1] = 0.0008145669684;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 18577.54695;
        step->P[0][1] = 22019.47778;
        step->P[1][0] = 22019.47778;
        step->P[1][1] = 49176.27875;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.886164057;
        step->Xe[1] = 80.4384289;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -2.91433544e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391244374;
        step->ell[1][1] = 0.0008291669601;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 18586.24921;
        step->P[0][1] = 22025.02605;
        step->P[1][0] = 22025.02605;
        step->P[1][1] = 49166.39959;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.919009959;
        step->Xe[1] = 80.43805958;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391226102;
        step->ell[1][1] = 0.0008437633409;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 18594.95578;
        step->P[0][1] = 22030.57572;
        step->P[1][0] = 22030.57572;
        step->P[1][1] = 49156.52035;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.951847737;
        step->Xe[1] = 80.43769026;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.07391207829;
        step->ell[0][1] = -0.0008583561117;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 18584.08216;
        step->P[0][1] = 22023.66121;
        step->P[1][0] = 22023.66121;
        step->P[1][1] = 49168.8795;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.910765316;
        step->Xe[1] = 80.43817929;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.07391232052;
        step->ell[0][1] = -0.0008400995036;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 18573.2093;
        step->P[0][1] = 22016.73398;
        step->P[1][0] = 22016.73398;
        step->P[1][1] = 49181.23178;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.869692814;
        step->Xe[1] = 80.43865007;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = 0.07391255353;
        step->ell[0][1] = -0.0008218473051;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 18562.33724;
        step->P[0][1] = 22009.79402;
        step->P[1][0] = 22009.79402;
        step->P[1][1] = 49193.57717;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.828630236;
        step->Xe[1] = 80.4391026;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.07391277733;
        step->ell[0][1] = -0.0008035995193;
        step->ell[1][0] = 0;
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
        step->P[0][0] = 1.851226911e+27;
        step->P[0][1] = 7.686480506e+26;
        step->P[1][0] = 7.686480506e+26;
        step->P[1][1] = 2.037588195e+27;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.894333662;
        step->Xe[1] = 80.43837715;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.07391241854;
        step->ell[0][1] = -0.0008327974439;
        step->ell[1][0] = 2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.850847953e+27;
        step->P[0][1] = 7.686643417e+26;
        step->P[1][0] = 7.686643417e+26;
        step->P[1][1] = 2.038019589e+27;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.853265125;
        step->Xe[1] = 80.43884063;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.07391264787;
        step->ell[0][1] = -0.0008145470086;
        step->ell[1][0] = 0;
        step->ell[1][1] = 1.421085472e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.850468742e+27;
        step->P[0][1] = 7.686800922e+26;
        step->P[1][0] = 7.686800922e+26;
        step->P[1][1] = 2.038450825e+27;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.812206515;
        step->Xe[1] = 80.43928585;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391286798;
        step->ell[1][1] = 0.0007963009871;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 1.850772631e+27;
        step->P[0][1] = 7.686674335e+26;
        step->P[1][0] = 7.686674335e+26;
        step->P[1][1] = 2.038105939e+27;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.845070709;
        step->Xe[1] = 80.43891653;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391268526;
        step->ell[1][1] = 0.0008109054963;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 1.851076436e+27;
        step->P[0][1] = 7.686547571e+26;
        step->P[1][0] = 7.686547571e+26;
        step->P[1][1] = 2.037761079e+27;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.877926774;
        step->Xe[1] = 80.4385472;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = 0.07391250253;
        step->ell[0][1] = -0.0008255063935;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 1.850696822e+27;
        step->P[0][1] = 7.686705451e+26;
        step->P[1][0] = 7.686705451e+26;
        step->P[1][1] = 2.038192178e+27;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.836862211;
        step->Xe[1] = 80.43900339;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391272818;
        step->ell[1][1] = 0.0008072577249;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 1.851000301e+27;
        step->P[0][1] = 7.686576934e+26;
        step->P[1][0] = 7.686576934e+26;
        step->P[1][1] = 2.037847167e+27;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.869720306;
        step->Xe[1] = 80.43863406;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391254545;
        step->ell[1][1] = 0.0008218595243;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 1.851303695e+27;
        step->P[0][1] = 7.686448241e+26;
        step->P[1][0] = 7.686448241e+26;
        step->P[1][1] = 2.037502181e+27;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.902570275;
        step->Xe[1] = 80.43826474;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391236273;
        step->ell[1][1] = 0.0008364577123;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 1.851607002e+27;
        step->P[0][1] = 7.686319372e+26;
        step->P[1][0] = 7.686319372e+26;
        step->P[1][1] = 2.037157219e+27;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.93541212;
        step->Xe[1] = 80.43789542;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.07391218;
        step->ell[0][1] = -0.00085105229;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 9;
        limitCycle.rho = 1.889486406e-27;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 3.118851162e+24;
        step->P[0][1] = 1.12878491e+24;
        step->P[1][0] = 1.12878491e+24;
        step->P[1][1] = 3.369673863e+24;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.820429185;
        step->Xe[1] = 80.43919017;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -0.07391282061;
        step->ell[1][1] = 0.0007999550574;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 3.119296988e+24;
        step->P[0][1] = 1.128750369e+24;
        step->P[1][0] = 1.128750369e+24;
        step->P[1][1] = 3.369167255e+24;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.853291345;
        step->Xe[1] = 80.43882084;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = -1.421085472e-14;
        step->ell[1][0] = -0.07391263788;
        step->ell[1][1] = 0.0008145586628;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 3.119742908e+24;
        step->P[0][1] = 1.128715958e+24;
        step->P[1][0] = 1.128715958e+24;
        step->P[1][1] = 3.368660785e+24;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.886145377;
        step->Xe[1] = 80.43845152;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -0.07391245516;
        step->ell[1][1] = 0.0008291586566;

        //
        // =============== Limit Cycle Step 4 ===============
        //
        step = &(limitCycle.cycleSteps[3]);

        //
        // Cycle Step 4 -- Matrix P
        //
        step->P[0][0] = 3.120188922e+24;
        step->P[0][1] = 1.128681679e+24;
        step->P[1][0] = 1.128681679e+24;
        step->P[1][1] = 3.368154453e+24;
        //
        // Cycle Step 4 -- Vector Xe
        //
        step->Xe[0] = 1.918991284;
        step->Xe[1] = 80.4380822;
        //
        // Cycle Step 4 -- Matrix ell
        //
        step->ell[0][0] = 0.07391227243;
        step->ell[0][1] = -0.0008437550394;
        step->ell[1][0] = 0;
        step->ell[1][1] = -1.421085472e-14;

        //
        // =============== Limit Cycle Step 5 ===============
        //
        step = &(limitCycle.cycleSteps[4]);

        //
        // Cycle Step 5 -- Matrix P
        //
        step->P[0][0] = 3.119631633e+24;
        step->P[0][1] = 1.128725201e+24;
        step->P[1][0] = 1.128725201e+24;
        step->P[1][1] = 3.368787632e+24;
        //
        // Cycle Step 5 -- Vector Xe
        //
        step->Xe[0] = 1.877916795;
        step->Xe[1] = 80.43855663;
        //
        // Cycle Step 5 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391250729;
        step->ell[1][1] = 0.0008255019578;

        //
        // =============== Limit Cycle Step 6 ===============
        //
        step = &(limitCycle.cycleSteps[5]);

        //
        // Cycle Step 6 -- Matrix P
        //
        step->P[0][0] = 3.120077623e+24;
        step->P[0][1] = 1.128690888e+24;
        step->P[1][0] = 1.128690888e+24;
        step->P[1][1] = 3.368281264e+24;
        //
        // Cycle Step 6 -- Vector Xe
        //
        step->Xe[0] = 1.910764737;
        step->Xe[1] = 80.43818731;
        //
        // Cycle Step 6 -- Matrix ell
        //
        step->ell[0][0] = 0.07391232457;
        step->ell[0][1] = -0.000840099245;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 7 ===============
        //
        step = &(limitCycle.cycleSteps[6]);

        //
        // Cycle Step 7 -- Matrix P
        //
        step->P[0][0] = 3.119520301e+24;
        step->P[0][1] = 1.128734265e+24;
        step->P[1][0] = 1.128734265e+24;
        step->P[1][1] = 3.368914417e+24;
        //
        // Cycle Step 7 -- Vector Xe
        //
        step->Xe[0] = 1.86969223;
        step->Xe[1] = 80.43865809;
        //
        // Cycle Step 7 -- Matrix ell
        //
        step->ell[0][0] = 0.07391255758;
        step->ell[0][1] = -0.0008218470448;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 0;

        //
        // =============== Limit Cycle Step 8 ===============
        //
        step = &(limitCycle.cycleSteps[7]);

        //
        // Cycle Step 8 -- Matrix P
        //
        step->P[0][0] = 3.118962823e+24;
        step->P[0][1] = 1.128776915e+24;
        step->P[1][0] = 1.128776915e+24;
        step->P[1][1] = 3.369547441e+24;
        //
        // Cycle Step 8 -- Vector Xe
        //
        step->Xe[0] = 1.828629648;
        step->Xe[1] = 80.43911061;
        //
        // Cycle Step 8 -- Matrix ell
        //
        step->ell[0][0] = -6.938893904e-17;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.07391278137;
        step->ell[1][1] = 0.0008035992572;

        //
        // =============== Limit Cycle Step 9 ===============
        //
        step = &(limitCycle.cycleSteps[8]);

        //
        // Cycle Step 9 -- Matrix P
        //
        step->P[0][0] = 3.119408671e+24;
        step->P[0][1] = 1.128742405e+24;
        step->P[1][0] = 1.128742405e+24;
        step->P[1][1] = 3.369040865e+24;
        //
        // Cycle Step 9 -- Vector Xe
        //
        step->Xe[0] = 1.86148978;
        step->Xe[1] = 80.43874129;
        //
        // Cycle Step 9 -- Matrix ell
        //
        step->ell[0][0] = 0.07391259865;
        step->ell[0][1] = -0.0008182019614;
        step->ell[1][0] = 0;
        step->ell[1][1] = 0;

    }

}
