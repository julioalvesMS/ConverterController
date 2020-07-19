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
            P[0][0] = 0.002002864862;
            P[0][1] = -1.584149142e-05;
            P[1][0] = -1.584149142e-05;
            P[1][1] = 0.002261064265;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.002021917996;
            P[0][1] = -6.348934187e-06;
            P[1][0] = -6.348934187e-06;
            P[1][1] = 0.002281680767;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 0.0002979570058;
            P[0][1] = 0.0001302151026;
            P[1][0] = 0.0001302151026;
            P[1][1] = 0.0007323883322;
            break;

        default:
            break;
        }
    }



    void Boost::GetH(double h[SYSTEM_ORDER])
    {
        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            h[0] = 5.106599295e-06;
            h[1] = 6.463491839e-06;
            break;

        default:
            break;
        }
    }



    double Boost::GetD(double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER])
    {
        double d = 0;

        switch(controlStrategy)
        {
        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            d = 1.140318697e-07;
            break;

        default:
            break;
        }

        return d;
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
            num[1] = -0.495;

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
        subSys->B[0] = 1/L;
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
        subSys->L[0] = 0.0004987654935;
        subSys->L[1] = -6.956138032e-06;
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
        subSys->L[0] = -0.0002659315521;
        subSys->L[1] = 3.708870416e-06;
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
        step->P[0][0] = 2003.359898;
        step->P[0][1] = -15.50508606;
        step->P[1][0] = -15.50508606;
        step->P[1][1] = 2261.645486;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.516814519;
        step->Xe[1] = 97.88514157;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.04940593022;
        step->ell[0][1] = -0.0006704783905;
        step->ell[1][0] = 1.595945598e-16;
        step->ell[1][1] = -4.58967513e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2003.364493;
        step->P[0][1] = -15.50312974;
        step->P[1][0] = -15.50312974;
        step->P[1][1] = 2261.640269;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.500345838;
        step->Xe[1] = 97.88536263;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.04940603994;
        step->ell[1][1] = 0.0006631598742;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2003.355304;
        step->P[0][1] = -15.50703726;
        step->P[1][0] = -15.50703726;
        step->P[1][1] = 2261.650705;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.53328716;
        step->Xe[1] = 97.8849132;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.0494058168;
        step->ell[0][1] = -0.0006777986676;
        step->ell[1][0] = -6.245004514e-17;
        step->ell[1][1] = 9.621179585e-15;

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
        step->P[0][0] = 5.706519803e+15;
        step->P[0][1] = 1.862717966e+15;
        step->P[1][0] = 1.862717966e+15;
        step->P[1][1] = 6.445397673e+15;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.516814519;
        step->Xe[1] = 97.88514157;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 0.04940593022;
        step->ell[0][1] = -0.0006704783905;
        step->ell[1][0] = 1.595945598e-16;
        step->ell[1][1] = -4.58967513e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 5.705968394e+15;
        step->P[0][1] = 1.862723292e+15;
        step->P[1][0] = 1.862723292e+15;
        step->P[1][1] = 6.446024797e+15;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.500345838;
        step->Xe[1] = 97.88536263;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.04940603994;
        step->ell[1][1] = 0.0006631598742;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 5.707072227e+15;
        step->P[0][1] = 1.862712622e+15;
        step->P[1][0] = 1.862712622e+15;
        step->P[1][1] = 6.444771075e+15;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.53328716;
        step->Xe[1] = 97.8849132;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.0494058168;
        step->ell[0][1] = -0.0006777986676;
        step->ell[1][0] = -6.245004514e-17;
        step->ell[1][1] = 9.621179585e-15;

    }



    void DefineLimitCycleHinf()
    {
        CycleStep* step;

        limitCycle.kappa = 3;
        limitCycle.rho = 1.889787707e-13;

        //
        // =============== Limit Cycle Step 1 ===============
        //
        step = &(limitCycle.cycleSteps[0]);

        //
        // Cycle Step 1 -- Matrix P
        //
        step->P[0][0] = 1.588785864e+30;
        step->P[0][1] = 4.803830846e+29;
        step->P[1][0] = 4.803830846e+29;
        step->P[1][1] = 1.7316694e+30;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.500345838;
        step->Xe[1] = 97.88536263;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.526556659e-16;
        step->ell[0][1] = 0;
        step->ell[1][0] = -0.04940603994;
        step->ell[1][1] = 0.0006631598742;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 1.589073107e+30;
        step->P[0][1] = 4.80363854e+29;
        step->P[1][0] = 4.80363854e+29;
        step->P[1][1] = 1.731348237e+30;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.53328716;
        step->Xe[1] = 97.8849132;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 0.0494058168;
        step->ell[0][1] = -0.0006777986676;
        step->ell[1][0] = -6.245004514e-17;
        step->ell[1][1] = -1.880052984e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 1.58892932e+30;
        step->P[0][1] = 4.803734989e+29;
        step->P[1][0] = 4.803734989e+29;
        step->P[1][1] = 1.731509056e+30;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.516814519;
        step->Xe[1] = 97.88514157;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0.04940593022;
        step->ell[0][1] = -0.0006704783905;
        step->ell[1][0] = -6.245004514e-17;
        step->ell[1][1] = -4.58967513e-15;

    }

}
