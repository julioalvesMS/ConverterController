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
            P[0][0] = 0.003683307197;
            P[0][1] = 0.002764889635;
            P[1][0] = 0.002764889635;
            P[1][1] = 0.00710659678;
            break;

        case CS_CONTINUOUS_THEOREM_2:
            //
            // Boost Converter - Rule 2
            //
            P[0][0] = 0.1146233299;
            P[0][1] = 0.005978730139;
            P[1][0] = 0.005978730139;
            P[1][1] = 0.1731460734;
            break;

        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter - Discrete Rule 1
            //
            P[0][0] = 1.786124042;
            P[0][1] = 0.8527711217;
            P[1][0] = 0.8527711217;
            P[1][1] = 2.714213794;
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



    void Boost::GetStateFeedbackH2Controller(double K[2], double* M)
    {
        K[0] = 0.6079054123;
        K[1] = 0.2956295741;

        (*M) = 0.3154097501;
    }



    void Boost::GetReferenceController(double num[2], double den[2])
    {
        num[0] = 1;
        num[1] = -0.9847;

        den[0] = 1;
        den[1] = -1;
    }



    void Boost::GetCurrentCorrectionController(double num[2], double den[2], double *designVoltage)
    {
        (*designVoltage) = 110;

        switch(controlStrategy)
        {
        case CS_CONTINUOUS_THEOREM_1:
        case CS_CONTINUOUS_THEOREM_2:
        case CS_DISCRETE_THEOREM_1:
            //
            // Boost Converter
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
        // Subsystem 1 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = -247.3498233;
        subSys->Ar0[0][1] = 0;
        subSys->Ar0[1][0] = 0;
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
        subSys->Q[1][1] = 1.549586777;


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
        subSys->B[0] = 504.7955578;
        subSys->B[1] = 0;
        //
        // Subsystem 2 -- Matrix Q
        //
        subSys->Q[0][0] = 0.49;
        subSys->Q[0][1] = 0;
        subSys->Q[1][0] = 0;
        subSys->Q[1][1] = 1.549586777;

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
        // Subsystem 1 -- Matrix Ar0
        //
        subSys->Ar0[0][0] = 0.9938353344;
        subSys->Ar0[0][1] = 0;
        subSys->Ar0[1][0] = 0;
        subSys->Ar0[1][1] = 1;
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
        subSys->Q[1][1] = 1.549586777;


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
        subSys->Q[1][1] = 1.549586777;

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
        step->P[0][0] = 146.8965083;
        step->P[0][1] = 89.07266183;
        step->P[1][0] = 89.07266183;
        step->P[1][1] = 241.3476757;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.900157366;
        step->Xe[1] = 96.39470541;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.21279075;
        step->ell[0][1] = -0.01884981427;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 3.540570614e-15;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 146.239699;
        step->P[0][1] = 88.80221544;
        step->P[1][0] = 88.80221544;
        step->P[1][1] = 242.0982171;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.493414541;
        step->Xe[1] = 96.40249126;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.212860298;
        step->ell[0][1] = -0.01434421351;
        step->ell[1][0] = 3.330669074e-16;
        step->ell[1][1] = -1.06702841e-14;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 145.5809682;
        step->P[0][1] = 88.51340483;
        step->P[1][0] = 88.51340483;
        step->P[1][1] = 242.8418722;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.089109602;
        step->Xe[1] = 96.40577063;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.212873324;
        step->ell[1][1] = 0.009865929884;

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
        step->P[0][0] = 2.493070291e+10;
        step->P[0][1] = 8219851488;
        step->P[1][0] = 8219851488;
        step->P[1][1] = 3.070742875e+10;
        //
        // Cycle Step 1 -- Vector Xe
        //
        step->Xe[0] = 1.493414541;
        step->Xe[1] = 96.40249126;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.212860298;
        step->ell[0][1] = -0.01434421351;
        step->ell[1][0] = 1.110223025e-16;
        step->ell[1][1] = -1.06702841e-14;

        //
        // =============== Limit Cycle Step 2 ===============
        //
        step = &(limitCycle.cycleSteps[1]);

        //
        // Cycle Step 2 -- Matrix P
        //
        step->P[0][0] = 2.487068951e+10;
        step->P[0][1] = 8210441381;
        step->P[1][0] = 8210441381;
        step->P[1][1] = 3.077638651e+10;
        //
        // Cycle Step 2 -- Vector Xe
        //
        step->Xe[0] = 1.089109602;
        step->Xe[1] = 96.40577063;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 4.440892099e-16;
        step->ell[0][1] = 1.421085472e-14;
        step->ell[1][0] = -1.212873324;
        step->ell[1][1] = 0.009865929884;

        //
        // =============== Limit Cycle Step 3 ===============
        //
        step = &(limitCycle.cycleSteps[2]);

        //
        // Cycle Step 3 -- Matrix P
        //
        step->P[0][0] = 2.499246671e+10;
        step->P[0][1] = 8228153403;
        step->P[1][0] = 8228153403;
        step->P[1][1] = 3.063807903e+10;
        //
        // Cycle Step 3 -- Vector Xe
        //
        step->Xe[0] = 1.900157366;
        step->Xe[1] = 96.39470541;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 1.21279075;
        step->ell[0][1] = -0.01884981427;
        step->ell[1][0] = 0;
        step->ell[1][1] = 3.540570614e-15;

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
        step->Xe[0] = 1.900157366;
        step->Xe[1] = 96.39470541;
        //
        // Cycle Step 1 -- Matrix ell
        //
        step->ell[0][0] = 1.21279075;
        step->ell[0][1] = -0.01884981427;
        step->ell[1][0] = -2.220446049e-16;
        step->ell[1][1] = 3.540570614e-15;

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
        step->Xe[0] = 1.493414541;
        step->Xe[1] = 96.40249126;
        //
        // Cycle Step 2 -- Matrix ell
        //
        step->ell[0][0] = 1.212860298;
        step->ell[0][1] = -0.01434421351;
        step->ell[1][0] = 3.330669074e-16;
        step->ell[1][1] = -1.06702841e-14;

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
        step->Xe[0] = 1.089109602;
        step->Xe[1] = 96.40577063;
        //
        // Cycle Step 3 -- Matrix ell
        //
        step->ell[0][0] = 0;
        step->ell[0][1] = 0;
        step->ell[1][0] = -1.212873324;
        step->ell[1][1] = 0.009865929884;

    }

}
