#include <src/Controller/LimitCycle/cycle_rule_hinf.h>

using namespace SwitchedSystem;

namespace LimitCycleRuleHinf
{
    static double Xi[SYSTEM_ORDER];  // (x - xe)
    static int k = 0;

    int SwitchingRule(System *sys, Cycle *cycle, double X[SYSTEM_ORDER])
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

        Xi[0]  = X[0] - cycle->cycleSteps[k].Xe[0];
        Xi[1]  = X[1] - cycle->cycleSteps[k].Xe[1];

        for (i=0;i<sys->N;i++)
        {
            new_sigma = EvaluateSubSystem(&(sys->subSystems[i]), i, cycle);

            if (new_sigma < best_sigma || i==0)
            {
                best_id = i;
                best_sigma = new_sigma;
            }
        }

        if(++k >= cycle->kappa)
            k = 0;

        return best_id;
    }

    double EvaluateSubSystem(SubSystem *subSys, int i, Cycle *cycle)
    {
        CycleStep *cycleStep, *nextStep;

        // Lcal
        double part_L_01[2][2], part_L_02[2][2], part_L_03[2][2];
        double part_L_11[2], part_L_20[2], part_L_21[2];
        double part_L_31;

        // Dcal
        double part_D_01[2], part_D_02[2], part_D_03[2];
        double part_D_11;

        double part_S_01[2];
        double part_S_02, part_S_03, part_S_04;

        double part_M_00[3][3];

        double part_F_00[3][3];
        double part_F_01[3], part_F_02[2];

        double val;

        int km;

        km = (k+1)%(cycle->kappa);

        cycleStep = &(cycle->cycleSteps[k]);
        nextStep = &(cycle->cycleSteps[km]);

        //
        // Lcal
        //
        part_L_01[0][0] = subSys->A[0][0]*nextStep->P[0][0] + subSys->A[1][0]*nextStep->P[1][0];
        part_L_01[0][1] = subSys->A[0][0]*nextStep->P[0][1] + subSys->A[1][0]*nextStep->P[1][1];
        part_L_01[1][0] = subSys->A[0][1]*nextStep->P[0][0] + subSys->A[1][1]*nextStep->P[1][0];
        part_L_01[1][1] = subSys->A[0][1]*nextStep->P[0][1] + subSys->A[1][1]*nextStep->P[1][1];

        part_L_02[0][0] = part_L_01[0][0]*subSys->A[0][0] + part_L_01[0][1]*subSys->A[1][0];
        part_L_02[0][1] = part_L_01[0][0]*subSys->A[0][1] + part_L_01[0][1]*subSys->A[1][1];
        part_L_02[1][0] = part_L_01[1][0]*subSys->A[0][0] + part_L_01[1][1]*subSys->A[1][0];
        part_L_02[1][1] = part_L_01[1][0]*subSys->A[0][1] + part_L_01[1][1]*subSys->A[1][1];

        part_L_03[0][0] = part_L_02[0][0] - cycleStep->P[0][0];
        part_L_03[0][0] = part_L_02[0][1] - cycleStep->P[0][1];
        part_L_03[0][0] = part_L_02[1][0] - cycleStep->P[1][0];
        part_L_03[0][0] = part_L_02[1][1] - cycleStep->P[1][1];


        part_L_11[0] = part_L_01[0][0]*cycleStep->ell[i][0] + part_L_01[0][1]*cycleStep->ell[i][1];
        part_L_11[1] = part_L_01[1][0]*cycleStep->ell[i][0] + part_L_01[1][1]*cycleStep->ell[i][1];


        part_L_20[0] = cycleStep->ell[i][0]*nextStep->P[0][0] + cycleStep->ell[i][1]*nextStep->P[1][0];
        part_L_20[1] = cycleStep->ell[i][0]*nextStep->P[0][1] + cycleStep->ell[i][1]*nextStep->P[1][1];

        part_L_21[0] = part_L_20[0]*subSys->A[0][0] + part_L_20[1]*subSys->A[1][0];
        part_L_21[1] = part_L_20[0]*subSys->A[0][1] + part_L_20[1]*subSys->A[1][1];


        part_L_31 = part_L_20[0]*cycleStep->ell[i][0] + part_L_20[1]*cycleStep->ell[i][1];



        //
        // Dcal
        //
        part_D_01[0] = part_L_01[0][0]*subSys->H[0] + part_L_01[0][1]*subSys->H[1];
        part_D_01[1] = part_L_01[1][0]*subSys->H[0] + part_L_01[1][1]*subSys->H[1];

        part_D_02[0] = subSys->E[0][0]*subSys->G[0] + subSys->E[0][1]*subSys->G[1];
        part_D_02[1] = subSys->E[1][0]*subSys->G[0] + subSys->E[1][1]*subSys->G[1];

        part_D_03[0] = part_D_01[0] + part_D_02[0];
        part_D_03[1] = part_D_01[1] + part_D_02[1];


        part_D_11 = part_L_20[0]*subSys->H[0] + part_L_20[1]*subSys->H[1];



        //
        // Sigmacal
        //
        part_S_01[0] = subSys->H[0]*nextStep->P[0][0] + subSys->H[1]*nextStep->P[1][0];
        part_S_01[1] = subSys->H[0]*nextStep->P[0][1] + subSys->H[1]*nextStep->P[1][1];

        part_S_02 = part_S_01[0]*subSys->H[0] + part_S_01[1]*subSys->H[1];

        part_S_03 = subSys->G[0]*subSys->G[0] + subSys->G[1]*subSys->G[1];

        part_S_04 = part_S_02 + part_S_03 - cycle->rho;

        part_M_00[0][0] = part_D_03[0]*part_D_03[0]/part_S_04;
        part_M_00[0][1] = part_D_03[0]*part_D_03[1]/part_S_04;
        part_M_00[0][2] = part_D_03[0]*part_D_11   /part_S_04;
        part_M_00[1][0] = part_D_03[1]*part_D_03[0]/part_S_04;
        part_M_00[1][1] = part_D_03[1]*part_D_03[1]/part_S_04;
        part_M_00[1][2] = part_D_03[1]*part_D_11   /part_S_04;
        part_M_00[2][0] = part_D_11   *part_D_03[0]/part_S_04;
        part_M_00[2][1] = part_D_11   *part_D_03[1]/part_S_04;
        part_M_00[2][2] = part_D_11   *part_D_11   /part_S_04;


        //
        // val
        //
        part_F_00[0][0] = part_L_03[0][0] - part_M_00[0][0];
        part_F_00[0][1] = part_L_03[0][1] - part_M_00[0][1];
        part_F_00[0][2] = part_L_11[0]    - part_M_00[0][2];
        part_F_00[1][0] = part_L_03[1][0] - part_M_00[1][0];
        part_F_00[1][1] = part_L_03[1][1] - part_M_00[1][1];
        part_F_00[1][2] = part_L_11[1]    - part_M_00[1][2];
        part_F_00[2][0] = part_L_21[0]    - part_M_00[2][0];
        part_F_00[2][1] = part_L_21[1]    - part_M_00[2][1];
        part_F_00[2][2] = part_L_31       - part_M_00[2][2];

        part_F_01[0] = Xi[0]*part_F_00[0][0] + Xi[1]*part_F_00[1][0] + part_F_00[2][0];
        part_F_01[1] = Xi[0]*part_F_00[0][1] + Xi[1]*part_F_00[1][1] + part_F_00[2][1];
        part_F_01[2] = Xi[0]*part_F_00[0][2] + Xi[1]*part_F_00[1][2] + part_F_00[2][2];

        part_F_02[0] = Xi[0]*subSys->Q[0][0] + Xi[1]*subSys->Q[1][0];
        part_F_02[1] = Xi[0]*subSys->Q[0][1] + Xi[1]*subSys->Q[1][1];

        val  = part_F_01[0]*Xi[0] + part_F_01[1]*Xi[1] + part_F_01[2];
        val -= part_F_02[0]*Xi[0] + part_F_02[1]*Xi[1];

        return val;
    }


}
