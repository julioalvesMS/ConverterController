#include <src/Controller/LimitCycle/cycle_rule_cost.h>

using namespace SwitchedSystem;

namespace LimitCycleRuleCost
{
    static double Xi[SYSTEM_ORDER];  // (x - xe)
    static int k = 0;

    int SwitchingRule(System *sys, Cycle *cycle, double X[SYSTEM_ORDER])
    {
        CycleStep *cycleStep, *nextStep;
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i, km;

        km = (k+1)%(cycle->kappa);

        cycleStep = &(cycle->cycleSteps[k]);
        nextStep = &(cycle->cycleSteps[km]);

        Xi[0]  = X[0] - cycleStep->Xe[0];
        Xi[1]  = X[1] - cycleStep->Xe[1];

        for (i=0;i<sys->N;i++)
        {
            new_sigma = EvaluateSubSystem(&(sys->subSystems[i]), i, cycleStep, nextStep);

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

    double EvaluateSubSystem(SubSystem *subSys, int i, CycleStep *cycleStep, CycleStep *nextStep)
    {
        double part_01[2][2], part_02[2][2], part_03[2][2], part_04[2][2], part_05[2][2];
        double part_11[2], part_20[2], part_21[2], finl_00[3];
        double part_31, val;

        part_01[0][0] = subSys->A[0][0]*nextStep->P[0][0] + subSys->A[1][0]*nextStep->P[1][0];
        part_01[0][1] = subSys->A[0][0]*nextStep->P[0][1] + subSys->A[1][0]*nextStep->P[1][1];
        part_01[1][0] = subSys->A[0][1]*nextStep->P[0][0] + subSys->A[1][1]*nextStep->P[1][0];
        part_01[1][1] = subSys->A[0][1]*nextStep->P[0][1] + subSys->A[1][1]*nextStep->P[1][1];

        part_02[0][0] = part_01[0][0]*subSys->A[0][0] + part_01[0][1]*subSys->A[1][0];
        part_02[0][1] = part_01[0][0]*subSys->A[0][1] + part_01[0][1]*subSys->A[1][1];
        part_02[1][0] = part_01[1][0]*subSys->A[0][0] + part_01[1][1]*subSys->A[1][0];
        part_02[1][1] = part_01[1][0]*subSys->A[0][1] + part_01[1][1]*subSys->A[1][1];

        part_03[0][0] = part_02[0][0] - cycleStep->P[0][0];
        part_03[0][0] = part_02[0][1] - cycleStep->P[0][1];
        part_03[0][0] = part_02[1][0] - cycleStep->P[1][0];
        part_03[0][0] = part_02[1][1] - cycleStep->P[1][1];

        part_04[0][0] = subSys->E[0][0]*subSys->E[0][0] + subSys->E[1][0]*subSys->E[1][0];
        part_04[0][1] = subSys->E[0][0]*subSys->E[0][1] + subSys->E[1][0]*subSys->E[1][1];
        part_04[1][0] = subSys->E[0][1]*subSys->E[0][0] + subSys->E[1][1]*subSys->E[1][0];
        part_04[1][1] = subSys->E[0][1]*subSys->E[0][1] + subSys->E[1][1]*subSys->E[1][1];

        part_05[0][0] = part_03[0][0] + part_04[0][0];
        part_05[0][1] = part_03[0][1] + part_04[0][1];
        part_05[1][0] = part_03[1][0] + part_04[1][0];
        part_05[1][1] = part_03[1][1] + part_04[1][1];



        part_11[0] = part_01[0][0]*cycleStep->ell[i][0] + part_01[0][1]*cycleStep->ell[i][1];
        part_11[1] = part_01[1][0]*cycleStep->ell[i][0] + part_01[1][1]*cycleStep->ell[i][1];



        part_20[0] = cycleStep->ell[i][0]*nextStep->P[0][0] + cycleStep->ell[i][1]*nextStep->P[1][0];
        part_20[1] = cycleStep->ell[i][0]*nextStep->P[0][1] + cycleStep->ell[i][1]*nextStep->P[1][1];

        part_21[0] = part_20[0]*subSys->A[0][0] + part_20[1]*subSys->A[1][0];
        part_21[1] = part_20[0]*subSys->A[0][1] + part_20[1]*subSys->A[1][1];



        part_31 = part_20[0]*cycleStep->ell[i][0] + part_20[1]*cycleStep->ell[i][1];


        finl_00[0] = Xi[0]*part_05[0][0] + Xi[1]*part_05[1][0] + part_21[0];
        finl_00[1] = Xi[0]*part_05[0][1] + Xi[1]*part_05[1][1] + part_21[1];
        finl_00[2] = Xi[0]*part_11[0] + Xi[1]*part_11[1] + part_31;

        val = finl_00[0]*Xi[0] + finl_00[1]*Xi[1] + finl_00[2];

        return val;
    }


}
