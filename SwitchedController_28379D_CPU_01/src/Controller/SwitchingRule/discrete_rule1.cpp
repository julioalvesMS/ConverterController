#include <src/Controller/SwitchingRule/discrete_rule1.h>

using namespace SwitchedSystem;

namespace DiscreteSwitchingRule1
{
    static double csi[SYSTEM_ORDER];  // (x - xe)

    static double prod_1[SYSTEM_ORDER+1];    // [1 Xipp'] * [d sys.h'; sys.h sys.P]

    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER], double d, double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

        csi[0]  = X[0] - Xe[0];
        csi[1]  = X[1] - Xe[1];

        for (i=0;i<sys->N;i++)
        {
            new_sigma = EvaluateSubSystem(&(sys->subSystems[i]), P, h, d, X, Xe, u);

            if (new_sigma < best_sigma || i==0)
            {
                best_id = i;
                best_sigma = new_sigma;
            }
        }

        return best_id;
    }

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double h[SYSTEM_ORDER], double d, double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u)
    {
        double val;

        double Xipp[SYSTEM_ORDER];

        Xipp[0]  = subSys->A[0][0] * csi[0]  +  subSys->L[0]*u;
        Xipp[0] += subSys->A[0][1] * csi[1]  +  subSys->L[1]*u;
        Xipp[1]  = subSys->A[1][0] * csi[0]  +  subSys->L[0]*u;
        Xipp[1] += subSys->A[1][1] * csi[1]  +  subSys->L[1]*u;

        //
        // Ai*X
        //
        prod_1[0]  = d;
        prod_1[0] += Xipp[0] * h[0];
        prod_1[0] += Xipp[1] * h[1];
        prod_1[1]  = h[0];
        prod_1[1] += Xipp[0] * P[0][0];
        prod_1[1] += Xipp[1] * P[1][0];
        prod_1[2]  = h[1];
        prod_1[2] += Xipp[0] * P[0][1];
        prod_1[2] += Xipp[1] * P[1][1];

        //
        // // [1 Xipp'] * [d sys.h'; sys.h sys.P] * [1; Xipp]
        //
        val  = prod_1[0];
        val += prod_1[1] * Xipp[0];
        val += prod_1[2] * Xipp[1];

        return val;
    }


}

