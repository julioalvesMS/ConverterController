#include <src/Core/SwitchingRule/rule1.h>

using namespace SwitchedSystem;

namespace SwitchingRule1
{
    static double csi[SYSTEM_ORDER];  // (x - xe)

    static double prod_01[SYSTEM_ORDER], prod_02[SYSTEM_ORDER];    // Ai*xe, Bi*u
    static double sum_0[SYSTEM_ORDER], sum_1[SYSTEM_ORDER];    // Ai*xe + Bi*u

    static double prod_11[SYSTEM_ORDER], prod_12[SYSTEM_ORDER];   // csi'*P

    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

//        matrix_diff(X, Xe, &csi);
        csi[0]  = X[0] - Xe[0];
        csi[1]  = X[1] - Xe[1];

        for (i=0;i<SUBSYSTEMS_COUNT;i++)
        {
            new_sigma = EvaluateSubSystem(&(sys->subSystems[i]), P, X, Xe, u);

            if (new_sigma < best_sigma || i==0)
            {
                best_id = i;
                best_sigma = new_sigma;
            }
        }

        return best_id;
    }

    double EvaluateSubSystem(SubSystem *subSys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u)
    {
        double val;

        //
        // Ai*X
        //
        prod_01[0]  = subSys->A[0][0] * X[0];
        prod_01[0] += subSys->A[0][1] * X[1];
        prod_01[1]  = subSys->A[1][0] * X[0];
        prod_01[1] += subSys->A[1][1] * X[1];

        //
        // Bi*u
        //
        prod_02[0]  = subSys->B[0] * u;
        prod_02[1]  = subSys->B[1] * u;

        //
        // Ai*x + Bi*u
        //
        sum_0[0]  = prod_01[0] + prod_02[0];
        sum_0[1]  = prod_01[1] + prod_02[1];

        //
        // 2P*(Ai*x + Bi*u)
        //
        prod_11[0]  = 2 * P[0][0] * sum_0[0];
        prod_11[0] += 2 * P[0][1] * sum_0[1];
        prod_11[1]  = 2 * P[1][0] * sum_0[0];
        prod_11[1] += 2 * P[1][1] * sum_0[1];

        //
        // Q*csi
        //
        prod_12[0]  = subSys->Q[0][0] * csi[0];
        prod_12[0] += subSys->Q[0][1] * csi[1];
        prod_12[1]  = subSys->Q[1][0] * csi[0];
        prod_12[1] += subSys->Q[1][1] * csi[1];

        //
        // (Q*csi + 2P*(Ai*x + Bi*u))
        //
        sum_1[0]  = prod_11[0] + prod_12[0];
        sum_1[1]  = prod_11[1] + prod_12[1];

        //
        // csi'*P * (Ai*xe + Bi*u)
        //
        val  = csi[0] * sum_1[0];
        val += csi[1] * sum_1[1];

        return val;
    }


}

