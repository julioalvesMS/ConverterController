#include <src/Controller/SwitchingRule/rule2.h>

using namespace SwitchedSystem;

namespace SwitchingRule2
{
    static double csi[SYSTEM_ORDER];  // (x - xe)

    static double prod_01[SYSTEM_ORDER], prod_02[SYSTEM_ORDER];    // Ai*xe, Bi*u
    static double sum_0[SYSTEM_ORDER];    // Ai*xe + Bi*u

    static double prod_1[SYSTEM_ORDER];   // csi'*P

    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double u)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

//        matrix_diff(X, Xe, &csi);
        csi[0]  = X[0] - Xe[0];
        csi[1]  = X[1] - Xe[1];

        for (i=0;i<sys->N;i++)
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
        // (x - xe)
        //
//        matrix_product(&(subSys->A), Xe, &prod_01);
        prod_01[0]  = subSys->A[0][0] * Xe[0];
        prod_01[0] += subSys->A[0][1] * Xe[1];
        prod_01[1]  = subSys->A[1][0] * Xe[0];
        prod_01[1] += subSys->A[1][1] * Xe[1];

        //
        // Bi*u
        //
//        matrix_product(&(subSys->B), u, &prod_02);
        prod_02[0]  = subSys->B[0] * u;
        prod_02[1]  = subSys->B[1] * u;

        //
        // Ai*xe + Bi*u
        //
//        matrix_sum(&prod_01, &prod_02, &sum_0);
        sum_0[0]  = prod_01[0] + prod_02[0];
        sum_0[1]  = prod_01[1] + prod_02[1];

        //
        // csi'*P
        //
//        matrix_product(&csi, P, &prod_1);
        prod_1[0]  = csi[0] * P[0][0];
        prod_1[0] += csi[1] * P[1][0];
        prod_1[1]  = csi[0] * P[0][1];
        prod_1[1] += csi[1] * P[1][1];

        //
        // csi'*P * (Ai*xe + Bi*u)
        //
//        val = matrix_product(&prod_1, &sum_0);
        val  = prod_1[0] * sum_0[0];
        val += prod_1[1] * sum_0[1];

        return val;
    }


}

