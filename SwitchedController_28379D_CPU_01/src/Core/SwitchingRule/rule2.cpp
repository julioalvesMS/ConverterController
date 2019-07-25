#include <src/Core/SwitchingRule/rule2.h>

using namespace Math;
using namespace SwitchedSystem;

namespace SwitchingRule2
{
    static Vector csi;  // (x - xe)

    static Vector prod_01, prod_02;    // Ai*xe, Bi*u
    static Vector sum_0;    // Ai*xe + Bi*u

    static Vector prod_1;   // csi'*P

    int SwitchingRule(System *sys, Matrix *P, Vector *X, Vector *Xe, double u)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

        matrix_diff(X, Xe, &csi);

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

    double EvaluateSubSystem(SubSystem *subSys, Matrix *P, Vector *X, Vector *Xe, double u)
    {
        double val;

        //
        // (x - xe)
        //
        matrix_product(&(subSys->A), Xe, &prod_01);

        //
        // Ai*xe, Bi*u
        //
        matrix_product(&(subSys->B), u, &prod_02);

        //
        // Ai*xe + Bi*u
        //
        matrix_sum(&prod_01, &prod_02, &sum_0);

        //
        // csi'*P
        //
        matrix_product(&csi, P, &prod_1);

        //
        // csi'*P * (Ai*xe + Bi*u)
        //
        val = matrix_product(&prod_1, &sum_0);

        return val;
    }


}

