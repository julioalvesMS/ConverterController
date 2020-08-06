#include <src/Controller/SwitchingRule/discrete_rule1.h>

using namespace SwitchedSystem;

namespace DiscreteSwitchingRule1
{
    double csi[SYSTEM_ORDER];  // (x - xe)

    double prod_1[SYSTEM_ORDER+1];    // [1 Xipp'] * [d sys.h'; sys.h sys.P]

    double lambdas[SYSTEM_ORDER];

    double h[SYSTEM_ORDER];
    double d;

    int SwitchingRule(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

        csi[0]  = X[0] - Xe[0];
        csi[1]  = X[1] - Xe[1];

        CalculateDiscreteSystem(sys, P, X, Xe_o, u);

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

        double Xipp[SYSTEM_ORDER];

        Xipp[0]  = subSys->A[0][0] * csi[0];
        Xipp[0] += subSys->A[0][1] * csi[1]  +  subSys->l[0];
        Xipp[1]  = subSys->A[1][0] * csi[0];
        Xipp[1] += subSys->A[1][1] * csi[1]  +  subSys->l[1];

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

    void CalculateDiscreteSystem(System *sys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u)
    {
        double Alamb[SYSTEM_ORDER][SYSTEM_ORDER] = {{0,0},{0,0}};
        double inv_A[SYSTEM_ORDER][SYSTEM_ORDER];
        double cl[SYSTEM_ORDER] = {0, 0};
        double prod_Pl[SYSTEM_ORDER], prod_hP[SYSTEM_ORDER];
        double det = 0;
        SubSystem* subSys;
        int i, j, k;

        CalculateLambdas(Xe_o, u);

        for(i=0;i<sys->N;i++)
        {
            subSys = &(sys->subSystems[i]);

            for(j=0;j<SYSTEM_ORDER;j++)
                subSys->l[j] = subSys->L[j][0]*Xe_o[0] + subSys->L[j][1]*Xe_o[1] + subSys->L[j][2]*u;


            for(j=0;j<SYSTEM_ORDER;j++)
            {
                for(k=0;k<SYSTEM_ORDER;k++)
                    Alamb[j][k] += lambdas[i]*subSys->A[j][k];

                prod_Pl[j] = P[j][0]*subSys->l[0] + P[j][1]*subSys->l[1];
            }

            for(j=0;j<SYSTEM_ORDER;j++)
                cl[j] += lambdas[i]*(subSys->A[0][j]*prod_Pl[0] + subSys->A[1][j]*prod_Pl[1]);
        }

        det = (1-Alamb[0][0])*(1-Alamb[1][1]) - Alamb[0][1]*Alamb[1][0];

        inv_A[0][0] = (1-Alamb[1][1])/det;
        inv_A[0][1] = Alamb[1][0]/det;
        inv_A[1][0] = Alamb[0][1]/det;
        inv_A[1][1] = (1-Alamb[0][0])/det;

        h[0] = inv_A[0][0]*cl[0] + inv_A[0][1]*cl[1];
        h[1] = inv_A[1][0]*cl[0] + inv_A[1][1]*cl[1];

        det = P[0][0]*P[1][1]-P[0][1]*P[1][0];

        prod_hP[0] = (h[0]*P[1][1] - h[1]*P[1][0])/det;
        prod_hP[1] = (-h[0]*P[0][1] + h[1]*P[0][0])/det;

        d = prod_hP[0]*h[0] + prod_hP[1]*h[1];
    }

    void CalculateLambdas(double Xe[SYSTEM_ORDER], double u)
    {
        double lamb;

        switch(activeConverter)
        {
        case ID_Buck:
            lamb = (R*Xe[0]+Xe[1])/(u);

            lambdas[0] = lamb;
            lambdas[1] = 1-lamb;
            break;

        case ID_Boost:
            lamb = (u-R*Xe[0])/(Xe[1]);

            lambdas[0] = 1-lamb;
            lambdas[1] = lamb;
            break;

        case ID_BuckBoost:
            lamb = (Xe[1]+R*Xe[0])/(u+Xe[1]);

            lambdas[0] = lamb;
            lambdas[1] = 1-lamb;
            break;

        case ID_BuckBoost3:
            lamb = (Xe[1]+R*Xe[0])/(u+Xe[1]);

            lambdas[0] = 0;
            lambdas[1] = 1-lamb;
            lambdas[1] = lamb;
            break;
        }
    }


}

