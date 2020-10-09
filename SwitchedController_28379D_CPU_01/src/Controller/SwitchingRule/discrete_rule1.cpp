#include <src/Controller/SwitchingRule/discrete_rule1.h>


using namespace SwitchedSystem;


extern bool LoadEstimationEnabled;


namespace DiscreteSwitchingRule1
{
    double csi[SYSTEM_ORDER];  // (x - xe)

    double prod_1[SYSTEM_ORDER+1];    // [1 Xipp'] * [d dsys.h'; dsys.h dsys.P]

    double lambdas[SYSTEM_ORDER];

    double h[SYSTEM_ORDER];
    double d;

    int SwitchingRule(System *sys, System *dsys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u, double Rom)
    {
        double best_sigma = 0, new_sigma;
        int best_id = 0;
        int i;

        csi[0]  = X[0] - Xe[0];
        csi[1]  = X[1] - Xe[1];

        CalculateDiscreteSystem(sys, dsys, P, X, Xe_o, u, Rom);

        for (i=0;i<dsys->N;i++)
        {
            new_sigma = EvaluateSubSystem(&(dsys->subSystems[i]), P, X, Xe, u);

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

        Xipp[0]  = subSys->Ar[0][0] * csi[0];
        Xipp[0] += subSys->Ar[0][1] * csi[1]  +  subSys->l[0];
        Xipp[1]  = subSys->Ar[1][0] * csi[0];
        Xipp[1] += subSys->Ar[1][1] * csi[1]  +  subSys->l[1];

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
        // // [1 Xipp'] * [d dsys.h'; dsys.h dsys.P] * [1; Xipp]
        //
        val  = prod_1[0];
        val += prod_1[1] * Xipp[0];
        val += prod_1[2] * Xipp[1];

        return val;
    }

    void CalculateDiscreteSystem(System *sys, System *dsys, double P[SYSTEM_ORDER][SYSTEM_ORDER], double X[SYSTEM_ORDER], double Xe_o[SYSTEM_ORDER], double u, double Rom)
    {
        double Alamb[SYSTEM_ORDER][SYSTEM_ORDER] = {{0,0},{0,0}};
        double inv_A[SYSTEM_ORDER][SYSTEM_ORDER];
        double cl[SYSTEM_ORDER] = {0, 0};
        double prod_Pl[SYSTEM_ORDER], prod_hP[SYSTEM_ORDER];
        double det = 0;
        SubSystem* subSys;
        int i, j, k;

        CalculateLambdas(Xe_o, u);

        for(i=0;i<dsys->N;i++)
        {
            subSys = &(dsys->subSystems[i]);

            EvaluateModelR(&(sys->subSystems[i]), subSys, Rom);

            if (LoadEstimationEnabled)
            {
                subSys->l[0] = (subSys->Ar[0][0] - 1)*Xe_o[0] + subSys->Ar[0][1]*Xe_o[1] + subSys->B[0]*u;
                subSys->l[1] = subSys->Ar[1][0]*Xe_o[0] + (subSys->Ar[1][1] - 1)*Xe_o[1] + subSys->B[1]*u;
            }
            else
            {
                for(j=0;j<SYSTEM_ORDER;j++)
                    subSys->l[j] = subSys->L[j][0]*Xe_o[0] + subSys->L[j][1]*Xe_o[1] + subSys->L[j][2]*u;
            }


            for(j=0;j<SYSTEM_ORDER;j++)
            {
                for(k=0;k<SYSTEM_ORDER;k++)
                    Alamb[j][k] += lambdas[i]*subSys->Ar[j][k];

                prod_Pl[j] = P[j][0]*subSys->l[0] + P[j][1]*subSys->l[1];
            }

            for(j=0;j<SYSTEM_ORDER;j++)
                cl[j] += lambdas[i]*(subSys->Ar[0][j]*prod_Pl[0] + subSys->Ar[1][j]*prod_Pl[1]);
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

    void EvaluateModelR(SubSystem *subSys, SubSystem *dsubSys, double Rom)
    {
        if (LoadEstimationEnabled)
        {
            double Acr[SYSTEM_ORDER][SYSTEM_ORDER];
            double inv_AB[SYSTEM_ORDER];
            double Ard0, Ard1, det;

            // Continuous model
            Acr[0][0] = subSys->Ar0[0][0] + subSys->Ard[0][0]/Rom;
            Acr[0][1] = subSys->Ar0[0][1] + subSys->Ard[0][1]/Rom;
            Acr[1][0] = subSys->Ar0[1][0] + subSys->Ard[1][0]/Rom;
            Acr[1][1] = subSys->Ar0[1][1] + subSys->Ard[1][1]/Rom;

            // Discrete model
            Ard0 = exp(dsubSys->Ard[0][0]/Rom);
            Ard1 = exp(dsubSys->Ard[1][1]/Rom);
            dsubSys->Ar[0][0] = dsubSys->Ar0[0][0] * Ard0;
            dsubSys->Ar[0][1] = dsubSys->Ar0[0][1] * Ard1;
            dsubSys->Ar[1][0] = dsubSys->Ar0[1][0] * Ard0;
            dsubSys->Ar[1][1] = dsubSys->Ar0[1][1] * Ard1;

            // B(:,:,i) = (A(:,:,i)-I)/Ac(:,:,i)*sys.B(:,:,i);
            // B
            det = Acr[0][0]*Acr[1][1] - Acr[0][1]*Acr[1][0];

            inv_AB[0] = ( Acr[1][1]*subSys->B[0] - Acr[0][1]*subSys->B[1])/det;
            inv_AB[1] = (-Acr[1][0]*subSys->B[0] + Acr[0][0]*subSys->B[1])/det;

            dsubSys->B[0] = (dsubSys->Ar[0][0]-1)*inv_AB[0] + dsubSys->Ar[0][1]*inv_AB[1];
            dsubSys->B[1] = dsubSys->Ar[1][0]*inv_AB[0] + (dsubSys->Ar[1][1]-1)*inv_AB[1];

        }
        else
        {
            // Discrete model
            dsubSys->Ar[0][0] = dsubSys->A[0][0];
            dsubSys->Ar[0][1] = dsubSys->A[0][1];
            dsubSys->Ar[1][0] = dsubSys->A[1][0];
            dsubSys->Ar[1][1] = dsubSys->A[1][1];
        }
    }


}

