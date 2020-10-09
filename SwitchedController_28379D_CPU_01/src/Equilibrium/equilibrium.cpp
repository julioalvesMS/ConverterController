#include <src/Equilibrium/equilibrium.h>

extern bool LoadEstimationEnabled;

namespace Equilibrium
{
    static double Xe[SYSTEM_ORDER];
    static double Xe_o[SYSTEM_ORDER];

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;

        Xe_o[0] = 0;
        Xe_o[1] = 0;
    }

    double* GetEquilibrium(void)
    {
        return Xe;
    }

    double* GetOriginalEquilibrium(void)
    {
        return Xe_o;
    }

    void UpdateEquilibrium(double u, double Rom)
    {
        Xe[0] = EstimateEquilibriumCurrent(Vref, u, Rom);
        Xe[1] = Vref;
    }

    void UpdateOriginalEquilibrium(double u, double Rom)
    {
        Xe_o[0] = EstimateEquilibriumCurrent(Vref, u, Rom);
        Xe_o[1] = Vref;
    }

    double EstimateEquilibriumCurrent(double Ve, double u, double Rom)
    {
        double Ie, load, val;

        if (LoadEstimationEnabled)
            load = Rom;
        else
            load = Ro;

        switch(activeConverter)
        {
        case ID_Buck:
            Ie = Ve/load;
            break;

        case ID_Boost:
            val = pow(u,2)/(4*pow(R,2)) - pow(Ve,2)/(R*load);
            if (val < 0)
                val = 0;
            Ie = (u/(2*R) - sqrt(val) );
            break;

        case ID_BuckBoost:
            val = pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*load);
            if (val < 0)
                val = 0;
            Ie = (u/(2*R) - sqrt(val) );
            break;

        case ID_BuckBoost3:
            val = pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*load);
            if (val < 0)
                val = 0;
            Ie = (u/(2*R) - sqrt(val) );
            break;
        }

        return Ie;
    }
}
