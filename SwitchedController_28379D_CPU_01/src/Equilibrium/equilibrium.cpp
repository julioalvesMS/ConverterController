#include <src/Equilibrium/equilibrium.h>

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

    void UpdateEquilibrium(double u)
    {
        Xe[0] = EstimateEquilibriumCurrent(Vref, u);
        Xe[1] = Vref;
    }

    void UpdateOriginalEquilibrium(double u)
    {
        Xe_o[0] = EstimateEquilibriumCurrent(Vref, u);
        Xe_o[1] = Vref;
    }

    double EstimateEquilibriumCurrent(double Ve, double u)
    {
        double Ie;

        switch(activeConverter)
        {
        case ID_Buck:
            Ie = Ve/Ro;
            break;

        case ID_Boost:
            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - pow(Ve,2)/(R*Ro) ) );
            break;

        case ID_BuckBoost:
            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*Ro) ) );
            break;

        case ID_BuckBoost3:
            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*Ro) ) );
            break;
        }

        return Ie;
    }
}
