#include <src/Equilibrium/reference_update.h>

namespace ReferenceUpdate
{

    static double r[2] = {0, 0};
    static double e[2] = {0, 0};

    static double numPID[2] = {0, 0};
    static double denPID[2] = {0, 0};

    void Configure(void)
    {
        ResetController();
    }

    void LoadController(void)
    {
        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            Buck::GetReferenceController(numPID, denPID);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetReferenceController(numPID, denPID);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetReferenceController(numPID, denPID);
            break;
        case BaseConverter::ID_BuckBoost3:
            BuckBoost3::GetReferenceController(numPID, denPID);
            break;

        default:
            break;
        }
    }

    void UpdateReference(double Vout, double u, double Rom, bool enable)
    {
        double Ve;
        double Ie;

        double upperLimit = 50, lowerLimit = -50;

        if (enable)
        {
            e[1] = e[0];
            r[1] = r[0];

            e[0] = Vref - Vout;
            r[0] = numPID[0]*e[0] + numPID[1]*e[1] - denPID[1]*r[1];
        }

        if (r[0] < lowerLimit)
            r[0] = lowerLimit;
        else if (r[0] > upperLimit)
            r[0] = upperLimit;

        Ve = Vref + r[0];
        Ie = Equilibrium::EstimateEquilibriumCurrent(Ve, u, Rom);


        Xe[0] = Ie;
        Xe[1] = Ve;
    }

    void ResetController(void)
    {
        r[0] = 0;
        r[1] = 0;

        e[0] = 0;
        e[1] = 0;
    }
}
