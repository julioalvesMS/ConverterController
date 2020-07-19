#include <src/Equilibrium/current_correction.h>

namespace CurrentCorrection
{

    static double c[2] = {0, 0};
    static double e[2] = {0, 0};

    static double numPI[2] = {0, 0};
    static double denPI[2] = {0, 0};

    void Configure(void)
    {
        LoadController();
        ResetController();
    }

    void LoadController(void)
    {
        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            Buck::GetCurrentCorrectionController(numPI, denPI);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetCurrentCorrectionController(numPI, denPI);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetCurrentCorrectionController(numPI, denPI);
            break;
        case BaseConverter::ID_BuckBoost3:
            BuckBoost3::GetCurrentCorrectionController(numPI, denPI);
            break;

        default:
            break;
        }
    }

    void UpdateReference(double Vout, double u)
    {
        double Ie;

        Ie = Equilibrium::EstimateEquilibriumCurrent(Vref, u);

        e[1] = e[0];
        c[1] = c[0];

        e[0] = Vref - Vout;
        c[0] = numPI[0]*e[0] + numPI[1]*e[1] - denPI[1]*c[1];

        Xe[0] = Ie + c[0];
        Xe[1] = Vref;
    }

    void ResetController(void)
    {
        c[0] = 0;
        c[1] = 0;

        e[0] = 0;
        e[1] = 0;
    }
}
