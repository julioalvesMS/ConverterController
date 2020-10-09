#include <src/Equilibrium/current_correction.h>

namespace CurrentCorrection
{

    static double c[2] = {0, 0};
    static double e[2] = {0, 0};

    static double numPI[2] = {0, 0};
    static double denPI[2] = {0, 0};

    static double designVoltage = 100;

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
            Buck::GetCurrentCorrectionController(numPI, denPI, &designVoltage);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetCurrentCorrectionController(numPI, denPI, &designVoltage);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetCurrentCorrectionController(numPI, denPI, &designVoltage);
            break;
        case BaseConverter::ID_BuckBoost3:
            BuckBoost3::GetCurrentCorrectionController(numPI, denPI, &designVoltage);
            break;

        default:
            break;
        }
    }

    void UpdateReference(double Vout, double u, double Rom, bool enable)
    {
        double Ie, Iec;

        Ie = Equilibrium::EstimateEquilibriumCurrent(Vref, u, Rom);

        if (enable)
        {
            e[1] = e[0];
            c[1] = c[0];

            e[0] = Vref - Vout;
            c[0] = numPI[0]*e[0] + numPI[1]*e[1] - denPI[1]*c[1];
        }

        Iec = ModulateCorrection(Ie, u, Rom);

        Xe[0] = Ie + Iec;
        Xe[1] = Vref;
    }

    double ModulateCorrection(double Ie, double u, double Rom)
    {
        double Ie100, gain, Iec;

        Ie100 = Equilibrium::EstimateEquilibriumCurrent(designVoltage, u, Rom);
        gain = Ie/Ie100;

        if (gain < 0.1)
            gain = 0.1;
        else if (gain > 1)
            gain = 1;

        Iec = gain*c[0];

        if (Iec > 8)
            Iec = 8;
        else if (Iec < -8)
            Iec = -8;

        return Iec;

    }

    void ResetController(void)
    {
        c[0] = 0;
        c[1] = 0;

        e[0] = 0;
        e[1] = 0;
    }
}
