#include <src/Core/Equilibrium/reference_update.h>

namespace Equilibrium
{

    static double Xe[SYSTEM_ORDER];

    static double pid_sum = 0;

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;
    }

    void UpdateReference(double Vref, double Vout, double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

        double erro = Vref - Vout;

        pid_sum += erro*REFERENCE_CONTROLLER_PERIOD/PERIOD_UNIT;

        Ve = pid_kp*erro + pid_ki*pid_sum;

        //
        // Limits for Buck
        //
        upperLimit = u; lowerLimit = 0;

        if (Ve < lowerLimit)
        {
            outsideLimit = lowerLimit - Ve;
            Ve = lowerLimit;
            pid_sum += outsideLimit/pid_ki;
        }
        else if (Ve > upperLimit)
        {
            outsideLimit = upperLimit - Ve;
            Ve = upperLimit;
            pid_sum += outsideLimit/pid_ki;
        }


        Ie = Ve/Ro;

        Xe[0] = Ie;
        Xe[1] = Ve;
    }

    double* GetReference()
    {
        return Xe;
    }
}
