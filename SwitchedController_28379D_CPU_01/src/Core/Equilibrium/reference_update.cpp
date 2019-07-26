#include <src/Core/Equilibrium/reference_update.h>

namespace Equilibrium
{

    static double Xe[SYSTEM_ORDER];

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;
    }

    void UpdateReference(double Vref, double X[SYSTEM_ORDER], double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

        double erro = Vref - X[1];


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
            pid_sum += outsideLimit/pid_kp;
        }
        else if (Ve > upperLimit)
        {
            outsideLimit = upperLimit - Ve;
            Ve = upperLimit;
            pid_sum += outsideLimit/pid_kp;
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
