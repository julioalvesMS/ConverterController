#include <src/Core/Equilibrium/reference_update.h>

using namespace Math;

namespace Equilibrium
{

    static Vector Xe;

    void Configure(void)
    {
        Xe.data[0] = 0;
        Xe.data[1] = 0;
    }

    void UpdateReference(double Vref, Vector* X, double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

        double erro = Vref - X->data[1];


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

        Xe.data[0] = Ie;
        Xe.data[1] = Ve;
    }

    Vector* GetReference()
    {
        return &Xe;
    }
}
