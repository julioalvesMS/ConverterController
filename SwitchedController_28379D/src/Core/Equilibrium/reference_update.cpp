#include <src/Core/Equilibrium/reference_update.h>

using namespace Math;

namespace Equilibrium
{
    const double pid_kp = 0.5;
    const double pid_ki = 14;

    static double pid_sum = 0;

    static Vector Xe;



    void init(void)
    {
        Xe.data[0] = 0;
        Xe.data[1] = 0;
    }

    void referenceUpdate(double Vref, Vector* X, double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

        double erro = Vref - X->data[1];


        pid_sum += erro*BOARD_PERIOD/PERIOD_UNIT;

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

    Vector* getReference()
    {
        return &Xe;
    }
}
