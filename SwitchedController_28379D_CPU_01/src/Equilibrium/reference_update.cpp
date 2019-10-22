#include <src/Equilibrium/reference_update.h>

namespace Equilibrium
{

    static double Xe[SYSTEM_ORDER];

    static double pid_sum = 0;

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;

        pid_sum = 0;
    }

    void UpdateReference(double Vref, double Vout, double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

#if REFERENCE_UPDATE_ENABLED
        double erro = Vref - Vout;

        pid_sum += erro*REFERENCE_CONTROLLER_PERIOD/PERIOD_UNIT;

        Ve = pid_kp*erro + pid_ki*pid_sum;
#else
        Ve = Vref;
#endif


        switch(activeConverter)
        {
        case ID_Buck:
            //
            // Limits for Buck
            //
            upperLimit = u; lowerLimit = -5;

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
            break;

        case ID_Boost:
            //
            // Limits for Buck
            //
            upperLimit = 3*u; lowerLimit = u*0.5;

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

            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - pow(Ve,2)/(R*Ro) ) );
            break;

        case ID_BuckBoost:
            //
            // Limits for Buck-Boost
            //
            upperLimit = 3*u; lowerLimit = -5;

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

            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*Ro) ) );
            break;
        }


        Xe[0] = Ie;
        Xe[1] = Ve;
    }

    double* GetReference(void)
    {
        return Xe;
    }

    void ResetController(void)
    {
        pid_sum = 0;
    }
}
