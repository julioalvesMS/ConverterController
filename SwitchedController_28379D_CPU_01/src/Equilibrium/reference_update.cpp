#include <src/Equilibrium/reference_update.h>

namespace Equilibrium
{

    static double Xe[SYSTEM_ORDER];

    static double r[2] = {0, 0};
    static double e[2] = {0, 0};

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;

        ResetController();
    }

    void UpdateReference(double Vref, double Vout, double u)
    {
        double Ve;
        double Ie;

        double upperLimit, lowerLimit;
        double outsideLimit;

        if (ReferenceControlerEnabled)
        {
            e[1] = e[0];
            r[1] = r[0];

            e[0] = Vref - Vout;
            r[0] = 0.5000*e[0] - 0.4700*e[1] + r[1];

            Ve = r[0];
        }
        else
        {
            Ve = Vref;
        }

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
//                pid_sum += outsideLimit/pid_ki;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
//                pid_sum += outsideLimit/pid_ki;
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
//                pid_sum += outsideLimit/pid_ki;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
//                pid_sum += outsideLimit/pid_ki;
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
//                pid_sum += outsideLimit/pid_ki;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
//                pid_sum += outsideLimit/pid_ki;
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
        r[0] = Xe[1];
        r[1] = Xe[1];

        e[0] = 0;
        e[1] = 0;
    }
}
