#include <src/Equilibrium/reference_update.h>

namespace Equilibrium
{

    static double Xe[SYSTEM_ORDER];

    static double r[2] = {0, 0};
    static double e[2] = {0, 0};

    static double numPID[2] = {0, 0};
    static double denPID[2] = {0, 0};

    void Configure(void)
    {
        Xe[0] = 0;
        Xe[1] = 0;

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

    void UpdateReference(double Vout, double u)
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
//            r[0] = 0.5000*e[0] - 0.4700*e[1] + r[1];
            r[0] = numPID[0]*e[0] + numPID[1]*e[1] - denPID[1]*r[1];

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
                r[0] += outsideLimit;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
                r[0] += outsideLimit;
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
                r[0] += outsideLimit;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
                r[0] += outsideLimit;
            }

            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - pow(Ve,2)/(R*Ro) ) );
            break;

        case ID_BuckBoost:
            //
            // Limits for Buck-Boost
            //
            upperLimit = 3*u; lowerLimit = -50;

            if (Ve < lowerLimit)
            {
                outsideLimit = lowerLimit - Ve;
                Ve = lowerLimit;
                r[0] += outsideLimit;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
                r[0] += outsideLimit;
            }

            Ie = (u/(2*R) - sqrt( pow(u,2)/(4*pow(R,2)) - Ve*(Ve+u)/(R*Ro) ) );
            break;

        case ID_BuckBoost3:
            //
            // Limits for Buck-Boost
            //
            upperLimit = 3*u; lowerLimit = -5;

            if (Ve < lowerLimit)
            {
                outsideLimit = lowerLimit - Ve;
                Ve = lowerLimit;
                r[0] += outsideLimit;
            }
            else if (Ve > upperLimit)
            {
                outsideLimit = upperLimit - Ve;
                Ve = upperLimit;
                r[0] += outsideLimit;
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
        r[0] = Vref;
        r[1] = Vref;

        e[0] = 0;
        e[1] = 0;
    }
}
