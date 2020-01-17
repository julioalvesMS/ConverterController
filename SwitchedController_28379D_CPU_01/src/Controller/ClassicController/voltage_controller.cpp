#include <src/Controller/ClassicController/voltage_controller.h>

namespace VoltageController
{
    static double d[2] = {0, 0};
    static double e[2] = {0, 0};

    static double numPID[2] = {0, 0};
    static double denPID[2] = {0, 0};

    void LoadController(void)
    {
        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            Buck::GetClassicVoltageController(numPID, denPID);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetClassicVoltageController(numPID, denPID);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetClassicVoltageController(numPID, denPID);
            break;

        default:
            break;
        }
    }


    double Update(double Vref, double Vout, double Vin)
    {
        double upperLimit, lowerLimit;
        double duty;

        lowerLimit = -0.1;

        switch (activeConverter)
        {
        case BaseConverter::ID_Boost:
            upperLimit = 0.9*(1 - sqrt(Rratio));
            break;

        case BaseConverter::ID_BuckBoost:
            upperLimit = 0.9*(Rratio + 1 - sqrt(Rratio*(1 + Rratio)));
            break;

        default:
            upperLimit = 1;
            break;
        }

        e[1] = e[0];
        d[1] = d[0];

        e[0] = Vref - Vout;

//        d[0] = 0.1230*e[0] - 0.1227*e[1] + d[1];
        d[0] = numPID[0]*e[0] + numPID[1]*e[1] - denPID[1]*d[1];

        if (d[0] > upperLimit) d[0] = upperLimit;
        else if (d[0] < lowerLimit) d[0] = lowerLimit;

        duty = d[0];

        if (duty > 1) duty = 1;
        else if (duty < 0) duty = 0;

        return duty;
    }

    void ResetController(void)
    {
        d[0] = 0;
        d[1] = 0;

        e[0] = 0;
        e[1] = 0;
    }
}
