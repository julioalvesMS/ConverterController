#include <src/Controller/ClassicController/voltage_current_controller.h>

namespace VoltageCurrentController
{
    static double d[2] = {0, 0};
    static double eI[2] = {0, 0};

    static double ir[2] = {0, 0};
    static double eV[2] = {0, 0};

    static double numVPID[2] = {0, 0};
    static double denVPID[2] = {0, 0};

    static double numIPID[2] = {0, 0};
    static double denIPID[2] = {0, 0};

    static double ILref = 0;

    void LoadController(void)
    {
        switch(activeConverter)
        {
        case BaseConverter::ID_Buck:
            Buck::GetClassicVoltageCurrnetController(numVPID, denVPID, numIPID, denIPID);
            break;
        case BaseConverter::ID_Boost:
            Boost::GetClassicVoltageCurrnetController(numVPID, denVPID, numIPID, denIPID);
            break;
        case BaseConverter::ID_BuckBoost:
            BuckBoost::GetClassicVoltageCurrnetController(numVPID, denVPID, numIPID, denIPID);
            break;

        default:
            break;
        }
    }


    double Update(double Vref, double Vout, double IL, double Vin)
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

        eV[1] = eV[0];
        ir[1] = ir[0];

        eI[1] = eI[0];
        d[1] = d[0];

        //
        // Voltage Control
        //
        eV[0] = Vref - Vout;

        ir[0] = numVPID[0]*eV[0] + numVPID[1]*eV[1] - denVPID[1]*ir[1];

        ILref = ir[0];


        //
        // Current Control
        //
        eI[0] = ILref - IL;

        d[0] = numIPID[0]*eI[0] + numIPID[1]*eI[1] - denIPID[1]*d[1];

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

        eI[0] = 0;
        eI[1] = 0;

        ir[0] = 0;
        ir[1] = 0;

        eV[0] = 0;
        eV[1] = 0;
    }
}
