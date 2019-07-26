#include <src/Core/Controller/switched_controller.h>

namespace Controller
{
    void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        //
        // Buck Converter - Rule 2
        //
        P[0][0] = 1.1656e-06;
        P[0][1] = 1.7630e-07;
        P[1][0] = 1.7630e-07;
        P[1][1] = 1.7635e-04;
    }
}
