#include <src/Core/Controller/switched_controller.h>

namespace Controller
{
    void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
        //
        // Buck Converter - Rule 2
        //
        P[0][0] = 1.8563e-07;
        P[0][1] = 2.9480e-07;
        P[1][0] = 2.9480e-07;
        P[1][1] = 2.8584e-05;
    }
}
