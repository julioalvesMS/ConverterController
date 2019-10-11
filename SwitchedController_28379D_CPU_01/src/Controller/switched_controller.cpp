#include <src/Controller/switched_controller.h>

namespace Controller
{
    void GetP(double P[SYSTEM_ORDER][SYSTEM_ORDER])
    {
#if SWITCHING_RULE == CONTINUOUS_RULE_1
        //
        // Buck Converter - Rule 1
        //
        P[0][0] = 4.5236e-05;
        P[0][1] = 9.5852e-06;
        P[1][0] = 9.5852e-06;
        P[1][1] = 5.6602e-05;
#elif SWITCHING_RULE == CONTINUOUS_RULE_2
//        //
//        // Buck Converter - Rule 2
//        //
//        P[0][0] = 0.2135e-04;
//        P[0][1] = 0.0983e-04;
//        P[1][0] = 0.0983e-04;
//        P[1][1] = 0.2921e-04;

        //
        // Buck Converter - Rule 2
        //
        P[0][0] = 4.5236e-05;
        P[0][1] = 9.5852e-06;
        P[1][0] = 9.5852e-06;
        P[1][1] = 5.6602e-05;
#endif
    }
}
