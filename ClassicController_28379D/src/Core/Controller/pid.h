#ifndef SRC_CORE_CONTROLLER_PID_H_
#define SRC_CORE_CONTROLLER_PID_H_

#include <src/Util/Common/constants.h>

namespace PID
{
    const double pid_kp = 0.5;
    const double pid_ki = 0.1;

    static double pid_sum = 0;

    double Update(double Vref, double Vout, double Vin);
}

#endif /* SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
