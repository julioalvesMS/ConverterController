#ifndef SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/settings_cpu_01.h>
#include <src/Util/Common/constants.h>

namespace Equilibrium
{
    const double pid_kp = REFERENCE_CONTROLLER_PID_KP;
    const double pid_ki = REFERENCE_CONTROLLER_PID_KI;

    void Configure(void);

    void UpdateReference(double Vref, double Vout, double u);

    double* GetReference();
}

#endif /* SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
