#ifndef SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include "F28x_Project.h"
#include <src/Common/constants.h>
#include <src/settings.h>

namespace Equilibrium
{
    const double pid_kp = REFERENCE_CONTROLLER_PID_KP;
    const double pid_ki = REFERENCE_CONTROLLER_PID_KI;

    void Configure(void);

    void UpdateReference(double Vref, double Vout, double u);

    double* GetReference();
}

#endif /* SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
