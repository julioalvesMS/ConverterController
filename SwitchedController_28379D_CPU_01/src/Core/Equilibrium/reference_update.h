#ifndef SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/Util/Common/constants.h>
namespace Equilibrium
{
    const double pid_kp = 0.5;
    const double pid_ki = 0.1;

    static double pid_sum = 0;

    void Configure(void);

    void UpdateReference(double Vref, double X[SYSTEM_ORDER], double u);

    double* GetReference();
}

#endif /* SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
