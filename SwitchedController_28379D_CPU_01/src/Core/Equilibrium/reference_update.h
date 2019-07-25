#ifndef SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/Util/Math/matrix.h>
#include <src/Util/Common/constants.h>

using namespace Math;

namespace Equilibrium
{
    const double pid_kp = 5;
    const double pid_ki = 140;

    static double pid_sum = 0;

    void Configure(void);

    void UpdateReference(double Vref, Vector* X, double u);

    Vector* GetReference();
}

#endif /* SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
