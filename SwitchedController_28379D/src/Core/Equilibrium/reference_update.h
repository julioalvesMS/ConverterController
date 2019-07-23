#ifndef SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/Util/Math/matrix.h>
#include <src/Util/Common/constants.h>

using namespace Math;

namespace Equilibrium
{
    void Configure(void);

    void UpdateReference(double Vref, Vector* X, double u);

    Vector* GetReference();
}

#endif /* SRC_CORE_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
