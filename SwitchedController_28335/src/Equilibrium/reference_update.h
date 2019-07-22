#ifndef SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/Math/matrix.h>
#include <src/Common/constants.h>

using namespace Math;

namespace Equilibrium
{
    void init(void);

    void referenceUpdate(Vector* X, double u);

    Vector* getReference();
}

#endif /* SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
