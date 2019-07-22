#ifndef SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_
#define SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_

#include <src/Util/Math/matrix.h>
#include <src/Util/Common/constants.h>

using namespace Math;

namespace Equilibrium
{
    void init(void);

    void referenceUpdate(Vector* X, double u);

    Vector* getReference();
}

#endif /* SRC_EQUILIBRIUM_REFERENCE_UPDATE_H_ */
