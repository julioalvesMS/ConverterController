#ifndef SRC_SWITCHED_SYSTEM_H_
#define SRC_SWITCHED_SYSTEM_H_

#include <src/Common/constants.h>
#include <src/Math/matrix.h>

using namespace Math;

namespace SwitchedSystem
{

    class SubSystem
    {
    public:
        Matrix A;
        Vector B;
        Matrix Q;

    };

    class System
    {
    public:
        SubSystem subSystems[SUBSYSTEMS_COUNT];
    };

}

#endif /* SRC_SWITCHED_SYSTEM_H_ */
