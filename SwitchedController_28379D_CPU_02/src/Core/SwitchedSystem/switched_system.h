#ifndef SRC_CORE_SWITCHED_SYSTEM_H_
#define SRC_CORE_SWITCHED_SYSTEM_H_

#include <src/Util/Common/constants.h>
#include <src/Util/Math/matrix.h>

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

#endif /* SRC_CORE_SWITCHED_SYSTEM_H_ */
