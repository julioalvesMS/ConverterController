#ifndef SRC_CONVERTER_BUCK_H_
#define SRC_CONVERTER_BUCK_H_

#include <src/Util/Common/constants.h>
#include <src/Util/Math/matrix.h>
#include <src/Core/SwitchedSystem/switched_system.h>

using namespace Math;
using namespace SwitchedSystem;

namespace Buck
{
    System* getSys();

    void defineSystem();
}

#endif  /* SRC_CONVERTER_BUCK_H_ */
