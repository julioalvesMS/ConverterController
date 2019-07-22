#ifndef SRC_CONVERTER_BUCK_H_
#define SRC_CONVERTER_BUCK_H_

#include <src/Math/matrix.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace Math;
using namespace SwitchedSystem;

namespace Buck
{
    System* getSys();

    void defineSystem();
}

#endif  /* SRC_CONVERTER_BUCK_H_ */
