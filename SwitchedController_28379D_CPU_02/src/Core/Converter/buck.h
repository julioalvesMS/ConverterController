#ifndef SRC_CORE_CONVERTER_BUCK_H_
#define SRC_CORE_CONVERTER_BUCK_H_

#include <src/Util/Common/constants.h>
#include <src/Core/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace Buck
{
    System* GetSys();

    void DefineSystem();
}

#endif  /* SRC_CORE_CONVERTER_BUCK_H_ */
