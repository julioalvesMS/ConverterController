#ifndef SRC_CONVERTER_BUCK_H_
#define SRC_CONVERTER_BUCK_H_

#include <src/Common/constants.h>
#include <src/SwitchedSystem/switched_system.h>

using namespace SwitchedSystem;

namespace Buck
{
    System* GetSys();

    void DefineSystem();
}

#endif  /* SRC_CONVERTER_BUCK_H_ */
