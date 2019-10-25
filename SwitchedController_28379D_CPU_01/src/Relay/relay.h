#ifndef SRC_RELAY_RELAY_H_
#define SRC_RELAY_RELAY_H_

#include "F28x_Project.h"
#include <src/Config/DEFINES_LP28379D.h>

namespace Relay
{
    bool PreLoadCapacitor(bool pre_load);

    bool StepOutputLoad(bool loadStep);
}

#endif /* SRC_RELAY_RELAY_H_ */
