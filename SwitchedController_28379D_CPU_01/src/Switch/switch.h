#ifndef SRC_SWITCH_SWITCH_H_
#define SRC_SWITCH_SWITCH_H_

#include "F28x_Project.h"
#include <src/settings.h>

#define DISBALE_SWITCHES -1

namespace Switch
{
    void Configure();

    bool SetState(int state);
}

#endif /* SRC_SWITCH_SWITCH_H_ */
