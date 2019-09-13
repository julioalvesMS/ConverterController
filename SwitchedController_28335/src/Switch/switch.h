#ifndef SRC_SWITCH_SWITCH_H_
#define SRC_SWITCH_SWITCH_H_

#include "DSP2833x_Device.h"
#include <src/settings.h>

#define DISBALE_SWITCHES -1

namespace Switch
{
    void Configure();

    void SetState(int state);
}

#endif /* SRC_SWITCH_SWITCH_H_ */
