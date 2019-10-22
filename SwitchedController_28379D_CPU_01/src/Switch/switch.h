#ifndef SRC_SWITCH_SWITCH_H_
#define SRC_SWITCH_SWITCH_H_

#include "F28x_Project.h"
#include <src/settings.h>
#include <src/Converter/base_converter.h>

#define DISBALE_SWITCHES -1

extern bool CapacitorPreLoad;

namespace Switch
{
    void Configure();

    bool SetState(int state);
}

#endif /* SRC_SWITCH_SWITCH_H_ */
