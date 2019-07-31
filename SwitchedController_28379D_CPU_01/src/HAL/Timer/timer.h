#ifndef SRC_HAL_TIMER_H_
#define SRC_HAL_TIMER_H_

#include "F28x_Project.h"
#include <src/Util/Common/constants.h>

namespace Timer
{
    void Configure();

    void ReferenceUpdate_Start();
}

#endif /* SRC_HAL_TIMER_H_ */
