#ifndef SRC_HAL_TIMER_H_
#define SRC_HAL_TIMER_H_

#include "F28x_Project.h"

namespace Timer
{
    void Configure();

    void MainLoop_Start();

    void Switch_Start();

    void Switch_Stop();
}

#endif /* SRC_HAL_TIMER_H_ */
