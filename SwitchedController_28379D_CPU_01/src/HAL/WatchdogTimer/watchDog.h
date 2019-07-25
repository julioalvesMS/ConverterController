#ifndef SRC_HAL_WATCHDOG_H_
#define SRC_HAL_WATCHDOG_H_

#include "F2837xD_Device.h"

namespace WatchDog
{

    void enable();

    void step1();

    void step2();
}


#endif /* SRC_HAL_WATCHDOG_H_ */
