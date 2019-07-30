#ifndef SRC_HAL_GATE_DRIVE_H_
#define SRC_HAL_GATE_DRIVE_H_

#include "F28x_Project.h"

#define GPIO_S1    6
#define GPIO_S2    7
#define GPIO_A1    8
#define GPIO_A2    9

namespace GateDrive
{
    void Configure();

    void SetState(short gate, bool state);
}

#endif /* SRC_HAL_GATE_DRIVE_H_ */
