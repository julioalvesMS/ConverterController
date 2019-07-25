#ifndef SRC_CORE_SWITCH_H_
#define SRC_CORE_SWITCH_H_

#include "F28x_Project.h"
#include <src/HAL/GateDrive/gate_drive.h>
#include <src/HAL/Timer/timer.h>

namespace Switch
{
    void Configure();

    void SetState(short state);

    void TurnOff(short gate);

    void TurnOn(short gate);

    __interrupt void Interruption_SwitchOnDelay(void);
}

#endif /* SRC_CORE_SWITCH_H_ */
