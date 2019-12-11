#include <src/HAL/GateDrive/gate_drive.h>

namespace GateDrive
{
    void SetState(int gate, bool state)
    {
        GPIO_WritePin(gate, state);
    }
}
