#include <src/HAL/GateDrive/gate_drive.h>

namespace GateDrive
{

    void Configure()
    {
        //
        // Configure switches GPIO
        //
        GPIO_SetupPinMux(GPIO_S1, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(GPIO_S1, GPIO_OUTPUT, GPIO_PUSHPULL);

        GPIO_SetupPinMux(GPIO_S2, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(GPIO_S2, GPIO_OUTPUT, GPIO_PUSHPULL);

    }

    void SetState(short gate, bool state)
    {
        GPIO_WritePin(gate, state);
    }
}
