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

        GPIO_SetupPinMux(GPIO_A1, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(GPIO_A1, GPIO_OUTPUT, GPIO_PUSHPULL);

        GPIO_SetupPinMux(GPIO_A2, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(GPIO_A2, GPIO_OUTPUT, GPIO_PUSHPULL);


        GPIO_WritePin(GPIO_A1, false);
        GPIO_WritePin(GPIO_A2, false);

    }

    void SetState(short gate, bool state)
    {
        GPIO_WritePin(gate, state);
    }
}
