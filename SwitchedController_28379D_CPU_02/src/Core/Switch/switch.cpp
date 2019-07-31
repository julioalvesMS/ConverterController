#include <src/Core/Switch/switch.h>

namespace Switch
{

    static int s_switch_state = -1;

    void SetState(int state)
    {
        //
        // If the switch is already ate the desired state, do nothing
        //
        if (state == s_switch_state)
            return;

        switch(state)
        {
        case 0:
            GateDrive::SetState(GPIO_S2, false);
            GateDrive::SetState(GPIO_S1, true);
            break;
        case 1:
            GateDrive::SetState(GPIO_S1, false);
            GateDrive::SetState(GPIO_S2, true);
            break;
        default:
            GateDrive::SetState(GPIO_S1, false);
            GateDrive::SetState(GPIO_S2, false);
            break;
        }

        //
        // Register new state
        //
        s_switch_state = state;
    }

}
