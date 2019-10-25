#include <src/Relay/Relay.h>

namespace Relay
{
    static bool s_pre_load_state = true;
    static bool s_output_load_step = false;

    bool PreLoadCapacitor(bool pre_load)
    {
        bool relayEngaged = s_pre_load_state;

        // If the switch is already ate the desired state, do nothing
        if (pre_load == s_pre_load_state)
            return relayEngaged;

        if (pre_load == false)
        {
            LIGARELE2;
        }
        else
        {
            DESLIGARELE2;
        }

        // Register new state
        s_pre_load_state = pre_load;

        return relayEngaged;
    }

    bool StepOutputLoad(bool loadStep)
    {
        bool relayEngaged = s_output_load_step;

        // If the switch is already ate the desired state, do nothing
        if (loadStep == s_output_load_step)
            return relayEngaged;

        if (loadStep)
        {
            LIGASAIDAISO1;
        }
        else
        {
            DESLIGASAIDAISO1;
        }

        // Register new state
        s_output_load_step = loadStep;

        return relayEngaged;
    }
}
