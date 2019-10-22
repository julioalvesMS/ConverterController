#include <src/Relay/Relay.h>

namespace Relay
{

    static bool s_pre_load_state = true;

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
}
