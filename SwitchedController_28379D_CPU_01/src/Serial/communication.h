#ifndef SRC_COMMUNICATION_H_
#define SRC_COMMUNICATION_H_

#include <stdio.h>
#include "F28x_Project.h"

#include <src/Config/CONFIGURATIONS.h>
#include <src/Equilibrium/reference_update.h>
#include <src/DAC/dac.h>
#include <src/OperationManagement/manager.h>
#include <src/Protection/protection.h>

#include <src/Relay/relay.h>

#define MESSAGES_COUNT 10

namespace Communication
{
    static char protocol_message[8];
    static int  data_index = 0;

    void Configure(void);

    void GenerateProtocol(void);

    void SendMessage(void);

    void ReceiveMessage(void);
}

#endif /* SRC_COMMUNICATION_H_ */
