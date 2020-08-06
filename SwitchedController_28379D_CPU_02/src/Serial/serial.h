#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <stdio.h>
#include "F28x_Project.h"

#include <src/settings.h>
#include <src/Config/CONFIGURATIONS.h>
#include <src/DAC/dac.h>
#include <src/IPC/protocol.h>

#define MESSAGES_COUNT 21

namespace Serial
{
    static char protocol_message[8];
    static int  data_index = 0;

    void Configure(void);

    void GenerateProtocol(void);

    void SendMessage(void);

    void ReceiveMessage(void);
}

#endif /* SRC_SERIAL_H_ */
