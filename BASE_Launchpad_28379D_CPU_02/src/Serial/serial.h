#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include <stdio.h>
#include "F28x_Project.h"

#include <src/Config/CONFIGURATIONS.h>
#include <src/DAC/dac.h>
#include <src/IPC/ipc.h>
#include <src/IPC/protocol.h>

#define MESSAGES_COUNT 1


static char protocol_message[8];
static int  data_index = 0;


void Serial_Configure(void);

void Serial_GenerateProtocol(void);

void Serial_SendMessage(void);

void Serial_ReceiveMessage(void);

#endif /* SRC_SERIAL_H_ */
