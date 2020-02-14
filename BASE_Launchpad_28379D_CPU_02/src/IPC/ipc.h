#ifndef SRC_IPC_IPC_H_
#define SRC_IPC_IPC_H_

#define IPC_COMMAND_BUFFER_SIZE         64

#include "F2837xD_Ipc_drivers.h"

#include <src/IPC/protocol.h>


void IPC_Configure(void);


void IPC_SendData(void);


void IPC_ReceiveData(void);


void IPC_CPUCommunication(void);


#endif /* SRC_IPC_IPC_H_ */
