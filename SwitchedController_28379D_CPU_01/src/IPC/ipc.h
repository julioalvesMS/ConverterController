#ifndef SRC_IPC_IPC_H_
#define SRC_IPC_IPC_H_

#include "F2837xD_Ipc_drivers.h"

#include <src/settings.h>
#include <src/IPC/protocol.h>

namespace IPC
{
    void Configure(void);


    void SendData(void);


    void ReceiveData(void);


    void CPUCommunication(void);
}


#endif /* SRC_IPC_IPC_H_ */
