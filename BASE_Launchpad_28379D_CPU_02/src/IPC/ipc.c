#include <src/IPC/ipc.h>


//
// Variables - CPU2
//
extern CommunicationCommand IPC_CommandBuffer[];
extern int IPC_BufferIndex;

//
// Variables - CPU1
//
extern double Ix, Iy, Iz, Icc2, Vcc2, Vx, Vy, Vz;
extern double Iu, Iv, Iw, Icc1, Vcc1, Vu, Vv, Vw;
extern int grupo_dac;


//
// Shared Memory Variables - CPU2
//
#pragma DATA_SECTION(shared_CommandBuffer, "SHARERAMGS1");
volatile CommunicationCommand shared_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];

#pragma DATA_SECTION(shared_BufferIndex, "SHARERAMGS1");
volatile int shared_BufferIndex;


//
// Shared Memory Variables - CPU1
//
#pragma DATA_SECTION(shared_Ix, "SHARERAMGS0");
volatile int shared_Ix;
#pragma DATA_SECTION(shared_Iy, "SHARERAMGS0");
volatile int shared_Iy;
#pragma DATA_SECTION(shared_Iz, "SHARERAMGS0");
volatile int shared_Iz;
#pragma DATA_SECTION(shared_Icc2, "SHARERAMGS0");
volatile int shared_Icc2;

#pragma DATA_SECTION(shared_Vcc2, "SHARERAMGS0");
volatile int shared_Vcc2;
#pragma DATA_SECTION(shared_Vx, "SHARERAMGS0");
volatile int shared_Vx;
#pragma DATA_SECTION(shared_Vy, "SHARERAMGS0");
volatile int shared_Vy;
#pragma DATA_SECTION(shared_Vz, "SHARERAMGS0");
volatile int shared_Vz;

#pragma DATA_SECTION(shared_Iu, "SHARERAMGS0");
volatile int shared_Iu;
#pragma DATA_SECTION(shared_Iv, "SHARERAMGS0");
volatile int shared_Iv;
#pragma DATA_SECTION(shared_Iw, "SHARERAMGS0");
volatile int shared_Iw;
#pragma DATA_SECTION(shared_Icc1, "SHARERAMGS0");
volatile int shared_Icc1;

#pragma DATA_SECTION(shared_Vcc1, "SHARERAMGS0");
volatile int shared_Vcc1;
#pragma DATA_SECTION(shared_Vu, "SHARERAMGS0");
volatile int shared_Vu;
#pragma DATA_SECTION(shared_Vv, "SHARERAMGS0");
volatile int shared_Vv;
#pragma DATA_SECTION(shared_Vw, "SHARERAMGS0");
volatile int shared_Vw;


void IPC_Configure(void)
{
    int i;
    for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
        IPC_CommandBuffer[i] = None;
}


void IPC_SendData(void)
{
    int i;
    for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
    {
        shared_CommandBuffer[i] = IPC_CommandBuffer[i];
    }

    shared_BufferIndex = IPC_BufferIndex;
}


void IPC_ReceiveData(void)
{
    Ix = shared_Ix;
    Iy = shared_Iy;
    Iz = shared_Iz;
    Icc2 = shared_Icc2;

    Vx = shared_Vx;
    Vy = shared_Vy;
    Vz = shared_Vz;
    Vcc2 = shared_Vcc2;

    Iu = shared_Iu;
    Iv = shared_Iv;
    Iw = shared_Iw;
    Icc1 = shared_Icc1;

    Vu = shared_Vu;
    Vv = shared_Vv;
    Vw = shared_Vw;
    Vcc1 = shared_Vcc1;
}


void IPC_CPUCommunication(void)
{
    // If there is no pending flag, transfer data between the CPUs
    if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
    {
        //
        // Read data from CPU 1
        //
        IPC_ReceiveData();

        //
        // Write data to CPU 1
        //
        IPC_SendData();

        //
        // Set a flag to notify CPU02 that data is available
        //
        IPCRtoLFlagAcknowledge(IPC_FLAG10);
    }
}
