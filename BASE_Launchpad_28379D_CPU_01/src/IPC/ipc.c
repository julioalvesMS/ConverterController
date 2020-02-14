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

void IPC_ReceiveData(void)
{
    int i;
    for(i=0;i<IPC_COMMAND_BUFFER_SIZE; i++)
    {
        IPC_CommandBuffer[i] = shared_CommandBuffer[i];
    }
    IPC_BufferIndex = shared_BufferIndex;
}


void IPC_SendData(void)
{
    shared_Ix = Ix;
    shared_Iy = Iy;
    shared_Iz = Iz;
    shared_Icc2 = Icc2;

    shared_Vx = Vx;
    shared_Vy = Vy;
    shared_Vz = Vz;
    shared_Vcc2 = Vcc2;

    shared_Iu = Iu;
    shared_Iv = Iv;
    shared_Iw = Iw;
    shared_Icc1 = Icc1;

    shared_Vu = Vu;
    shared_Vv = Vv;
    shared_Vw = Vw;
    shared_Vcc1 = Vcc1;
}


void IPC_CPUCommunication(void)
{
    // If there is no pending flag, transfer data between the CPUs
    if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
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
        IPCLtoRFlagSet(IPC_FLAG10);
    }
}
