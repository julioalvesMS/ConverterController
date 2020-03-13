
//
// Includes
//
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/DAC_SPI/dac.h>
#include <src/IPC/ipc.h>
#include <src/IPC/protocol.h>
#include <src/Serial/serial.h>
#include <src/Timer/timer.h>


//
// Interruptions defined in main
//
__interrupt void Interruption_CommunicationTimer(void);

//
// DAC
//
int grupo_dac = 0;

//
// CPU Communication
//
CommunicationCommand IPC_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];
int IPC_BufferIndex = 0;

//
// VARIÁVEIS MEDIDAS NOS CANAIS ANALÓGICOS
//
double Ix=0, Iy=0, Iz=0, Icc2=0, Vcc2=0, Vx=0, Vy=0, Vz=0;   // Grandezas medidas entrada esquerda
double Iu=0, Iv=0, Iw=0, Icc1=0, Vcc1=0, Vu=0, Vv=0, Vw=0;   // Grandezas medidas entrada direita

//
// MEDIÇÃO DE VELOCIDADE
//
double pos=0, dir=0;
double vel_rpm_inst=0;          // Velocidade medida em rpm

Uint16 OpenCommunication = 0;

void main(void)
{
    int loop_count;

    InitSysCtrl();          // Inicializações básicas
    InitIpc();              // Inicializações para comunicação entre as CPUs
    InitCpuTimers();        // Inicializações do timer 0, 1 e 2


    //
    // Step 2. Wait for verification from CPU1 that CPU2 has been given control
    // of SPI-A and for control of GS2 to initialize the CMP value to a proper
    // value. This is important to prevent glitching on the EPWM when the first
    // DMA transfer occurs.
    //
    while(IpcRegs.IPCSTS.bit.IPC0 == 0);

    //
    // Step 3. Turn on clock to SPI-A
    //
    CpuSysRegs.PCLKCR8.bit.SPI_A = 1;
    EDIS;

    DINT;                   // Disable all interrupts


    InitPieCtrl();          // basic setup of PIE table

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();                         // default ISR's in PIE


    EALLOW;
    PieVectTable.TIMER1_INT = &(Interruption_CommunicationTimer);

    IER |= M_INT13;     // Timer01
    IER |= M_INT14;     // Timer02

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer
    EDIS;

    //
    // Step 8. Wait for IPC from CPU1 confirming DMA is configured before
    // initializing SPI. Note that because of the way the TXFIFO interrupt
    // is configured a DMA transfer will be triggered immediately after the
    // SPI is released from reset
    //
    while(IpcRegs.IPCSTS.bit.IPC1 == 0);

    //
    // Step 9. Setup SPI for FIFO mode
    //
    Timer_Configure();
    Serial_Configure();
    DAC_SPI_Configure();
    IPC_Configure();

    //
    // Wait until Shared RAM is available.
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1));

    Timer_CommunicationTimer_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    loop_count = 0;
    for(;;)
    {
        IPC_CPUCommunication();

        //
        // Receive from PC
        //
        Serial_ReceiveMessage();

        //
        // Send to DAC
        //
        DAC_SPI_SendData(grupo_dac);


        if(OpenCommunication)
        {
            loop_count++;
            //
            // Send to PC
            //
            Serial_SendMessage();

            if (loop_count >= MESSAGES_COUNT)
            {
                OpenCommunication = 0;
                loop_count = 0;
            }
        }
    }
}


//
// Interrupção executada a cada 100ms para medições temporais
//
__interrupt void Interruption_CommunicationTimer(void)
{
    CpuTimer1.InterruptCount++;

    OpenCommunication = 1;
}

