
//
// Includes
//
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/DAC/dac.h>
#include <src/IPC/ipc.h>
#include <src/IPC/protocol.h>
#include <src/Serial/serial.h>
#include <src/Timer/timer.h>

using namespace DAC_SPI;
using namespace Protocol;


//
// Interruptions defined in main
//
__interrupt void Interruption_CommunicationTimer(void);


//
// Global static variables
//
double Vin, Vout, IL, Iout;
double Vout_mean;
double Vref;
double loadResistance;
double VoltageRipple;

int ADC_Vout, ADC_Vin, ADC_IL, ADC_Iout;

int activeConverter;
int controlStrategy;
int CorrectionMethod;
int protection;
int SwitchingFrequency, ADCFrequency;

bool ConverterEnabled;
bool OutputLoadStep;
bool ModeHoppingEnabled;

DAC_SPI::Channel DacChannel = DAC_SPI::CH_CONTROLE;

CommunicationCommand IPC_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];
int IPC_BufferIndex = 0;

bool OpenCommunication = false;

double stateDutyCycle[4] = {0, 0, 0, 0};

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
    Timer::Configure();
    Serial::Configure();
    DAC_SPI::Configure();
    IPC::Configure();

    //
    // Wait until Shared RAM is available.
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1));

    Timer::CommunicationTimer_Start();

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    loop_count = 0;
    for(;;)
    {
        IPC::CPUCommunication();

        //
        // Receive from PC
        //
        Serial::ReceiveMessage();

        //
        // Send to DAC
        //
        DAC_SPI::SendData(DacChannel);

        if(OpenCommunication)
        {
            loop_count++;
            //
            // Send to PC
            //
            Serial::SendMessage();

            if (loop_count >= MESSAGES_COUNT)
            {
                OpenCommunication = false;
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

    OpenCommunication = true;
}

