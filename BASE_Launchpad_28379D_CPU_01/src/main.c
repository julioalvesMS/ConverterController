
//
// Includes
//
#include <src/Config/CONFIG_GPIO_DUAL_LP28379D.h>
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/DAC_PWM/dac.h>
#include <src/IPC/ipc.h>
#include <src/Timer/timer.h>

//
// MEDIÇÃO DE VELOCIDADE
//

extern float medir_velocidade(float pos, float dir);
float pos_2;            // Posição final
float pos_1=0;          // Posição da medida anterior
int divF=0;             // Divisor de frenquencia para medição
float vel_rpm=0;        // Velocidade medida em rpm

double pos=0, dir=0;
double vel_rpm_inst=0;          // Velocidade medida em rpm

//
// Functions in main
//
void ConfigureCPU02(void);

//
// Interruptions defined in main
//
__interrupt void Interruption_ADC(void);
__interrupt void Interruption_Timer1(void);


//
// VARIÁVEIS MEDIDAS NOS CANAIS ANALÓGICOS
//
double Ix=0, Iy=0, Iz=0, Icc2=0, Vcc2=0, Vx=0, Vy=0, Vz=0;   // Grandezas medidas entrada esquerda
double Iu=0, Iv=0, Iw=0, Icc1=0, Vcc1=0, Vu=0, Vv=0, Vw=0;   // Grandezas medidas entrada direita

//
// IPC Communication
//
CommunicationCommand IPC_CommandBuffer[IPC_COMMAND_BUFFER_SIZE];
int IPC_BufferIndex = 0;


//
// DAC PWM
//
Uint16 grupo_dac = 0;


void main(void)
{
    InitSysCtrl();          // Inicializações básicas
    InitGpio();             // Inicializações padrão para as GPIOs
    InitIpc();              // Inicializações para comunicação entre as CPUs
    InitCpuTimers();        // Inicializações do timer 0, 1 e 2

    //
    // Step 2. Give control of SPI-A to CPU2
    //
    EALLOW;
    DevCfgRegs.CPUSEL6.bit.SPI_B = 1;
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;
    CpuSysRegs.SECMSEL.bit.PF1SEL = 1;      // Set EPWM Secondary Master to DMA
    EDIS;

    //
    // Step 3. Send IPC to CPU2 telling it to proceed with configuring the SPI
    //
    IpcRegs.IPCSET.bit.IPC0 = 1;

    DINT;                   // Disable all interrupts

    Gpio_setup();           // Configura os pinos de GPIO de acordo com a ligaçaõ na placa


    InitPieCtrl();          // basic setup of PIE table

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();                         // default ISR's in PIE

    EALLOW;
    PieVectTable.ADCA1_INT = &(Interruption_ADC);
    PieVectTable.TIMER1_INT = &(Interruption_Timer1);
    EDIS;

    IER |= M_INT1;      // Enable CPU Interrupt 1
    IER |= M_INT2;      // Enable CPU Interrupt 1
    IER |= M_INT13;     // Timer01
    IER |= M_INT14;     // Timer02

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // ADC
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;  // Timer

    // Configuração do canais analógicos de entrada
    Setup_ADC();

    // Configuração do canais de PWM e TZ
    Setup_ePWM();

    // Configuração do ENCODER  GPIO EQEP
    Setup_EQEP();

    Timer_Configure();
    IPC_Configure();
    DAC_PWM_Configure();


    EALLOW;
    IpcRegs.IPCSET.bit.IPC1 = 1;
    EDIS;

    ConfigureCPU02();

    Timer_Timer1_Start();

    DESLISGATZSENSORES;

    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    for(;;)
    {
        IPC_CPUCommunication();
    }
}


void ConfigureCPU02(void)
{

    //
    // Give Memory Access to GS1 SARAM to CPU02
    //
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS1 &
            MemCfgRegs.GSxMSEL.bit.MSEL_GS12 &
            MemCfgRegs.GSxMSEL.bit.MSEL_GS13))
    {
        EALLOW;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
        EDIS;
    }
}


//
// Interrupção executada a cada 100ms para medições temporais
//
__interrupt void Interruption_Timer1(void)
{
    CpuTimer1.InterruptCount++;
}


//
// Interruption_MainLoopPeriod - CPU Timer0 ISR with interrupt counter
//
__interrupt void Interruption_ADC(void)
{
    // Sensores lado esquerdo da placa

    // Ix, Iy e Iz e Icc2
    Ix   = (AdccResultRegs.ADCRESULT0);
    Iy   = (AdcaResultRegs.ADCRESULT3);
    Iz   = (AdcbResultRegs.ADCRESULT0);
    Icc2 = (AdcbResultRegs.ADCRESULT1);

    // Vx, Vy e Vz Vcc2
    Vx   = (AdcaResultRegs.ADCRESULT2);
    Vy   = (AdcaResultRegs.ADCRESULT0);
    Vz   = (AdccResultRegs.ADCRESULT1);
    Vcc2 = (AdcbResultRegs.ADCRESULT4);

    // Sensores lado direito da placa

    // Iu, Iv e Iw e Icc1
    Iu   = (AdccResultRegs.ADCRESULT4);
    Iv   = (AdcaResultRegs.ADCRESULT1);
    Iw   = (AdcbResultRegs.ADCRESULT3);
    Icc1 = (AdccResultRegs.ADCRESULT2);

    // Vu, Vz e Vw Vcc1
    Vu   = (AdccResultRegs.ADCRESULT3);
    Vv   = (AdcaResultRegs.ADCRESULT4);
    Vw   = (AdcbResultRegs.ADCRESULT2);
    Vcc1 = (AdcaResultRegs.ADCRESULT5);


    dir = EQep1Regs.QEPSTS.bit.QDF;
    pos = 0.025*(EQep1Regs.QPOSCNT);
    vel_rpm=medir_velocidade(pos,dir);

    //TRATAMENTO DA INTERRUPÇÃO

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag

    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    DAC_PWM_SendData(grupo_dac);

    return;
}
