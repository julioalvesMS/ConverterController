#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Config/CONFIG_GPIO_V2_LP28379D.h>
#include <src/Config/SPI_DAC.h>
#include <src/Config/PWM_DAC.h>
#include <src/Config/COM_SERIAL.h>
#include <src/Testes/testes.h>


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$     FILTROS PARA ENTRADAS ANAL�GICAS            $$
//$      Obs.: UTILIZADO PARA CALIBRA��O            $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
extern float filtro_1(float vP);
extern float filtro_2(float vP);
extern float filtro_3(float vP);
extern float filtro_4(float vP);


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$     COMUNICA��O SERIAL VIA SCI-RS232            $$
//$      Obs.: UTILIZADO EM COM_SERIAL.C            $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
char RS232B_message[8]="";
Uint16 RS232B_buffer;

char RS232C_message[8]="";
Uint16 RS232C_buffer;


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$      VARI�VEIS PARA MEDI��O DE VELOCIDADE       $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
extern float medir_velocidade(float pos, float dir);
float pos_2;            // Posi��o final
float pos_1=0;          // Posi��o da medida anterior
int divF=0;             // Divisor de frenquencia para medi��o
float vel_rpm=0;        // Velocidade medida em rpm


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$	  PROT�TIPO DAS INTERRUP��ES UTILIZADAS 		$$
//$ 	  Obs.: TESTES NA PLACA VERSAO V4		    $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
interrupt void cpu_timer0_isr(void);
interrupt void adc_isr(void);


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$       VARI�VEIS GLOBAIS DO SISTEMA              $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//
// VARI�VEIS MEDIDAS NOS CANAIS ANAL�GICOS
//
float Ix=0, Iy=0, Iz=0, Icc2=0, Vcc2=0, Vx=0, Vy=0, Vz=0;   // Grandezas medidas entrada esquerda
float Iu=0, Iv=0, Iw=0, Icc1=0, Vcc1=0, Vu=0, Vv=0, Vw=0;   // Grandezas medidas entrada direita
//
// MEDI��O DE VELOCIDADE
//
float pos=0, dir=0;
float vel_rpm_inst=0;			// Velocidade medida em rpm

//
// DAC
//
int grupo_dac = 0;

//int C1=60, C2=120, C3=180, C4=240;
int C1=20, C2=40, C3=60, C4=80;

//#############################################################################################
//											C�DIGO PRINCIPAL
//#############################################################################################

void main(void)
{
    InitSysCtrl();          // Inicializa��es b�sicas
    InitGpio();             // Inicializa��es padr�o para as GPIOs
    InitIpc();              // Inicializa��es para comunica��o entre as CPUs
    InitCpuTimers();        // Inicializa��es do timer 0, 1 e 2

    DINT;                   // Desabilita as interrupi��es

    InitPieCtrl();          // Inicializa��es para as interrup��es

    IER = 0x0000;           // Limpa as flgas de interru��es
    IFR = 0x0000;           // Limpa as flgas de interru��es

    InitPieVectTable();     // Inicializa o vetor de interrup��es

	Gpio_setup();           // Configura os pinos de GPIO de acordo com a liga�a� na placa

	EALLOW;
	PieVectTable.ADCA1_INT = &adc_isr; 			// Interrup��o do ADC
	//PieVectTable.EPWM1_TZINT = &ePWM1_TZ_isr;
	EDIS;

	// Configura o Timer 0
	ConfigCpuTimer(&CpuTimer0,200,10000);
    CpuTimer0Regs.TCR.all = 0x4000;

    // Habilita os grupos de interrup��o desejados
	PieCtrlRegs.PIEIER1.bit.INTx1 = 1;			// ADC

    // Configura��o do canais anal�gicos de entrada
    Setup_ADC();

    // Configura��o do canais de PWM e TZ
    Setup_ePWM();

    // Configura��o do ENCODER  GPIO EQEP
    Setup_EQEP();

    // CONFIG  DA COMUNICA��O POR SCIA-RS232
    SCIB_init();
    SCIC_init();

    // Configura SPI-FIFO DA DAC
    spi_fifo_init();
    // Inicializa SPI-FIFO DA DAC
    spi_init();


    // Habilita as interrup��es
    IER |= M_INT1;
    IER |= M_INT2;
    EINT;
    ERTM;

    // Inicia o timer 0
    CpuTimer0Regs.TCR.bit.TSS = 0;

    while(1)
    {
        //TESTA_LEDS();                 // NO
        //TESTA_BOTOES();               // OK
        //TESTA_RS232();                // RS232C - NOK (v1)
        //TESTA_RELES();                // OK
        //TESTA_DAC_PWM();              // OK
        //TESTA_DAC_SPI();              // OK
        //TESTA_PWM();                  // OK
        //TESTA_BKR_RST();              // OK
        //TESTA_SAIDAS_ISOLADAS();      // OK
        //TESTA_ENCODER();              // OK
        //TESTA_ENTRADAS_ANALOGICAS();  // OK - Vx n�o funciona placa V1.1
    }
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%INTERRUP��O ADC%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

interrupt void  adc_isr(void)
{

//#####################################MEDI��ES#####################################################

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


//############################MEDI��O DE VELOCIDADE#################################################

	dir = EQep1Regs.QEPSTS.bit.QDF;
	pos = 0.025*(EQep1Regs.QPOSCNT);
	vel_rpm=medir_velocidade(pos,dir);

	//enviar_dac_spi_4Canais(Vx,Vy,Vz,Vz);

	//TRATAMENTO DA INTERRUP��O

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

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%INTERRUP��O ADC%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
