#include <string.h>
#include "F28x_Project.h"
#include <src/Config/CONFIGURATIONS.h>
#include <src/Config/DEFINES_LP28379D.h>
#include <src/Config/CONFIG_GPIO_V1_LP28379D.h>
#include <src/Config/SPI_DAC.h>
#include <src/Config/COM_SERIAL.h>


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
//$	  PROT�TIPO DAS FUN��ES PARA EXEXUTAR TESTES	$$
//$ 	  Obs.: TESTES NA PLACA VERSAO V4		    $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void TESTA_LEDS(void);
void TESTA_BOTOES(void);
void TESTA_RELES(void);
void TESTA_DAC_SPI(void);
void TESTA_PWM(void);
void TESTA_BKR_RST(void);
void TESTA_RS232(void);
void TESTA_SAIDAS_ISOLADAS(void);
void TESTA_ENTRADAS_ANALOGICAS(void);
void TESTA_ENCODER(void);


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
float Ix=0,Iy=0,Iz=0, Icc2=0,Vcc2=0,Vx=0, Vy=0, Vz=0;				// Grandezas medidas entrada esquerda
float Iu=0,Iv=0,Iw=0, Icc1=0,Vcc1=0,Vu=0, Vv=0, Vw=0;				// Grandezas medidas entrada direita
//
// MEDI��O DE VELOCIDADE
//
float pos=0, dir=0;
float vel_rpm_inst=0;			// Velocidade medida em rpm

//
// DAC
//
int grupo_dac = 0;

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
        //TESTA_RELES();                // RELE 1 - NOK (v1)
        //TESTA_DAC_SPI();              // OK
        TESTA_PWM();                  // OK
        //TESTA_BKR_RST();              // OK
        //TESTA_SAIDAS_ISOLADAS();      // OK
        //TESTA_ENCODER();              // OK
        //TESTA_ENTRADAS_ANALOGICAS();  // OK - Vx n�o funciona placa V1.1
    }
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_LEDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO LEDS 1-4 19-21  DA PLACA ligados aos GPIO 67,69,71,71	   %
//OS LEDS PISCA 20 VEZES A CADA CHAMADA  DESTA FUN��O										   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_LEDS(void){
	int cont=0;
	float flag;
	while(cont<20){
        if(flag > 800000){
            LIGALED1;
            LIGALED2;
            //LIGALED3;
            //LIGALED4;
        }
        else{
            DESLIGALED1;
            DESLIGALED2;
            //DESLIGALED3;
            //DESLIGALED4;
        }

        if(flag >= 1600000){
            flag = 0;
            cont++;
        }

        flag++;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_BOTOESS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 49 e 48, que est�o ligados aos bot�es 1 e 2               %
//Os LEDS 1,2 -19,18 ser�o ligados enquanto o bot�o BOT1 for pressionado                       %
//Os LEDS 3,4- 20,21 ser�o ligados enquanto o bot�o BOT2 for pressionado                       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_BOTOES(void){
	int cont=0;
	float flag;

	while(cont<20){
        if(BOT1==1){
            LIGALED1;
            //LIGALED3;
        }
        else{
          DESLIGALED1;
          //DESLIGALED3;
        }

        if(BOT2==1){
            LIGALED2;
            //LIGALED4;
        }
        else{
            DESLIGALED2;
            //DESLIGALED4;
        }

        if(flag >= 1600000){
            flag = 0;
            cont++;
        }
        flag++;
	}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_REL�S%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 23 e 16, que est�o ligados aos rel�s RELE1 E RELE2        %
//Os rel�s s�o ligados e desligados a cada 1 segundo										   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void TESTA_RELES(void){
	int cont=0;
	float flag;

	while(cont<20){
        if(flag > 900000)     //9000 //11000
        {
            DESLIGARELE1;
            DESLIGARELE2;
        }
        else
        {
            LIGARELE1;
            LIGARELE2;
        }
        if(flag >= 1600000)     //9000 //11000
        {
            flag = 0;
            cont++;
        }

        flag++;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_DAC_SPI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a DAC por SPI 															   %
// S�O CRIADAS SAIDAS EM RAMPAS QUE S�O ENVIADAS PARA OS CANAIS DA DAC						   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_DAC_SPI(void)
{
    static int C = 0;

	int cont=0;
	float flag;
	int i=0;

	while(cont<20){
	    C=C+1;

	    enviar_dac_spi_4Canais(C,C,C,C);

	    for (i = 0; i < 400; i++);

	    //enviar_dac_spi_uni(3,C);
	    //enviar_dac_spi_4Canais(C,C,C,C);
	    if(C==4095){
	        C=0;
	    }
	    if(flag >= 16000)     //9000 //11000
	    {
	        flag = 0;
	        cont++;
	    }
	    flag++;
	}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_PWN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os canais os 12 canais ePWM												   %
// OS PWM 1-6 s�o acionados com indice de modula��o (m) que varia de de-1 at�1				   %
// Caso o bot�o B1 seja acionado o Trip zone dos sensores � acionado					       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_PWM(void)
{
    int cont=0;
    float m=-1;

	EALLOW;
	//Inicializa��o de pwm em zero
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZEINT.bit.OST = 1;

    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;

    EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;

    EPwm4Regs.TZCLR.bit.OST = 1;
    EPwm4Regs.TZEINT.bit.OST = 1;

    EPwm5Regs.TZCLR.bit.OST = 1;
    EPwm5Regs.TZEINT.bit.OST = 1;

    EPwm6Regs.TZCLR.bit.OST = 1;
    EPwm6Regs.TZEINT.bit.OST = 1;
    EDIS;

    while(cont<20)
    {
        if(m>1){
            m=-1;
            cont++;
        }
        else{
            m=m+0.000001;
        }

        //#######################################
        //  ATUALIZA PWM 1-3
        EPwm1Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm1A
        EPwm1Regs.CMPB.bit.CMPB = EPwm1Regs.CMPA.bit.CMPA;	//EPwm1B
        EPwm2Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm2A
        EPwm2Regs.CMPB.bit.CMPB = EPwm2Regs.CMPA.bit.CMPA;  //EPwm2B
        EPwm3Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm3A
        EPwm3Regs.CMPB.bit.CMPB = EPwm3Regs.CMPA.bit.CMPA;  //EPwm3B

        //#######################################
        //  ATUALIZA PWM4-6
        EPwm4Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm4A
        EPwm4Regs.CMPB.bit.CMPB = EPwm4Regs.CMPA.bit.CMPA;  //EPwm4B
        EPwm5Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm5A
        EPwm5Regs.CMPB.bit.CMPB = EPwm5Regs.CMPA.bit.CMPA;  //EPwm5B
        EPwm6Regs.CMPA.bit.CMPA = 2500*(0.5*(1+m));         //EPwm6A
        EPwm6Regs.CMPB.bit.CMPB = EPwm6Regs.CMPA.bit.CMPA;  //EPwm6B

        if(BOT1==1){
            LISGATZSENSORES;
        }
        else{
            DESLISGATZSENSORES;
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS_RESET/BKR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa as sa�das de reset e de break dos VSCs									   %
// Caso o bot�o B1 seja acionado as saidas RESETVSC(gpio62) E RESETVSC2 s�o acionadas		   %
// Caso o bot�o B2 seja acionado as saidas BRK e BKR2 s�o acionadas 					       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_BKR_RST(void)
{
	int cont=0;
	float flag;

	while(cont<20)
	{
	    if(BOT1==1){
            LIGARESETVSC;
            LIGARESETVSC2;
            LISGATZSENSORES;
	    }
	    else{
            DESLIGARESETVSC;
            DESLIGARESETVSC2;
            DESLISGATZSENSORES;
	    }

        if(BOT2==1){
            LIGABKR;
            LIGABKR2;
        }
        else{
            DESLIGABKR;
            DESLIGABKR2;
        }

        if(flag >= 1600000) {
            flag = 0;
            cont++;
        }

        flag++;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS ISOLADAS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 27,21,61,60 que controlam as SAIDAS ISOLADAS1-4		   %
//AS SAIDAS PISCAM 20 VEZES A CADA CHAMADA  DESTA FUN��O									   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void TESTA_SAIDAS_ISOLADAS(void)
{
	int cont=0;
	float flag;

	while(cont<20)
	{
        if(flag > 800000){
            LIGASAIDAISO1;
            LIGASAIDAISO2;
            LIGASAIDAISO3;
            LIGASAIDAISO4;
        }
        else{
            DESLIGASAIDAISO1;
            DESLIGASAIDAISO2;
            DESLIGASAIDAISO3;
            DESLIGASAIDAISO4;
        }
        if(flag >= 1600000){
            flag = 0;
            cont++;
        }
        flag++;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA CANAIS DE ENTRADAS ANAL�GICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde s�o lidos o 16 canais anal�gicos			%		   	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_ENTRADAS_ANALOGICAS(void)
{
	IER |= 3;
	EINT;
	ERTM;

	switch(grupo_dac){
	case 0:
        enviar_dac_spi_4Canais(Ix,Iy,Iz,Icc2);
        break;
	case 1:
        enviar_dac_spi_4Canais(Vx,Vy,Vz,Vcc2);
        break;
	case 2:
        enviar_dac_spi_4Canais(Iu,Iv,Iw,Icc1);
        break;
	case 3:
        enviar_dac_spi_4Canais(Vu,Vv,Vw,Vcc1);
        break;
	default:
	    break;
	}
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA ENCODER EQEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde � medida a velocidade atraves da fun��o	%
// medir_velocidade(angulo, dire��o)					 								      	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_ENCODER(void)
{
	IER |= 3;
	EINT;
	ERTM;
	enviar_dac_spi_4Canais(pos*11.37,pos*11.37,vel_rpm*2.04,vel_rpm*(-2.04));
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA RS232 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a comunica��o por RS232    SCI-A on GPIO35 - GPIO36                         %
// Envia a mensagem TX-OK 20 vezes e espera a chegada do caracter  T definido na fun�� SCI_RX   %				 								      	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_RS232(void)
{
    int cont=0;
    float flag;

    while(cont<20)
    {
        if(flag >= 1600000){

            SCIB_RX();

            if (RS232B_buffer == 'T'){
                RS232B_buffer=0;
                strcpy(RS232B_message,"RS232OK!");
                SCIB_TX();
            }

            SCIC_RX();

            if (RS232C_buffer == 'T'){
                RS232C_buffer=0;
                strcpy(RS232C_message,"RS232OK!");
                SCIC_TX();
            }
        }
        flag++;
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
