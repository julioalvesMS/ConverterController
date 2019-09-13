//###################################################################################################
// Autor: T�rcio Andr� dos Santos Barros															#
//																									#
// Programa desenvolvido para testar perif�ricos da placa de condicionamentos (PLACA GERAL) v4		#
//																									#
//###################################################################################################
//
//         DFIG / FAP
//
////////////////////////////////////////////////////////////////////////////////////
///	  			   		CABE�ALHO COM ARQUIVOS COM FUN��ES				  	   	  //
////////////////////////////////////////////////////////////////////////////////////
#include "DSP2833x_Device.h"
#include "DEFINESCONTROLCARDV2.c"
#include "CONFIGURATIONS.h"
#include "SPI_DAC.h"
#include "COM_SERIAL.h"

extern float filtro_1(float vP);
extern float filtro_2(float vP);
extern float filtro_3(float vP);
extern float filtro_4(float vP);

/////////////////////////////////////////////////////////////////
///    PROT�TIPO DAS FUN��ES EXTERNAS PARA CONFIGURAR          //
/////////////////////////////////////////////////////////////////
extern Gpio_setup1();												//Configura os GPIO( Est� em Config_GPIO.c)
extern void InitAdc(void);
extern void InitSysCtrl(void);
extern void InitPieCtrl(void);
extern void InitPieVectTable(void);
extern void InitCpuTimers(void);
extern void ConfigCpuTimer(struct CPUTIMER_VARS *, float, float);
extern void display_ADC(unsigned int);
extern void velocidade(float,float);

//////////////////////////////////////////////////////
///	    PROT�TIPO CONFIGURA��O PERIF�RICOS 		   ///
///   Obs.: Fun��es no arquivo CONFIGURATIONS.C	   ///
//////////////////////////////////////////////////////
extern void Setup_ePWM(void);
extern void Setup_ADC(void);
extern void Setup_EQEP(void);
extern void SCIA_init(void);

//////////////////////////////////////////////////////
///	 	PROT�TIPO DAS FUN��ES DA DAC VIA SPI		//
/// 	  Obs.: Fun��es no arquivo SPI_DAC.C	   ///
//////////////////////////////////////////////////////

extern void spi_fifo_init(); 										// Configura Spi FIFO
extern void spi_init();     				     					// Configura SPI
extern void enviar_dac_spi_uni(Uint16 ,Uint16); 					// Enviar para um canal apenas (canal(0-3), dados)
extern void enviar_dac_spi_4Canais(Uint16,Uint16,Uint16,Uint16); 	//Envia para os 4 canais
long DelaySPI;  													// Vari�vel utilizada no delay da tranmiss�o

//////////////////////////////////////////////////////
///	  PROT�TIPO DAS FUN��ES DA DAC VIA SCI-RS232	//
/// 	  Obs.: Fun��es no arquivo COM_SERIAL.C	   ///
//////////////////////////////////////////////////////
extern void SCIA_TX(void);
extern void SCIA_RX(void);
char message[8]="12345678";                             			//serial
char message2[4];                                       			//serial
int index =0;			                                			//serial
Uint16 buffer;

///////////////////////////////////////////////////////
//////VARI�VEIS PARA MEDI��O DE VELOCIDADE		 //////
///////////////////////////////////////////////////////
extern float medir_velocidade(float pos, float dir);
 float pos_2;			// Posi��o final
 float pos_1=0;			// Posi��o da medida anterior
 int divF=0;				// Divisor de frenquencia para medi��o
 float vel_rpm=0;			// Velocidade medida em rpm


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$    PROT�TIPO DAS FUN��ES/INTERRUP��ES LOCAIS               $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$	  PROT�TIPO DAS FUN��ES PARA EXEXUTAR TESTES	$$
//$ 	  Obs.: TESTES NA PLACA VERSAO V4		    $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

void TESTA_LEDS(void);     											// FUN��O PARA TESTAR LEDS
void TESTA_BOTOES(void);   											// FUN��O PARA BOTOES
void TESTA_RELES(void);												//
void TESTA_DAC_SPI(void);											//
void TESTA_PWM(void);												//
void TESTA_BKR_RST(void);											//
void TESTA_RS232(void);												//
void TESTA_SAIDAS_ISOLADAS(void);									//
void TESTA_ENTRADAS_ANALOGICAS(void);								//
void TESTA_ENCODER(void);								//

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$	  PROT�TIPO DAS INTERRUP��ES UTILIZADAS 		$$
//$ 	  Obs.: TESTES NA PLACA VERSAO V4		    $$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

interrupt void cpu_timer0_isr(void); 								// Prototype for Timer 0 Interrupt Service Routine
interrupt void adc_isr(void);		 								// ADC  End of Sequence ISR


//################################################################
//#				DEFINI��O DAS VARI�VEIL GLOBAIS				     #
//#																 #
//################################################################


//#######  VARI�VEIS MEDIDAS NOS CANAIS ANAL�GICOS #######
float Ix=0,Iy=0,Iz=0, Icc2=0,Vcc2=0,Vx=0, Vy=0, Vz=0;				//Grandezas medidas entrada esquerda
float Iu=0,Iv=0,Iw=0, Icc1=0,Vcc1=0,Vu=0, Vv=0, Vw=0;				//Grandezas medidas entrada direita

//#######  MEDI��O DE VELOCIDADE #######
float pos=0, dir=0;
float vel_rpm_inst=0;			// Velocidade medida em rpm
int16 C=0;
////// Prot�tipo das fun��es neste arquivo//////

//#############################################################################################
//											C�DIGO PRINCIPAL
//#############################################################################################

void main(void)

 {
	InitSysCtrl();								// Basic Core Init from DSP2833x_SysCtrl.c
	Gpio_setup1();								// Configura os pinos de GPIO de acordo com a liga�a� na placa
	DINT;										// Disable all interrupts

	InitPieCtrl();								// basic setup of PIE table; from DSP2833x_PieCtrl.c
	InitPieVectTable();							// default ISR's in PIE

	EALLOW;  									// This is needed to wrifgte to EALLOW protected registers
	PieVectTable.ADCINT = &adc_isr; 			// Vetor interrup�a� do AD
	//PieVectTable.EPWM1_TZINT = &ePWM1_TZ_isr;
	EDIS;    									// This is needed to disable write to EALLOW protected registers

	InitCpuTimers();							// Inicializa CPU Timer0, 1 and 2
	ConfigCpuTimer(&CpuTimer0,150,10000);		// ConfiguraCPU CLOCK and timer

////////// COTNROLE DAS  INTERRU��ES ESPEC�FICAS
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1;			// ADC

////////// INTERRUP��O GERAL//////////////////////////
	IER |= 3;
	EINT;
	ERTM;
	CpuTimer0Regs.TCR.bit.TSS = 0;

/////////////// Configura��o do canais anal�gicos de entrada/////
	 InitAdc();   								// Inicializa perif�rico ADC
     Setup_ADC(); 								// Configura canais de de ADC e interrup��es para leitura
///// Configura��o do canais de PWM e TZ/////
	 Setup_ePWM();								// Configura canais de PWM
///// Configura��o do ENCODER  GPIO EQEP/////
	 Setup_EQEP();								// Configura encoder quadratura 	EQEP
//////////CONFIG  DA COMUNICA��O POR RS232//////////////////////
	 SCIA_init();  								// Initializa e configura  SCIA- RS232
//////////CONFIGURA��O DA COMUNICA��O POR RS232//////////////////////
	 spi_fifo_init();							// Configura SPI-FIFO DA DAC
	 spi_init();								// Inicializa SPI-FIFO DA DAC


//############################################################################################LOOP CENTRAL##############################################################################

		 while(1){
		//TESTA_LEDS();
		//TESTA_BOTOES();
		TESTA_RS232();
		//TESTA_RELES();
	   // TESTA_DAC_SPI();
        TESTA_PWM();
	    // TESTA_BKR_RST();
		//TESTA_SAIDAS_ISOLADAS();
		//TESTA_ENCODER();
	    //TESTA_ENTRADAS_ANALOGICAS();
	 	}

}
//###########################################################################LOOP CENTRAL###############################################################################################







//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_LEDS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO LEDS 1-4 19-21  DA PLACA ligados aos GPIO 67,69,71,71	   %
//OS LEDS PISCA 20 VEZES A CADA CHAMADA  DESTA FUN��O										   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_LEDS(void){
	int cont=0;
	float flag;
	while(cont<20){
if(flag > 800000)
			 {
	LIGALED1
	LIGALED2
	//LIGALED3
	//LIGALED4
			 }
else{
	DESLIGALED1
	DESLIGALED2
	//DESLIGALED3
	//DESLIGALED4
			 }

if(flag >= 1600000)      {
	flag = 0;
	cont = cont +1;
						 }

			 flag = flag +1;
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
LIGALED1
//LIGALED3
			}
else{
  DESLIGALED1
  //DESLIGALED3
	}

if(BOT2==1){
	LIGALED2
	//LIGALED4
		   }
else{
	DESLIGALED2
	//DESLIGALED4
    }
if(flag >= 1600000){
flag = 0;
cont = cont +1;
				   }
			 flag = flag +1;
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
				 DESLIGARELE1
				 DESLIGARELE2

			 }
			 else{
				 LIGARELE1
				 LIGARELE2

			 }

			 if(flag >= 1600000)     //9000 //11000
						 {
							 flag = 0;
							 cont = cont +1;
						 }

			 flag = flag +1;
}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_DAC_SPI%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a DAC por SPI 															   %
// S�O CRIADAS SAIDAS EM RAMPAS QUE S�O ENVIADAS PARA OS CANAIS DA DAC						   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_DAC_SPI(void){
	int cont=0;
	float flag;
	int i=0;
	while(cont<20){
		 {
C=C+1;


enviar_dac_spi_4Canais(C,C,C,C);


    for (i = 0; i < 400; i++) {}



//enviar_dac_spi_uni(3,C);
//enviar_dac_spi_4Canais(C,C,C,C);
if(C==4095){
C=0;
}
			 if(flag >= 16000)     //9000 //11000
						 {
				 flag = 0;
							 cont = cont +1;
						 }
			 flag = flag +1;
}
}
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_PWN%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os canais os 12 canais ePWM												   %
// OS PWM 1-6 s�o acionados com indice de modula��o (m) que varia de de-1 at�1				   %
// Caso o bot�o B1 seja acionado o Trip zone dos sensores � acionado					       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void TESTA_PWM(void){
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

while(cont<20){
	if(m>1){
		m=-1;
		cont = cont +1;
		}
	else{
		m=m+0.000001;}

//#######################################ATUALIZA PWM 1-3
	EPwm1Regs.CMPA.half.CMPA = 2500*(0.5*(1+m));  //EPwm1A  m
	EPwm1Regs.CMPB = EPwm1Regs.CMPA.half.CMPA;	   //EPwm1B
	EPwm2Regs.CMPA.half.CMPA = 2500*(0.5*(1+m));  //EPwm2A
	EPwm2Regs.CMPB = EPwm2Regs.CMPA.half.CMPA;     //EPwm2B
	EPwm3Regs.CMPA.half.CMPA = 2500*(0.5*(1+m));  //EPwm3A
	EPwm3Regs.CMPB = EPwm3Regs.CMPA.half.CMPA;     //EPwm3B
//#######################################ATUALIZA PWM4-6
	EPwm4Regs.CMPA.half.CMPA = 2500*(0.5*(1+m)); //EPwm4A mar
    EPwm4Regs.CMPB = EPwm4Regs.CMPA.half.CMPA;	   //EPwm4B
	EPwm5Regs.CMPA.half.CMPA = 2500*(0.5*(1+m)); //EPwm5A
	EPwm5Regs.CMPB = EPwm5Regs.CMPA.half.CMPA;     //EPwm5B
	EPwm6Regs.CMPA.half.CMPA = 2500*(0.5*(1+m)); //EPwm6A
	EPwm6Regs.CMPB = EPwm6Regs.CMPA.half.CMPA;     //EPwm6B

	if(BOT1==1){
		LISGATZSENSORES
				}
	else{
		DESLISGATZSENSORES
				 }
			}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS_RESET/BKR%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa as sa�das de reset e de break dos VSCs									   %
// Caso o bot�o B1 seja acionado as saidas RESETVSC(gpio62) E RESETVSC2 s�o acionadas		   %
// Caso o bot�o B2 seja acionado as saidas BRK e BKR2 s�o acionadas 					       %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_BKR_RST(void){
	int cont=0;
	float flag;

	while(cont<20){
if(BOT1==1){
	LIGARESETVSC
	LIGARESETVSC2
	LISGATZSENSORES
			}
else{
	DESLIGARESETVSC
	DESLIGARESETVSC2
	DESLISGATZSENSORES
			 }

if(BOT2==1){
	LIGABKR
	LIGABKR2
		   }
else{
	DESLIGABKR
	DESLIGABKR2

    }
if(flag >= 1600000) {
	flag = 0;
	 cont = cont +1;
					 }

 flag = flag +1;
}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA_SAIDAS ISOLADAS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa os pinos de GPIO 27,21,61,60 que controlam as SAIDAS ISOLADAS1-4		   %
//AS SAIDAS PISCAM 20 VEZES A CADA CHAMADA  DESTA FUN��O									   %
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


void TESTA_SAIDAS_ISOLADAS(void){
	int cont=0;
	float flag;

	while(cont<20){
		if(flag > 800000) {
			LIGASAIDAISO1
			LIGASAIDAISO2
			LIGASAIDAISO3
			LIGASAIDAISO4
			 }
			 else{
			DESLIGASAIDAISO1
			DESLIGASAIDAISO2
			DESLIGASAIDAISO3
			DESLIGASAIDAISO4
			 }
		if(flag >= 1600000){
			flag = 0;
			cont = cont +1;
						   }
			 flag = flag +1;
}
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA CANAIS DE ENTRADAS ANAL�GICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde s�o lidos o 16 canais anal�gicos			%		   	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_ENTRADAS_ANALOGICAS(void){
	IER |= 3;
	EINT;
	ERTM;

	enviar_dac_spi_4Canais(Ix,Iy,Iz,Icc2);
	//enviar_dac_spi_4Canais(Vx,Vy,Vz,Vcc2);

	//enviar_dac_spi_4Canais(	Iu,Iv,Iw,Icc1);
	//enviar_dac_spi_4Canais(Vu,Vv,Vw,Vcc1);





}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA ENCODER EQEP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o ativa a interrup��o do AD adc_isr onde � medida a velocidade atraves da fun��o	%
// medir_velocidade(angulo, dire��o)					 								      	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_ENCODER(void){
	IER |= 3;
	EINT;
	ERTM;
	enviar_dac_spi_4Canais(pos*11.37,pos*11.37,vel_rpm*2.04,vel_rpm*(-2.04));
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%TESTA RS232 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//Esta fun��o testa a comunica��o por RS232    SCI-A on GPIO35 - GPIO36                         %
// Envia a mensagem TX-OK 20 vezes e espera a chegada do caracter  T definido na fun�� SCI_RX   %				 								      	%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void TESTA_RS232(void){
	int cont=0;
	float flag;

	while(cont<20){
		if(flag >= 1600000) {
			SCIA_RX();
			if (buffer == 'T'){
				buffer=0;
			    sprintf(message,"RS232OK!");
				SCIA_TX();
							 }
							}
		flag = flag +1;
                   }
}




//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%INTERRUP��O ADC%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

interrupt void  adc_isr(void)
{

//#####################################MEDI��ES#####################################################

// Sensores lado esquerdo da placa
	// Ix, Iy e Iz e Icc2
	Ix=filtro_1(AdcMirror.ADCRESULT0);
	Iy=(AdcMirror.ADCRESULT1);
	Iz=filtro_3(AdcMirror.ADCRESULT2);
	Icc2=(AdcMirror.ADCRESULT3);

	// Vx, Vy e Vz Vcc2
	Vx=filtro_2(AdcMirror.ADCRESULT4);
	Vy=(AdcMirror.ADCRESULT6);
	Vz=filtro_4(AdcMirror.ADCRESULT5);
	Vcc2=(AdcMirror.ADCRESULT7);

// Sensores lado direito da placa

	// Iu, Iv e Iw e Icc1
	Iu=(AdcMirror.ADCRESULT8);
	Iv=(AdcMirror.ADCRESULT9);
	Iw=(AdcMirror.ADCRESULT12);
	Icc1=(AdcMirror.ADCRESULT15);

	// Vu, Vz e Vw Vcc1
	Vu=((AdcMirror.ADCRESULT10));
	Vv=(AdcMirror.ADCRESULT11);
	Vw=(AdcMirror.ADCRESULT13);
	Vcc1=(AdcMirror.ADCRESULT14);


//############################MEDI��O DE VELOCIDADE#################################################

	dir = EQep1Regs.QEPSTS.bit.QDF;
	pos = 0.025*(EQep1Regs.QPOSCNT);
	vel_rpm=medir_velocidade(pos,dir);

//	enviar_dac_spi_4Canais(Vx,Vy,Vz,Vz);

//TRATAMENTO DA INTERRUP��O

  	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;       // Reset SEQ1
  	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;		// Clear INT SEQ1 bit
  	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%INTERRUP��O ADC%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%























